#pragma once

#include "opendbc/safety/safety_declarations.h"

// CAN msgs we care about
/********** GEN1 msgs **********/
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U
#define TI_STEER_TORQUE     0x24AU
// Radar
#define MAZDA_CRZ_INFO      0x21BU
#define MAZDA_CRZ_CTRL      0x21CU
#define MAZDA_RADAR_361     0x361U
#define MAZDA_RADAR_362     0x362U
#define MAZDA_RADAR_363     0x363U
#define MAZDA_RADAR_364     0x364U
#define MAZDA_RADAR_365     0x365U
#define MAZDA_RADAR_366     0x366U
#define MAZDA_RADAR_499     0x499U
/************* GEN2 msgs *************/
#define MAZDA_2019_BRAKE          0x43FU  // main bus
#define MAZDA_2019_GAS            0x202U  // camera bus DBC: ENGINE_DATA
#define MAZDA_2019_CRUISE         0x44AU  // main bus. DBC: CRUISE_STATE
#define MAZDA_2019_SPEED          0x217U  // camera bus. DBC: SPEED
#define MAZDA_2019_STEER_TORQUE   0x24BU  // aux bus. DBC: TI_FEEDBACK
#define MAZDA_2019_CRZ_BTNS       0x9DU   // rx on main tx on camera. DBC: CRZ_BTNS
#define MAZDA_2019_ACC            0x220U  // main bus. DBC: ACC
#define MAZDA_TI_LKAS             0x249U

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_AUX  1
#define MAZDA_CAM  2

// param flag masks
const int FLAG_GEN1 = 1;
const int FLAG_GEN2 = 2;
const int FLAG_TORQUE_INTERCEPTOR = 4;
const int FLAG_RADAR_INTERCEPTOR = 8;
const int FLAG_NO_FSC = 16;
const int FLAG_NO_MRCC = 32;

bool gen1 = false;
bool gen2 = false;
bool torque_interceptor = false;
bool radar_interceptor = false;
bool no_fsc = false;
bool no_mrcc = false;


// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if ((int)msg->bus == MAZDA_MAIN) {
    if (gen1) {
      if (msg->addr == MAZDA_ENGINE_DATA) {
        // sample speed: scale by 0.01 to get kph
        int speed = (msg->data[2] << 8) | msg->data[3];
        vehicle_moving = speed > 10; // moving when speed > 0.1 kph
      }

      if (msg->addr == MAZDA_STEER_TORQUE && !torque_interceptor) {
        int torque_driver_new = msg->data[0] - 127U;
        // update array of samples
        update_sample(&torque_driver, torque_driver_new);
      }

      // enter controls on rising edge of ACC, exit controls on ACC off
      if (msg->addr == MAZDA_CRZ_CTRL && !no_mrcc && !radar_interceptor) {
        acc_main_on = GET_BIT(msg, 17U);
        bool cruise_engaged = msg->data[0] & 0x8U;
        pcm_cruise_check(cruise_engaged);
      }

      if (msg->addr == MAZDA_ENGINE_DATA) {
        gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
      }

      if (msg->addr == MAZDA_PEDALS) {
        brake_pressed = (msg->data[0] & 0x10U);
        if (radar_interceptor || no_mrcc) {
          // acc_main_on = GET_BIT(to_push, 17U); TODO
          bool cruise_engaged = msg->data[0] & 0x8U;
          pcm_cruise_check(cruise_engaged);
        }
      }
    }

    if (gen2) {
      if (msg->addr == MAZDA_2019_BRAKE) {
        brake_pressed = (msg->data[5] & 0x4U);
      }

      if (msg->addr == MAZDA_2019_CRUISE) {
        acc_main_on = true;
        bool cruise_engaged = msg->data[0] & 0x20U;
        bool pre_enable = msg->data[0] & 0x40U;
        pcm_cruise_check((cruise_engaged || pre_enable));
      }
    }
  }

  if ((msg->bus == MAZDA_AUX)) {

    if (msg->addr == TI_STEER_TORQUE && gen1 && torque_interceptor) {
      int torque_driver_new = msg->data[0] - 127;
      update_sample(&torque_driver, torque_driver_new);
    }

    if (msg->addr == MAZDA_2019_STEER_TORQUE && gen2) {
      update_sample(&torque_driver, (int16_t)(msg->data[0] << 8 | msg->data[1]));
    }
  }

  if ((msg->bus == MAZDA_CAM)) {
    if (gen2) {

      if (msg->addr == MAZDA_2019_GAS) {
        gas_pressed = (msg->data[4] || ((msg->data[5] & 0xC0U)));
      }

      if (msg->addr == MAZDA_2019_SPEED) {
        // sample speed: scale by 0.01 to get kph
        int speed = (msg->data[4] << 8) | (msg->data[5]);
        vehicle_moving = (speed > 10);  // moving when speed > 0.1 kph
      }
    }
  }
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    .max_torque = 800,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  const TorqueSteeringLimits MAZDA_2019_STEERING_LIMITS = {
    .max_torque = 8000,
    .max_rate_up = 45,
    .max_rate_down = 80,
    .max_rt_delta = 1688,  // (45*100hz*250000/1000000)*1.5
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 1400,
    .type = TorqueDriverLimited,
  };

  bool tx = true;
  // Check if msg is sent on the main BUS
  if (msg->bus == (unsigned char)MAZDA_MAIN) {
    // steer cmd checks
    if (msg->addr == MAZDA_LKAS) {
      int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

      if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_STEERING_LIMITS)) {
        tx = false;
      }
    }

    // cruise buttons check
    if (msg->addr == MAZDA_CRZ_BTNS) {
      // allow resume spamming while controls allowed, but
      // only allow cancel while controls not allowed
      bool cancel_cmd = (msg->data[0] == 0x1U);
      if (!controls_allowed && !cancel_cmd) {
        tx = false;
      }
    }
  }
  if (gen2 && (msg->bus == MAZDA_AUX) && (msg->addr == MAZDA_TI_LKAS)) {
    int desired_torque = (int16_t)((msg->data[0] << 8) | msg->data[1]);  // signal is signed
    if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_2019_STEERING_LIMITS)) {
      tx = false;
    }
  }

  return tx;
}

static safety_config mazda_init(uint16_t param) {
  static const CanMsg MAZDA_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true}};
  static const CanMsg MAZDA_TI_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_TI_LKAS, 1, 8, .check_relay = false}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true}};
  static const CanMsg MAZDA_RI_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
                                    {MAZDA_CRZ_CTRL, 0, 8, .check_relay = true}, {MAZDA_CRZ_INFO, 0, 8, .check_relay = true}, {MAZDA_RADAR_361, 0, 8, .check_relay = true}, {MAZDA_RADAR_362, 0, 8, .check_relay = true},
                                    {MAZDA_RADAR_363, 0, 8, .check_relay = true}, {MAZDA_RADAR_364, 0, 8, .check_relay = true}, {MAZDA_RADAR_365, 0, 8, .check_relay = true}, {MAZDA_RADAR_366, 0, 8, .check_relay = true},
                                    {MAZDA_RADAR_499, 0, 8, .check_relay = true}};
  static const CanMsg MAZDA_TI_RI_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_TI_LKAS, 1, 8, .check_relay = false}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
                                  {MAZDA_CRZ_CTRL, 0, 8, .check_relay = true}, {MAZDA_CRZ_INFO, 0, 8, .check_relay = true}, {MAZDA_RADAR_361, 0, 8, .check_relay = true}, {MAZDA_RADAR_362, 0, 8, .check_relay = true},
                                  {MAZDA_RADAR_363, 0, 8, .check_relay = true}, {MAZDA_RADAR_364, 0, 8, .check_relay = true}, {MAZDA_RADAR_365, 0, 8, .check_relay = true}, {MAZDA_RADAR_366, 0, 8, .check_relay = true},
                                  {MAZDA_RADAR_499, 0, 8, .check_relay = true}};

  static const CanMsg MAZDA_2019_TX_MSGS[] = {{MAZDA_TI_LKAS, 1, 8, .check_relay = false}, {MAZDA_2019_ACC, 2, 8, .check_relay = true}};

  static RxCheck mazda_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };
  static RxCheck mazda_ti_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{TI_STEER_TORQUE,    1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };
  static RxCheck mazda_2019_rx_checks[] = {
    {.msg = {{MAZDA_2019_BRAKE,         0, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_GAS,           2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_CRUISE,        0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_SPEED,         2, 8, 30U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_STEER_TORQUE,  1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };
  return BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
};
