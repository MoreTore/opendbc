import copy
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mazda.values import DBC, LKAS_LIMITS, MazdaFlags, TI_STATE, CarControllerParams

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]
    if CP.flags & MazdaFlags.MANUAL_TRANSMISSION:
      self.shifter_values = can_define.dv["MANUAL_GEAR"]["GEAR"]

    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.lkas_allowed_speed = False
    self.cam_lkas = 0
    self.params = CarControllerParams(CP)

    self.distance_button = 0

    self.ti_ramp_down = False
    self.ti_version = 1
    self.ti_state = TI_STATE.RUN
    self.ti_violation = 0
    self.ti_error = 0
    self.ti_lkas_allowed = False

    self.shifting = False
    self.torque_converter_lock = True

  def update(self, can_parsers) -> structs.CarState:
    if self.CP.flags & MazdaFlags.GEN2:
      return self.update_gen2(can_parsers)

    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_body = can_parsers[Bus.body]

    ret = structs.CarState()

    prev_distance_button = self.distance_button
    self.distance_button = cp.vl["CRZ_BTNS"]["DISTANCE_LESS"]

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FL"],
      cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"],
      cp.vl["WHEEL_SPEEDS"]["RR"],
    )

    # Match panda speed reading
    speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    ret.standstill = speed_kph <= .1

    can_gear = int(cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.genericToggle = bool(cp.vl["BLINK_INFO"]["HIGH_BEAMS"])
    ret.leftBlindspot = cp.vl["BSM"]["LEFT_BS_STATUS"] != 0
    ret.rightBlindspot = cp.vl["BSM"]["RIGHT_BS_STATUS"] != 0
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      ret.steeringTorque = cp_body.vl["TI_FEEDBACK"]["TI_TORQUE_SENSOR"]

      self.ti_version = cp_body.vl["TI_FEEDBACK"]["VERSION_NUMBER"]
      self.ti_state = cp_body.vl["TI_FEEDBACK"]["STATE"] # DISCOVER = 0, OFF = 1, DRIVER_OVER = 2, RUN=3
      self.ti_violation = cp_body.vl["TI_FEEDBACK"]["VIOL"] # 0 = no violation
      self.ti_error = cp_body.vl["TI_FEEDBACK"]["ERROR"] # 0 = no error
      if self.ti_version > 1:
        self.ti_ramp_down = (cp_body.vl["TI_FEEDBACK"]["RAMP_DOWN"] == 1)

      ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.TI_STEER_THRESHOLD
      self.ti_lkas_allowed = not self.ti_ramp_down and self.ti_state == TI_STATE.RUN
    else:
      ret.steeringTorque = cp.vl["STEER_TORQUE"]["STEER_TORQUE_SENSOR"]
      ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]

    ret.steeringTorqueEps = cp.vl["STEER_TORQUE"]["STEER_TORQUE_MOTOR"]
    ret.steeringRateDeg = cp.vl["STEER_RATE"]["STEER_ANGLE_RATE"]

    # TODO: this should be from 0 - 1.
    ret.brakePressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1
    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"]

    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["DRIVER_SEATBELT"] == 0
    ret.doorOpen = any([cp.vl["DOORS"]["FL"], cp.vl["DOORS"]["FR"],
                        cp.vl["DOORS"]["BL"], cp.vl["DOORS"]["BR"]])

    # TODO: this should be from 0 - 1.
    ret.gasPressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0

    # Either due to low speed or hands off
    lkas_blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] == 1

    if self.CP.minSteerSpeed > 0:
      # LKAS is enabled at 52kph going up and disabled at 45kph going down
      # wait for LKAS_BLOCK signal to clear when going up since it lags behind the speed sometimes
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED and not lkas_blocked:
        self.lkas_allowed_speed = True
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    # TODO: the signal used for available seems to be the adaptive cruise signal, instead of the main on
    #       it should be used for carState.cruiseState.nonAdaptive instead
    ret.cruiseState.standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS

    if self.CP.flags & MazdaFlags.RADAR_INTERCEPTOR:
      self.crz_info = copy.copy(cp_cam.vl["CRZ_INFO"])
      self.crz_cntr = copy.copy(cp_cam.vl["CRZ_CTRL"])
      self.cp_cam = cp_cam
      ret.cruiseState.enabled = cp.vl["PEDALS"]["ACC_ACTIVE"] == 1
      ret.cruiseState.available = cp.vl["PEDALS"]["CRZ_AVAILABLE"] == 1
    elif self.CP.flags & MazdaFlags.NO_MRCC:
      ret.cruiseState.enabled = cp.vl["PEDALS"]["ACC_ACTIVE"] == 1
      ret.cruiseState.available = cp.vl["PEDALS"]["CRZ_AVAILABLE"] == 1
    else:
      ret.cruiseState.available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
      ret.cruiseState.enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1

    # stock lkas should be on
    # TODO: is this needed?
    if not self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      if ret.cruiseState.enabled:
        if not self.lkas_allowed_speed and self.acc_active_last:
          self.low_speed_alert = True
        else:
          self.low_speed_alert = False
      ret.lowSpeedAlert = self.low_speed_alert

    # Check if LKAS is disabled due to lack of driver torque when all other states indicate
    # it should be enabled (steer lockout). Don't warn until we actually get lkas active
    # and lose it again, i.e, after initial lkas activation
    ret.steerFaultTemporary = self.lkas_allowed_speed and lkas_blocked and not self.ti_lkas_allowed

    self.acc_active_last = ret.cruiseState.enabled

    self.crz_btns_counter = cp.vl["CRZ_BTNS"]["CTR"]

    # camera signals
    if not self.CP.flags & MazdaFlags.NO_FSC:
      ret.invalidLkasSetting = cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0
      self.lkas_disabled = cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0 if not self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR else False
      self.cam_lkas = cp_cam.vl["CAM_LKAS"]
      self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]
      ret.steerFaultPermanent = cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1 if not self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR else False
    self.cp_cam = cp_cam
    self.cp = cp

    # TODO: add button types for inc and dec
    ret.buttonEvents = create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    return ret

  def update_gen2(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_body = can_parsers[Bus.body]
    ret = structs.CarState()

    ret.wheelSpeeds = self.get_wheel_speeds(
        cp_cam.vl["WHEEL_SPEEDS"]["FL"],
        cp_cam.vl["WHEEL_SPEEDS"]["FR"],
        cp_cam.vl["WHEEL_SPEEDS"]["RL"],
        cp_cam.vl["WHEEL_SPEEDS"]["RR"],
    )

    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw) # Doesn't match cluster speed exactly

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(100, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    ret.engineRpm = cp_cam.vl["ENGINE_DATA"]["RPM"]
    #self.shifting = cp_cam.vl["GEAR"]["SHIFT"]
    #self.torque_converter_lock = cp_cam.vl["GEAR"]["TORQUE_CONVERTER_LOCK"]

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    ret.steeringRateDeg = cp.vl["STEER"]["STEER_RATE"]

    ret.steeringTorque = cp_body.vl["TI_FEEDBACK"]["STEER_TORQUE_SENSOR"]
    ret.gas = cp_cam.vl["ENGINE_DATA"]["PEDAL_GAS"]

    unit_conversion = CV.MPH_TO_MS if cp.vl["SYSTEM_SETTINGS"]["IMPERIAL_UNIT"] else CV.KPH_TO_MS

    ret.steeringPressed = abs(ret.steeringTorque) > self.params.STEER_DRIVER_ALLOWANCE
    if self.CP.flags & MazdaFlags.MANUAL_TRANSMISSION:
      can_gear = int(cp_cam.vl["MANUAL_GEAR"]["GEAR"])
    else:
      can_gear = int(cp_cam.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.gasPressed = ret.gas > 0
    ret.seatbeltUnlatched = False # Cruise will not engage if seatbelt is unlatched (handled by car)
    ret.doorOpen = False # Cruise will not engage if door is open (handled by car)
    ret.brakePressed = cp.vl["BRAKE_PEDAL"]["BRAKE_PRESSED"] == 1
    ret.brake = .1
    ret.steerFaultPermanent = False # TODO locate signal. Car shows light on dash if there is a fault
    ret.steerFaultTemporary = False # TODO locate signal. Car shows light on dash if there is a fault

    ret.standstill = cp_cam.vl["SPEED"]["SPEED"] * unit_conversion < 0.1
    ret.cruiseState.speed = cp.vl["CRUZE_STATE"]["CRZ_SPEED"] * unit_conversion
    ret.cruiseState.enabled = (cp.vl["CRUZE_STATE"]["CRZ_STATE"] >= 2)
    ret.cruiseState.available = (cp.vl["CRUZE_STATE"]["CRZ_STATE"] != 0)
    ret.cruiseState.standstill = ret.standstill if not self.CP.openpilotLongitudinalControl else False

    self.cp = cp
    self.cp_cam = cp_cam
    self.acc = copy.copy(cp.vl["ACC"])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.body: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
