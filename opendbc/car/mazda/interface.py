#!/usr/bin/env python3
from math import exp

from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase, LatControlInputs, FRICTION_THRESHOLD, TorqueFromLateralAccelCallbackType
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.values import CAR, LKAS_LIMITS, MazdaSafetyFlags, MazdaFlags, GEN1, GEN2
from openpilot.common.params import Params
from opendbc.car import get_friction

NON_LINEAR_TORQUE_PARAMS = {
  CAR.MAZDA_3_2019: (3.8818, 0.8, 0.2, 0.3605),
}

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def torque_from_lateral_accel_siglin(self, latcontrol_inputs: LatControlInputs, torque_params: structs.CarParams.LateralTorqueTuning, lateral_accel_error: float,
                                      lateral_accel_deadzone: float, friction_compensation: bool, gravity_adjusted: bool) -> float:
    friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD, torque_params, friction_compensation)

    def sig(val):
      # https://timvieira.github.io/blog/post/2014/02/11/exp-normalize-trick
      if val >= 0:
        return 1 / (1 + exp(-val)) - 0.5
      else:
        z = exp(val)
        return z / (1 + z) - 0.5

    # The "lat_accel vs torque" relationship is assumed to be the sum of "sigmoid + linear" curves
    # An important thing to consider is that the slope at 0 should be > 0 (ideally >1)
    # This has big effect on the stability about 0 (noise when going straight)
    # ToDo: To generalize to other GMs, explore tanh function as the nonlinear
    non_linear_torque_params = NON_LINEAR_TORQUE_PARAMS.get(self.CP.carFingerprint)
    assert non_linear_torque_params, "The params are not defined"
    a, b, c = float(self.params.get("a", encoding='utf-8')), float(self.params.get("b", encoding='utf-8')), float(self.params.get("c", encoding='utf-8'))
    steer_torque = (sig(latcontrol_inputs.lateral_acceleration * a) * b) + (latcontrol_inputs.lateral_acceleration * c)
    return float(steer_torque) + friction

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    if self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
      return self.torque_from_lateral_accel_siglin
    else:
      return self.torque_from_lateral_accel_linear

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]
    p = Params()

    ret.radarUnavailable = True

    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_3_2019, CAR.MAZDA_CX_30, CAR.MAZDA_CX_50) and not ret.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    ret.centerToFront = ret.wheelbase * 0.41

    if p.get_bool("ManualTransmission"):
      ret.flags |= MazdaFlags.MANUAL_TRANSMISSION.value
      ret.transmissionType = structs.CarParams.TransmissionType.manual
    else:
      ret.transmissionType = structs.CarParams.TransmissionType.automatic

    if candidate in GEN1:
      ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.GEN1.value
      if p.get_bool("TorqueInterceptorEnabled"): # Torque Interceptor Installed
        ret.flags |= MazdaFlags.TORQUE_INTERCEPTOR.value
        ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.TORQUE_INTERCEPTOR.value
      if p.get_bool("RadarInterceptorEnabled"): # Radar Interceptor Installed
        ret.flags |= MazdaFlags.RADAR_INTERCEPTOR.value
        ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.RADAR_INTERCEPTOR.value
        ret.alphaLongitudinalAvailable = alpha_long
        ret.openpilotLongitudinalControl = True
        ret.radarUnavailable = False
        ret.startingState = True
        ret.longitudinalTuning.kpBP = [0., 5., 30.]
        ret.longitudinalTuning.kpV = [1.3, 1.0, 0.7]
        ret.longitudinalTuning.kiBP = [0., 5., 20., 30.]
        ret.longitudinalTuning.kiV = [0.36, 0.23, 0.17, 0.1]
      if p.get_bool("NoMRCC"): # No Mazda Radar Cruise Control; Missing CRZ_CTRL signal
        ret.flags |= MazdaFlags.NO_MRCC.value
        ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.NO_MRCC.value
      if p.get_bool("NoFSC"):  # No Front Sensing Camera
        ret.flags |= MazdaFlags.NO_FSC.value
        ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.NO_FSC.value

      ret.steerActuatorDelay = 0.1
      ret.enableBsm = True

    if candidate in GEN2:
      ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.GEN2.value
      ret.alphaLongitudinalAvailable = alpha_long
      ret.openpilotLongitudinalControl = True
      ret.stopAccel = -.5
      ret.vEgoStarting = .2
      ret.longitudinalActuatorDelay = 0.35 # gas is 0.25s and brake looks like 0.5
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [0.0, 0.0, 0.0]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.1, 0.1]
      ret.startingState = True
      ret.steerActuatorDelay = 0.01

    return ret
