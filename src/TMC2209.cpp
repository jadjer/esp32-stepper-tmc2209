// Copyright 2025 Pavel Suprunov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ----------------------------------------------------------------------------
// TMC2209.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#include "motor/driver/TMC2209.hpp"

namespace motor::driver {

TMC2209::TMC2209(std::uint8_t const uartPort, SlaveAddress const slaveAddress, std::int8_t const rxPin, std::int8_t const txPin)
    : m_slaveAddress(slaveAddress),

      m_stepPin(21, gpio::PIN_LEVEL_LOW),
      m_directionPin(19, gpio::PIN_LEVEL_LOW),
      m_enablePin(nullptr),

      m_coolStepEnabled(false),
      m_toff(TOFF_DEFAULT),
      m_serial(uartPort),

      m_pwmConfig(),
      m_coolConfig(),
      m_globalConfig(),
      m_driverCurrent(),
      m_chopperConfig()

{
  m_serial.begin(115200, SERIAL_8N1, rxPin, txPin);
  initialize();
}

void TMC2209::setDirection(motor::Direction direction) {
  switch (direction) {

  case motor::MOTOR_ROTATE_CW:
    m_directionPin.setLevel(gpio::PIN_LEVEL_LOW);
    break;
  case motor::MOTOR_ROTATE_CCW:
    m_directionPin.setLevel(gpio::PIN_LEVEL_HIGH);
    break;
  }
}

void TMC2209::setMicroSteps(motor::MotorSteps microSteps) {
  switch (microSteps) {

  case motor::MOTOR_FULL_STEP:
    m_chopperConfig.mres = MRES_001;
    break;
  case motor::MOTOR_PER_STEP_2_MICRO_STEPS:
    m_chopperConfig.mres = MRES_002;
    break;
  case motor::MOTOR_PER_STEP_4_MICRO_STEPS:
    m_chopperConfig.mres = MRES_004;
    break;
  case motor::MOTOR_PER_STEP_8_MICRO_STEPS:
    m_chopperConfig.mres = MRES_008;
    break;
  case motor::MOTOR_PER_STEP_16_MICRO_STEPS:
    m_chopperConfig.mres = MRES_016;
    break;
  case motor::MOTOR_PER_STEP_32_MICRO_STEPS:
    m_chopperConfig.mres = MRES_032;
    break;
  case motor::MOTOR_PER_STEP_64_MICRO_STEPS:
    m_chopperConfig.mres = MRES_064;
    break;
  case motor::MOTOR_PER_STEP_128_MICRO_STEPS:
    m_chopperConfig.mres = MRES_128;
    break;
  case motor::MOTOR_PER_STEP_256_MICRO_STEPS:
    m_chopperConfig.mres = MRES_256;
    break;
  }

  writeStoredChopperConfig();
}

void TMC2209::setHardwareEnablePin(std::uint8_t hardwareEnablePin) {
  m_enablePin = std::make_unique<gpio::OutputPin>(hardwareEnablePin, gpio::PIN_LEVEL_HIGH);
}

void TMC2209::enable() {
  if (m_enablePin != nullptr) {
    m_enablePin->setLevel(gpio::PIN_LEVEL_LOW);
  }

  m_chopperConfig.toff = m_toff;
  writeStoredChopperConfig();
}

void TMC2209::disable() {
  if (m_enablePin != nullptr) {
    m_enablePin->setLevel(gpio::PIN_LEVEL_LOW);
  }

  m_chopperConfig.toff = TOFF_DISABLE;
  writeStoredChopperConfig();
}

void TMC2209::setRunCurrent(std::uint8_t percent) {
  std::uint8_t run_current = percentToCurrentSetting(percent);
  m_driverCurrent.irun = run_current;
  writeStoredDriverCurrent();
}

void TMC2209::setHoldCurrent(std::uint8_t percent) {
  std::uint8_t hold_current = percentToCurrentSetting(percent);

  m_driverCurrent.ihold = hold_current;
  writeStoredDriverCurrent();
}

void TMC2209::setHoldDelay(std::uint8_t percent) {
  std::uint8_t hold_delay = percentToHoldDelaySetting(percent);

  m_driverCurrent.iholddelay = hold_delay;
  writeStoredDriverCurrent();
}

void TMC2209::setAllCurrentValues(std::uint8_t run_current_percent,
                                  std::uint8_t hold_current_percent,
                                  std::uint8_t hold_delay_percent) {
  std::uint8_t run_current = percentToCurrentSetting(run_current_percent);
  std::uint8_t hold_current = percentToCurrentSetting(hold_current_percent);
  std::uint8_t hold_delay = percentToHoldDelaySetting(hold_delay_percent);

  m_driverCurrent.irun = run_current;
  m_driverCurrent.ihold = hold_current;
  m_driverCurrent.iholddelay = hold_delay;
  writeStoredDriverCurrent();
}

void TMC2209::setRMSCurrent(std::uint16_t const mA, float const rSense, float const holdMultiplier) {
  // Taken from https://github.com/teemuatlut/TMCStepper/blob/74e8e6881adc9241c2e626071e7328d7652f361a/src/source/TMCStepper.cpp#L41.

  std::uint8_t CS = 32.0 * 1.41421 * mA / 1000.0 * (rSense + 0.02) / 0.325 - 1;
  // If Current Scale is too low, turn on high sensitivity R_sense and calculate again
  if (CS < 16) {
    enableVSense();
    CS = 32.0 * 1.41421 * mA / 1000.0 * (rSense + 0.02) / 0.180 - 1;
  } else {// If CS >= 16, turn off high_sense_r
    disableVSense();
  }

  if (CS > 31) {
    CS = 31;
  }

  m_driverCurrent.irun = CS;
  m_driverCurrent.ihold = CS * holdMultiplier;
  writeStoredDriverCurrent();
}

void TMC2209::enableDoubleEdge() {
  m_chopperConfig.double_edge = DOUBLE_EDGE_ENABLE;
  writeStoredChopperConfig();
}

void TMC2209::disableDoubleEdge() {
  m_chopperConfig.double_edge = DOUBLE_EDGE_DISABLE;
  writeStoredChopperConfig();
}

void TMC2209::enableVSense() {
  m_chopperConfig.vsense = VSENSE_ENABLE;
  writeStoredChopperConfig();
}

void TMC2209::disableVSense() {
  m_chopperConfig.vsense = VSENSE_DISABLE;
  writeStoredChopperConfig();
}

void TMC2209::enableInverseMotorDirection() {
  m_globalConfig.shaft = 1;
  writeStoredGlobalConfig();
}

void TMC2209::disableInverseMotorDirection() {
  m_globalConfig.shaft = 0;
  writeStoredGlobalConfig();
}

void TMC2209::setStandstillMode(StandstillMode mode) {
  m_pwmConfig.freewheel = mode;
  writeStoredPwmConfig();
}

void TMC2209::enableAutomaticCurrentScaling() {
  m_pwmConfig.pwm_autoscale = STEPPER_DRIVER_FEATURE_ON;
  writeStoredPwmConfig();
}

void TMC2209::disableAutomaticCurrentScaling() {
  m_pwmConfig.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
  writeStoredPwmConfig();
}

void TMC2209::enableAutomaticGradientAdaptation() {
  m_pwmConfig.pwm_autograd = STEPPER_DRIVER_FEATURE_ON;
  writeStoredPwmConfig();
}

void TMC2209::disableAutomaticGradientAdaptation() {
  m_pwmConfig.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
  writeStoredPwmConfig();
}

void TMC2209::setPwmOffset(std::uint8_t pwm_amplitude) {
  m_pwmConfig.pwm_offset = pwm_amplitude;
  writeStoredPwmConfig();
}

void TMC2209::setPwmGradient(std::uint8_t pwm_amplitude) {
  m_pwmConfig.pwm_grad = pwm_amplitude;
  writeStoredPwmConfig();
}

void TMC2209::setPowerDownDelay(std::uint8_t power_down_delay) {
  write(REGISTER_TPOWERDOWN, power_down_delay);
}

void TMC2209::setReplyDelay(std::uint8_t reply_delay) {
  if (reply_delay > REPLY_DELAY_MAX) {
    reply_delay = REPLY_DELAY_MAX;
  }

  ReplyDelay replyDelayData{};
  replyDelayData.bytes = 0;
  replyDelayData.replydelay = reply_delay;
  write(REGISTER_REPLYDELAY, replyDelayData.bytes);
}

void TMC2209::moveAtVelocity(int32_t microsteps_per_period) {
  write(REGISTER_VACTUAL, microsteps_per_period);
}

void TMC2209::moveUsingStepDirInterface() {
  write(REGISTER_VACTUAL, VACTUAL_STEP_DIR_INTERFACE);
}

void TMC2209::enableStealthChop() {
  m_globalConfig.enable_spread_cycle = 0;
  writeStoredGlobalConfig();
}

void TMC2209::disableStealthChop() {
  m_globalConfig.enable_spread_cycle = 1;
  writeStoredGlobalConfig();
}

void TMC2209::setCoolStepDurationThreshold(std::uint32_t duration_threshold) {
  write(REGISTER_TCOOLTHRS, duration_threshold);
}

void TMC2209::setStealthChopDurationThreshold(std::uint32_t duration_threshold) {
  write(REGISTER_TPWMTHRS, duration_threshold);
}

void TMC2209::setStallGuardThreshold(std::uint8_t stall_guard_threshold) {
  write(REGISTER_SGTHRS, stall_guard_threshold);
}

void TMC2209::enableCoolStep(std::uint8_t const lowerThreshold, std::uint8_t const upperThreshold) {
  std::uint8_t const lowerThresholdConstrained = getConstrainedValue(lowerThreshold, SEMIN_MIN, SEMIN_MAX);
  m_coolConfig.semin = lowerThresholdConstrained;

  std::uint8_t const upperThresholdConstrained = getConstrainedValue(upperThreshold, SEMAX_MIN, SEMAX_MAX);
  m_coolConfig.semax = upperThresholdConstrained;

  write(REGISTER_COOLCONF, m_coolConfig.bytes);

  m_coolStepEnabled = true;
}

void TMC2209::disableCoolStep() {
  m_coolConfig.semin = SEMIN_OFF;
  write(REGISTER_COOLCONF, m_coolConfig.bytes);

  m_coolStepEnabled = false;
}

void TMC2209::setCoolStepCurrentIncrement(CurrentIncrement current_increment) {
  m_coolConfig.seup = current_increment;
  write(REGISTER_COOLCONF, m_coolConfig.bytes);
}

void TMC2209::setCoolStepMeasurementCount(MeasurementCount measurement_count) {
  m_coolConfig.sedn = measurement_count;
  write(REGISTER_COOLCONF, m_coolConfig.bytes);
}

void TMC2209::enableAnalogCurrentScaling() {
  m_globalConfig.i_scale_analog = 1;
  writeStoredGlobalConfig();
}

void TMC2209::disableAnalogCurrentScaling() {
  m_globalConfig.i_scale_analog = 0;
  writeStoredGlobalConfig();
}

void TMC2209::useExternalSenseResistors() {
  m_globalConfig.internal_rsense = 0;
  writeStoredGlobalConfig();
}

void TMC2209::useInternalSenseResistors() {
  m_globalConfig.internal_rsense = 1;
  writeStoredGlobalConfig();
}

// bidirectional methods

std::uint8_t TMC2209::getVersion() {
  Input input{};
  input.bytes = read(REGISTER_IOIN);

  return input.version;
}

motor::MotorSteps TMC2209::getMicroSteps() const {
  switch (m_chopperConfig.mres) {
  case MRES_001:
    return motor::MOTOR_FULL_STEP;
  case MRES_002:
    return motor::MOTOR_PER_STEP_2_MICRO_STEPS;
  case MRES_004:
    return motor::MOTOR_PER_STEP_4_MICRO_STEPS;
  case MRES_008:
    return motor::MOTOR_PER_STEP_8_MICRO_STEPS;
  case MRES_016:
    return motor::MOTOR_PER_STEP_16_MICRO_STEPS;
  case MRES_032:
    return motor::MOTOR_PER_STEP_32_MICRO_STEPS;
  case MRES_064:
    return motor::MOTOR_PER_STEP_64_MICRO_STEPS;
  case MRES_128:
    return motor::MOTOR_PER_STEP_128_MICRO_STEPS;
  case MRES_256:
  default:
    return motor::MOTOR_PER_STEP_256_MICRO_STEPS;
  }
}

bool TMC2209::isCommunicating() {
  return (getVersion() == VERSION);
}

bool TMC2209::isSetupAndCommunicating() {
  return serialOperationMode();
}

bool TMC2209::isCommunicatingButNotSetup() {
  return (isCommunicating() && (not isSetupAndCommunicating()));
}

bool TMC2209::hardwareDisabled() {
  Input input{};
  input.bytes = read(REGISTER_IOIN);

  return input.enn;
}

Settings TMC2209::getSettings() {
  Settings settings{};
  settings.is_communicating = isCommunicating();

  if (settings.is_communicating) {
    readAndStoreRegisters();

    settings.is_setup = m_globalConfig.pdn_disable;
    settings.software_enabled = (m_chopperConfig.toff > TOFF_DISABLE);
    settings.inverse_motor_direction_enabled = m_globalConfig.shaft;
    settings.stealth_chop_enabled = not m_globalConfig.enable_spread_cycle;
    settings.irun_percent = currentSettingToPercent(m_driverCurrent.irun);
    settings.irun_register_value = m_driverCurrent.irun;
    settings.ihold_percent = currentSettingToPercent(m_driverCurrent.ihold);
    settings.ihold_register_value = m_driverCurrent.ihold;
    settings.iholddelay_percent = holdDelaySettingToPercent(m_driverCurrent.iholddelay);
    settings.iholddelay_register_value = m_driverCurrent.iholddelay;
    settings.automatic_current_scaling_enabled = m_pwmConfig.pwm_autoscale;
    settings.automatic_gradient_adaptation_enabled = m_pwmConfig.pwm_autograd;
    settings.pwm_offset = m_pwmConfig.pwm_offset;
    settings.pwm_gradient = m_pwmConfig.pwm_grad;
    settings.cool_step_enabled = m_coolStepEnabled;
    settings.analog_current_scaling_enabled = m_globalConfig.i_scale_analog;
    settings.internal_sense_resistors_enabled = m_globalConfig.internal_rsense;
  } else {
    settings.is_setup = false;
    settings.software_enabled = false;
    settings.inverse_motor_direction_enabled = false;
    settings.stealth_chop_enabled = false;
    settings.irun_percent = 0;
    settings.irun_register_value = 0;
    settings.ihold_percent = 0;
    settings.ihold_register_value = 0;
    settings.iholddelay_percent = 0;
    settings.iholddelay_register_value = 0;
    settings.automatic_current_scaling_enabled = false;
    settings.automatic_gradient_adaptation_enabled = false;
    settings.pwm_offset = 0;
    settings.pwm_gradient = 0;
    settings.cool_step_enabled = false;
    settings.analog_current_scaling_enabled = false;
    settings.internal_sense_resistors_enabled = false;
  }

  settings.standstill_mode = static_cast<StandstillMode>(m_pwmConfig.freewheel);
  settings.microsteps_per_step = getMicroSteps();

  return settings;
}

Status TMC2209::getStatus() {
  DriveStatus driveStatus{};
  driveStatus.bytes = 0;
  driveStatus.bytes = read(REGISTER_DRV_STATUS);
  return driveStatus.status;
}

GlobalStatus TMC2209::getGlobalStatus() {
  GlobalStatusUnion globalStatusUnion{};
  globalStatusUnion.bytes = 0;
  globalStatusUnion.bytes = read(REGISTER_GSTAT);
  return globalStatusUnion.global_status;
}

void TMC2209::clearReset() {
  GlobalStatusUnion globalStatusUnion{};
  globalStatusUnion.bytes = 0;
  globalStatusUnion.global_status.reset = 1;
  write(REGISTER_GSTAT, globalStatusUnion.bytes);
}

void TMC2209::clearDriveError() {
  GlobalStatusUnion globalStatusUnion{};
  globalStatusUnion.bytes = 0;
  globalStatusUnion.global_status.drv_err = 1;
  write(REGISTER_GSTAT, globalStatusUnion.bytes);
}

std::uint8_t TMC2209::getInterfaceTransmissionCounter() {
  return read(REGISTER_IFCNT);
}

std::uint32_t TMC2209::getInterstepDuration() {
  return read(REGISTER_TSTEP);
}

std::uint16_t TMC2209::getStallGuardResult() {
  return read(REGISTER_SG_RESULT);
}

std::uint8_t TMC2209::getPwmScaleSum() {
  PwmScale pwmScale{};
  pwmScale.bytes = read(REGISTER_PWM_SCALE);

  return pwmScale.pwm_scale_sum;
}

std::int16_t TMC2209::getPwmScaleAuto() {
  PwmScale pwmScale{};
  pwmScale.bytes = read(REGISTER_PWM_SCALE);

  return static_cast<std::int16_t>(pwmScale.pwm_scale_auto);
}

std::uint8_t TMC2209::getPwmOffsetAuto() {
  PwmAuto pwmAuto{};
  pwmAuto.bytes = read(REGISTER_PWM_AUTO);

  return pwmAuto.pwm_offset_auto;
}

std::uint8_t TMC2209::getPwmGradientAuto() {
  PwmAuto pwmAuto{};
  pwmAuto.bytes = read(REGISTER_PWM_AUTO);

  return pwmAuto.pwm_gradient_auto;
}

std::uint16_t TMC2209::getMicrostepCounter() {
  return read(REGISTER_MSCNT);
}

// private
void TMC2209::initialize() {
  setOperationModeToSerial();
  setRegistersToDefaults();
  clearDriveError();

  minimizeMotorCurrent();
  disable();
  disableAutomaticCurrentScaling();
  disableAutomaticGradientAdaptation();
}

void TMC2209::setOperationModeToSerial() {
  m_globalConfig.bytes = 0;
  m_globalConfig.i_scale_analog = 0;
  m_globalConfig.pdn_disable = 1;
  m_globalConfig.mstep_reg_select = 1;
  m_globalConfig.multistep_filt = 1;

  writeStoredGlobalConfig();
}

void TMC2209::setRegistersToDefaults() {
  m_driverCurrent.bytes = 0;
  m_driverCurrent.ihold = IHOLD_DEFAULT;
  m_driverCurrent.irun = IRUN_DEFAULT;
  m_driverCurrent.iholddelay = IHOLDDELAY_DEFAULT;
  write(REGISTER_IHOLD_IRUN, m_driverCurrent.bytes);

  m_chopperConfig.bytes = CHOPPER_CONFIG_DEFAULT;
  m_chopperConfig.tbl = TBL_DEFAULT;
  m_chopperConfig.hend = HEND_DEFAULT;
  m_chopperConfig.hstart = HSTART_DEFAULT;
  m_chopperConfig.toff = TOFF_DEFAULT;
  write(REGISTER_CHOPCONF, m_chopperConfig.bytes);

  m_pwmConfig.bytes = PWM_CONFIG_DEFAULT;
  write(REGISTER_PWMCONF, m_pwmConfig.bytes);

  m_coolConfig.bytes = COOLCONF_DEFAULT;
  write(REGISTER_COOLCONF, m_coolConfig.bytes);

  write(REGISTER_TPOWERDOWN, TPOWERDOWN_DEFAULT);
  write(REGISTER_TPWMTHRS, TPWMTHRS_DEFAULT);
  write(REGISTER_VACTUAL, VACTUAL_DEFAULT);
  write(REGISTER_TCOOLTHRS, TCOOLTHRS_DEFAULT);
  write(REGISTER_SGTHRS, SGTHRS_DEFAULT);
}

void TMC2209::readAndStoreRegisters() {
  m_globalConfig.bytes = readGlobalConfigBytes();
  m_chopperConfig.bytes = readChopperConfigBytes();
  m_pwmConfig.bytes = readPwmConfigBytes();
}

bool TMC2209::serialOperationMode() {
  GlobalConfig globalConfig{};
  globalConfig.bytes = readGlobalConfigBytes();

  return globalConfig.pdn_disable;
}

void TMC2209::minimizeMotorCurrent() {
  m_driverCurrent.irun = CURRENT_SETTING_MIN;
  m_driverCurrent.ihold = CURRENT_SETTING_MIN;
  writeStoredDriverCurrent();
}

std::uint8_t TMC2209::percentToCurrentSetting(std::uint8_t percent) {
  std::uint8_t constrained_percent = getConstrainedValue(percent, PERCENT_MIN, PERCENT_MAX);
  std::uint8_t current_setting = map(constrained_percent, PERCENT_MIN, PERCENT_MAX, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX);
  return current_setting;
}

std::uint8_t TMC2209::currentSettingToPercent(std::uint8_t current_setting) {
  std::uint8_t percent = map(current_setting, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX, PERCENT_MIN, PERCENT_MAX);
  return percent;
}

std::uint8_t TMC2209::percentToHoldDelaySetting(std::uint8_t percent) {
  std::uint8_t constrained_percent = getConstrainedValue(percent, PERCENT_MIN, PERCENT_MAX);
  std::uint8_t hold_delay_setting = map(constrained_percent, PERCENT_MIN, PERCENT_MAX, HOLD_DELAY_MIN, HOLD_DELAY_MAX);
  return hold_delay_setting;
}

std::uint8_t TMC2209::holdDelaySettingToPercent(std::uint8_t hold_delay_setting) {
  std::uint8_t percent = map(hold_delay_setting, HOLD_DELAY_MIN, HOLD_DELAY_MAX, PERCENT_MIN, PERCENT_MAX);
  return percent;
}

void TMC2209::writeStoredGlobalConfig() {
  write(REGISTER_GCONF, m_globalConfig.bytes);
}

std::uint32_t TMC2209::readGlobalConfigBytes() {
  return read(REGISTER_GCONF);
}

void TMC2209::writeStoredDriverCurrent() {
  write(REGISTER_IHOLD_IRUN, m_driverCurrent.bytes);

  if (m_driverCurrent.irun >= SEIMIN_UPPER_CURRENT_LIMIT) {
    m_coolConfig.seimin = SEIMIN_UPPER_SETTING;
  } else {
    m_coolConfig.seimin = SEIMIN_LOWER_SETTING;
  }
  if (m_coolStepEnabled) {
    write(REGISTER_COOLCONF, m_coolConfig.bytes);
  }
}

void TMC2209::writeStoredChopperConfig() {
  write(REGISTER_CHOPCONF, m_chopperConfig.bytes);
}

std::uint32_t TMC2209::readChopperConfigBytes() {
  return read(REGISTER_CHOPCONF);
}

void TMC2209::writeStoredPwmConfig() {
  write(REGISTER_PWMCONF, m_pwmConfig.bytes);
}

std::uint32_t TMC2209::readPwmConfigBytes() {
  return read(REGISTER_PWMCONF);
}

template<typename Datagram>
void TMC2209::sendDatagramBidirectional(Datagram const &datagram, std::size_t const datagramSize) {
  std::uint8_t byte = 0;

  // Wait for the transmission of outgoing serial data to complete
  m_serial.flush();

  // clear the serial receive buffer if necessary
  while (m_serial.available() > 0) {
    byte = m_serial.read();
  }

  // write datagram
  for (std::size_t byteIndex = 0; byteIndex < datagramSize; ++byteIndex) {
    byte = (datagram.bytes >> (byteIndex * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    m_serial.write(byte);
  }

  // Wait for the transmission of outgoing serial data to complete
  m_serial.flush();

  // wait for bytes sent out on TX line to be echoed on RX line
  std::uint32_t echoDelay = 0;
  while ((m_serial.available() < datagramSize) and (echoDelay < ECHO_DELAY_MAX_MICROSECONDS)) {
    delayMicroseconds(ECHO_DELAY_INC_MICROSECONDS);
    echoDelay += ECHO_DELAY_INC_MICROSECONDS;
  }

  if (echoDelay >= ECHO_DELAY_MAX_MICROSECONDS) {
    return;
  }

  // clear RX buffer of echo bytes
  for (std::size_t byteIndex = 0; byteIndex < datagramSize; ++byteIndex) {
    byte = m_serial.read();
  }
}

template<typename Datagram>
void TMC2209::sendDatagramUnidirectional(Datagram const &datagram, std::size_t const datagramSize) {
  std::uint8_t byte = 0;

  for (std::size_t byteIndex = 0; byteIndex < datagramSize; ++byteIndex) {
    byte = (datagram.bytes >> (byteIndex * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    m_serial.write(byte);
  }
}

void TMC2209::write(RegisterAddress const registerAddress, std::uint32_t const data) {
  WriteReadReplyDatagram writeDatagram{};
  writeDatagram.bytes = 0;
  writeDatagram.sync = SYNC;
  writeDatagram.slave_address = m_slaveAddress;
  writeDatagram.register_address = registerAddress;
  writeDatagram.rw = ACCESS_WRITE;
  writeDatagram.data = reverseBits(data);
  writeDatagram.crc = calculateCrc(writeDatagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

  sendDatagramUnidirectional(writeDatagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
}

std::uint32_t TMC2209::read(RegisterAddress registerAddress) {
  ReadRequestDatagram readRequestDatagram{};
  readRequestDatagram.bytes = 0;
  readRequestDatagram.sync = SYNC;
  readRequestDatagram.slave_address = m_slaveAddress;
  readRequestDatagram.register_address = registerAddress;
  readRequestDatagram.rw = ACCESS_READ;
  readRequestDatagram.crc = calculateCrc(readRequestDatagram, READ_REQUEST_DATAGRAM_SIZE);

  for (std::size_t retry = 0; retry < MAX_READ_RETRIES; retry++) {
    sendDatagramBidirectional(readRequestDatagram, READ_REQUEST_DATAGRAM_SIZE);

    std::uint32_t replyDelay = 0;
    while ((m_serial.available() < WRITE_READ_REPLY_DATAGRAM_SIZE) and (replyDelay < REPLY_DELAY_MAX_MICROSECONDS)) {
      delayMicroseconds(REPLY_DELAY_INC_MICROSECONDS);
      replyDelay += REPLY_DELAY_INC_MICROSECONDS;
    }

    if (replyDelay >= REPLY_DELAY_MAX_MICROSECONDS) {
      return 0;
    }

    std::uint64_t byte;
    std::uint8_t byteCount = 0;

    WriteReadReplyDatagram readReplyDatagram{};
    readReplyDatagram.bytes = 0;

    for (std::uint8_t i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; ++i) {
      byte = m_serial.read();
      readReplyDatagram.bytes |= (byte << (byteCount++ * BITS_PER_BYTE));
    }

    auto crc = calculateCrc(readReplyDatagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
    if (crc == readReplyDatagram.crc) {
      return reverseBits(readReplyDatagram.data);
    }

    delay(READ_RETRY_DELAY_MS);
  }

  return 0;
}

std::uint32_t TMC2209::reverseBits(std::uint32_t const data) {
  std::uint32_t reversedData = 0;

  std::uint8_t rightShift;
  std::uint8_t leftShift;

  for (std::uint8_t i = 0; i < DATA_SIZE; ++i) {
    rightShift = (DATA_SIZE - i - 1) * BITS_PER_BYTE;
    leftShift = i * BITS_PER_BYTE;
    reversedData |= ((data >> rightShift) & BYTE_MAX_VALUE) << leftShift;
  }

  return reversedData;
}

template<typename Datagram>
std::uint8_t TMC2209::calculateCrc(Datagram &datagram, std::size_t const datagramSize) {
  std::uint8_t crc = 0;
  std::uint8_t byte = 0;

  for (std::size_t byteIndex = 0; byteIndex < (datagramSize - 1); ++byteIndex) {
    byte = (datagram.bytes >> (byteIndex * BITS_PER_BYTE)) & BYTE_MAX_VALUE;

    for (std::size_t j = 0; j < BITS_PER_BYTE; ++j) {
      if ((crc >> 7) ^ (byte & 0x01)) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc = crc << 1;
      }
      byte = byte >> 1;
    }
  }

  return crc;
}

std::uint32_t TMC2209::getConstrainedValue(std::uint32_t const value, std::uint32_t const min, std::uint32_t const max) {
  return (value < min ? min : (value > max ? max : value));
}

std::uint32_t TMC2209::map(std::uint32_t value, std::uint32_t start1, std::uint32_t stop1, std::uint32_t start2, std::uint32_t stop2) {
  return start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
}
void TMC2209::stepUp() {
}
void TMC2209::stepDown() {
}
motor::Direction TMC2209::getDirection() const {
  return motor::MOTOR_ROTATE_CCW;
}

bool TMC2209::isFault() const {
  return false;
}
bool TMC2209::inHomed() const {
  return false;
}
bool TMC2209::isEnabled() const {
  return false;
}

}// namespace motor::driver
