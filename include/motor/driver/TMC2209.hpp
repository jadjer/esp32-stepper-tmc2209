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
// TMC2209.h
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#pragma once

#include <HardwareSerial.h>
#include <cstdint>
#include <gpio/OutputPin.hpp>
#include <motor/driver/interface/Driver.hpp>

namespace motor::driver {

enum SlaveAddress {
  SLAVE_ADDRESS_0 = 0,
  SLAVE_ADDRESS_1 = 1,
  SLAVE_ADDRESS_2 = 2,
  SLAVE_ADDRESS_3 = 3,
};

enum RegisterAddress {
  REGISTER_GCONF = 0x00,
  REGISTER_GSTAT = 0x01,
  REGISTER_IFCNT = 0x02,
  REGISTER_REPLYDELAY = 0x03,
  REGISTER_IOIN = 0x06,
  REGISTER_IHOLD_IRUN = 0x10,
  REGISTER_TPOWERDOWN = 0x11,
  REGISTER_TSTEP = 0x12,
  REGISTER_TPWMTHRS = 0x13,
  REGISTER_TCOOLTHRS = 0x14,
  REGISTER_VACTUAL = 0x22,
  REGISTER_SGTHRS = 0x40,
  REGISTER_SG_RESULT = 0x41,
  REGISTER_COOLCONF = 0x42,
  REGISTER_MSCNT = 0x6A,
  REGISTER_MSCURACT = 0x6B,
  REGISTER_CHOPCONF = 0x6C,
  REGISTER_DRV_STATUS = 0x6F,
  REGISTER_PWMCONF = 0x70,
  REGISTER_PWM_SCALE = 0x71,
  REGISTER_PWM_AUTO = 0x72
};

enum AccessType {
  ACCESS_READ = 0,
  ACCESS_WRITE = 1,
};

enum StandstillMode {
  NORMAL = 0,
  FREEWHEELING = 1,
  STRONG_BRAKING = 2,
  BRAKING = 3,
};

enum CurrentIncrement {
  CURRENT_INCREMENT_1 = 0,
  CURRENT_INCREMENT_2 = 1,
  CURRENT_INCREMENT_4 = 2,
  CURRENT_INCREMENT_8 = 3,
};

enum MeasurementCount {
  MEASUREMENT_COUNT_32 = 0,
  MEASUREMENT_COUNT_8 = 1,
  MEASUREMENT_COUNT_2 = 2,
  MEASUREMENT_COUNT_1 = 3,
};

enum MicroStepResolution {
  MRES_256 = 0b0000,
  MRES_128 = 0b0001,
  MRES_064 = 0b0010,
  MRES_032 = 0b0011,
  MRES_016 = 0b0100,
  MRES_008 = 0b0101,
  MRES_004 = 0b0110,
  MRES_002 = 0b0111,
  MRES_001 = 0b1000,
};

union WriteReadReplyDatagram {
  struct
  {
    std::uint64_t sync : 4;
    std::uint64_t reserved : 4;
    std::uint64_t slave_address : 8;
    std::uint64_t register_address : 7;
    std::uint64_t rw : 1;
    std::uint64_t data : 32;
    std::uint64_t crc : 8;
  };
  std::uint64_t bytes;
};

union ReadRequestDatagram {
  struct
  {
    std::uint32_t sync : 4;
    std::uint32_t reserved : 4;
    std::uint32_t slave_address : 8;
    std::uint32_t register_address : 7;
    std::uint32_t rw : 1;
    std::uint32_t crc : 8;
  };
  std::uint32_t bytes;
};

struct Settings {
  bool is_communicating;
  bool is_setup;
  bool software_enabled;
  motor::MotorSteps microsteps_per_step;
  bool inverse_motor_direction_enabled;
  bool stealth_chop_enabled;
  StandstillMode standstill_mode;
  std::uint8_t irun_percent;
  std::uint8_t irun_register_value;
  std::uint8_t ihold_percent;
  std::uint8_t ihold_register_value;
  std::uint8_t iholddelay_percent;
  std::uint8_t iholddelay_register_value;
  bool automatic_current_scaling_enabled;
  bool automatic_gradient_adaptation_enabled;
  std::uint8_t pwm_offset;
  std::uint8_t pwm_gradient;
  bool cool_step_enabled;
  bool analog_current_scaling_enabled;
  bool internal_sense_resistors_enabled;
};

union GlobalConfig {
  struct
  {
    std::uint32_t i_scale_analog : 1;
    std::uint32_t internal_rsense : 1;
    std::uint32_t enable_spread_cycle : 1;
    std::uint32_t shaft : 1;
    std::uint32_t index_otpw : 1;
    std::uint32_t index_step : 1;
    std::uint32_t pdn_disable : 1;
    std::uint32_t mstep_reg_select : 1;
    std::uint32_t multistep_filt : 1;
    std::uint32_t test_mode : 1;
    std::uint32_t reserved : 22;
  };
  std::uint32_t bytes;
};

struct GlobalStatus {
  std::uint32_t reset : 1;
  std::uint32_t drv_err : 1;
  std::uint32_t uv_cp : 1;
  std::uint32_t reserved : 29;
};

union GlobalStatusUnion {
  struct
  {
    GlobalStatus global_status;
  };
  std::uint32_t bytes;
};

union ReplyDelay {
  struct
  {
    std::uint32_t reserved_0 : 8;
    std::uint32_t replydelay : 4;
    std::uint32_t reserved_1 : 20;
  };
  std::uint32_t bytes;
};

union Input {
  struct
  {
    std::uint32_t enn : 1;
    std::uint32_t reserved_0 : 1;
    std::uint32_t ms1 : 1;
    std::uint32_t ms2 : 1;
    std::uint32_t diag : 1;
    std::uint32_t reserved_1 : 1;
    std::uint32_t pdn_serial : 1;
    std::uint32_t step : 1;
    std::uint32_t spread_en : 1;
    std::uint32_t dir : 1;
    std::uint32_t reserved_2 : 14;
    std::uint32_t version : 8;
  };
  std::uint32_t bytes;
};

union DriverCurrent {
  struct
  {
    std::uint32_t ihold : 5;
    std::uint32_t reserved_0 : 3;
    std::uint32_t irun : 5;
    std::uint32_t reserved_1 : 3;
    std::uint32_t iholddelay : 4;
    std::uint32_t reserved_2 : 12;
  };
  std::uint32_t bytes;
};

union CoolConfig {
  struct
  {
    std::uint32_t semin : 4;
    std::uint32_t reserved_0 : 1;
    std::uint32_t seup : 2;
    std::uint32_t reserved_1 : 1;
    std::uint32_t semax : 4;
    std::uint32_t reserved_2 : 1;
    std::uint32_t sedn : 2;
    std::uint32_t seimin : 1;
    std::uint32_t reserved_3 : 16;
  };
  std::uint32_t bytes;
};

union ChopperConfig {
  struct
  {
    std::uint32_t toff : 4;
    std::uint32_t hstart : 3;
    std::uint32_t hend : 4;
    std::uint32_t reserved_0 : 4;
    std::uint32_t tbl : 2;
    std::uint32_t vsense : 1;
    std::uint32_t reserved_1 : 6;
    std::uint32_t mres : 4;
    std::uint32_t interpolation : 1;
    std::uint32_t double_edge : 1;
    std::uint32_t diss2g : 1;
    std::uint32_t diss2vs : 1;
  };
  std::uint32_t bytes;
};

struct Status {
  std::uint32_t over_temperature_warning : 1;
  std::uint32_t over_temperature_shutdown : 1;
  std::uint32_t short_to_ground_a : 1;
  std::uint32_t short_to_ground_b : 1;
  std::uint32_t low_side_short_a : 1;
  std::uint32_t low_side_short_b : 1;
  std::uint32_t open_load_a : 1;
  std::uint32_t open_load_b : 1;
  std::uint32_t over_temperature_120c : 1;
  std::uint32_t over_temperature_143c : 1;
  std::uint32_t over_temperature_150c : 1;
  std::uint32_t over_temperature_157c : 1;
  std::uint32_t reserved0 : 4;
  std::uint32_t current_scaling : 5;
  std::uint32_t reserved1 : 9;
  std::uint32_t stealth_chop_mode : 1;
  std::uint32_t standstill : 1;
};

union DriveStatus {
  struct
  {
    Status status;
  };
  std::uint32_t bytes;
};

union PwmConfig {
  struct
  {
    std::uint32_t pwm_offset : 8;
    std::uint32_t pwm_grad : 8;
    std::uint32_t pwm_freq : 2;
    std::uint32_t pwm_autoscale : 1;
    std::uint32_t pwm_autograd : 1;
    std::uint32_t freewheel : 2;
    std::uint32_t reserved : 2;
    std::uint32_t pwm_reg : 4;
    std::uint32_t pwm_lim : 4;
  };
  std::uint32_t bytes;
};

union PwmScale {
  struct
  {
    std::uint32_t pwm_scale_sum : 8;
    std::uint32_t reserved_0 : 8;
    std::uint32_t pwm_scale_auto : 9;
    std::uint32_t reserved_1 : 7;
  };
  std::uint32_t bytes;
};

union PwmAuto {
  struct
  {
    std::uint32_t pwm_offset_auto : 8;
    std::uint32_t reserved_0 : 8;
    std::uint32_t pwm_gradient_auto : 8;
    std::uint32_t reserved_1 : 8;
  };
  std::uint32_t bytes;
};

// Serial Settings
const static std::uint8_t BYTE_MAX_VALUE = 0xFF;
const static std::uint8_t BITS_PER_BYTE = 8;

const static std::uint32_t ECHO_DELAY_INC_MICROSECONDS = 1;
const static std::uint32_t ECHO_DELAY_MAX_MICROSECONDS = 4000;

const static std::uint32_t REPLY_DELAY_INC_MICROSECONDS = 1;
const static std::uint32_t REPLY_DELAY_MAX_MICROSECONDS = 10000;

const static std::uint8_t STEPPER_DRIVER_FEATURE_OFF = 0;
const static std::uint8_t STEPPER_DRIVER_FEATURE_ON = 1;

const static std::uint8_t MAX_READ_RETRIES = 5;
const static std::uint32_t READ_RETRY_DELAY_MS = 20;

// Datagrams
const static std::uint8_t WRITE_READ_REPLY_DATAGRAM_SIZE = 8;
const static std::uint8_t DATA_SIZE = 4;

const static std::uint8_t SYNC = 0b101;
const static std::uint8_t READ_REPLY_SERIAL_ADDRESS = 0b11111111;

const static std::uint8_t READ_REQUEST_DATAGRAM_SIZE = 4;

const static std::uint8_t VERSION = 0x21;

const static std::uint8_t PERCENT_MIN = 0;
const static std::uint8_t PERCENT_MAX = 100;
const static std::uint8_t CURRENT_SETTING_MIN = 0;
const static std::uint8_t CURRENT_SETTING_MAX = 31;
const static std::uint8_t HOLD_DELAY_MIN = 0;
const static std::uint8_t HOLD_DELAY_MAX = 15;
const static std::uint8_t IHOLD_DEFAULT = 16;
const static std::uint8_t IRUN_DEFAULT = 31;
const static std::uint8_t IHOLDDELAY_DEFAULT = 1;
const static std::uint8_t TPOWERDOWN_DEFAULT = 20;
const static std::uint32_t TPWMTHRS_DEFAULT = 0;

const static std::int32_t VACTUAL_DEFAULT = 0;
const static std::int32_t VACTUAL_STEP_DIR_INTERFACE = 0;

const static std::uint8_t TCOOLTHRS_DEFAULT = 0;
const static std::uint8_t SGTHRS_DEFAULT = 0;

const static std::uint8_t COOLCONF_DEFAULT = 0;

const static std::uint8_t SEIMIN_UPPER_CURRENT_LIMIT = 20;
const static std::uint8_t SEIMIN_LOWER_SETTING = 0;
const static std::uint8_t SEIMIN_UPPER_SETTING = 1;
const static std::uint8_t SEMIN_OFF = 0;
const static std::uint8_t SEMIN_MIN = 1;
const static std::uint8_t SEMIN_MAX = 15;
const static std::uint8_t SEMAX_MIN = 0;
const static std::uint8_t SEMAX_MAX = 15;

const static std::uint32_t CHOPPER_CONFIG_DEFAULT = 0x10000053;
const static std::uint8_t TBL_DEFAULT = 0b10;
const static std::uint8_t HEND_DEFAULT = 0;
const static std::uint8_t HSTART_DEFAULT = 5;
const static std::uint8_t TOFF_DEFAULT = 3;
const static std::uint8_t TOFF_DISABLE = 0;

const static std::uint8_t DOUBLE_EDGE_DISABLE = 0;
const static std::uint8_t DOUBLE_EDGE_ENABLE = 1;
const static std::uint8_t VSENSE_DISABLE = 0;
const static std::uint8_t VSENSE_ENABLE = 1;

const static std::size_t MICROSTEPS_PER_STEP_MIN = 1;
const static std::size_t MICROSTEPS_PER_STEP_MAX = 256;

const static std::uint32_t PWM_CONFIG_DEFAULT = 0xC10D0024;
const static std::uint8_t PWM_OFFSET_MIN = 0;
const static std::uint8_t PWM_OFFSET_MAX = 255;
const static std::uint8_t PWM_OFFSET_DEFAULT = 0x24;
const static std::uint8_t PWM_GRAD_MIN = 0;
const static std::uint8_t PWM_GRAD_MAX = 255;
const static std::uint8_t PWM_GRAD_DEFAULT = 0x14;

const static std::uint8_t CURRENT_SCALING_MAX = 31;
const static std::uint8_t REPLY_DELAY_MAX = 15;

class TMC2209 : public motor::driver::interface::Driver {
public:
  explicit TMC2209(std::uint8_t uartPort, SlaveAddress slaveAddress, std::int8_t rxPin = -1, std::int8_t txPin = -1);
  ~TMC2209() override = default;

public:
  void setDirection(motor::Direction direction) override;
  void setMicroSteps(motor::MotorSteps microSteps) override;
  void setHardwareEnablePin(std::uint8_t hardwareEnablePin);
  void setRunCurrent(std::uint8_t percent);
  void setHoldCurrent(std::uint8_t percent);
  void setHoldDelay(std::uint8_t percent);
  void setAllCurrentValues(std::uint8_t run_current_percent, std::uint8_t hold_current_percent, std::uint8_t hold_delay_percent);
  void setRMSCurrent(std::uint16_t mA, float rSense, float holdMultiplier = 0.5f);
  void setStandstillMode(StandstillMode mode);
  void setPwmOffset(std::uint8_t pwm_amplitude);
  void setPwmGradient(std::uint8_t pwm_amplitude);
  void setPowerDownDelay(std::uint8_t power_down_delay);
  void setReplyDelay(std::uint8_t delay);

public:
  void enable() override;
  void disable() override;

public:
  void enableDoubleEdge();
  void disableDoubleEdge();

public:
  void enableVSense();
  void disableVSense();

public:
  void enableInverseMotorDirection();
  void disableInverseMotorDirection();

public:
  void enableAutomaticCurrentScaling();
  void disableAutomaticCurrentScaling();

public:
  void enableAutomaticGradientAdaptation();
  void disableAutomaticGradientAdaptation();

public:
  void enableStealthChop();
  void disableStealthChop();

public:
  void enableCoolStep(std::uint8_t lower_threshold = 1, std::uint8_t upper_threshold = 0);
  void disableCoolStep();

public:
  void moveAtVelocity(std::int32_t microsteps_per_period);
  void moveUsingStepDirInterface();

  void setStealthChopDurationThreshold(std::uint32_t duration_threshold);
  void setStallGuardThreshold(std::uint8_t stall_guard_threshold);

  void setCoolStepCurrentIncrement(CurrentIncrement current_increment);

  void setCoolStepMeasurementCount(MeasurementCount measurement_count);
  void setCoolStepDurationThreshold(std::uint32_t duration_threshold);

  void enableAnalogCurrentScaling();
  void disableAnalogCurrentScaling();

  void useExternalSenseResistors();
  void useInternalSenseResistors();

public:
  void stepUp() override;
  void stepDown() override;

public:
  [[nodiscard]] std::uint8_t getVersion();
  [[nodiscard]] motor::Direction getDirection() const override;
  [[nodiscard]] motor::MotorSteps getMicroSteps() const override;

  Settings getSettings();

  Status getStatus();

  GlobalStatus getGlobalStatus();
  std::uint8_t getInterfaceTransmissionCounter();

  std::uint32_t getInterstepDuration();

  std::uint16_t getStallGuardResult();

  uint8_t getPwmScaleSum();
  std::int16_t getPwmScaleAuto();
  std::uint8_t getPwmOffsetAuto();
  std::uint8_t getPwmGradientAuto();

  std::uint16_t getMicrostepCounter();

public:
  [[nodiscard]] bool isFault() const override;
  [[nodiscard]] bool inHomed() const override;
  [[nodiscard]] bool isEnabled() const override;
  bool isCommunicating();
  // bidirectional methods

  bool isSetupAndCommunicating();

  bool isCommunicatingButNotSetup();

  bool hardwareDisabled();

  void clearReset();
  void clearDriveError();

private:
  void initialize();

private:
  void setOperationModeToSerial();

  void setRegistersToDefaults();
  void readAndStoreRegisters();

  bool serialOperationMode();

  void minimizeMotorCurrent();

  static std::uint8_t percentToCurrentSetting(std::uint8_t percent);
  static std::uint8_t currentSettingToPercent(std::uint8_t current_setting);
  static std::uint8_t percentToHoldDelaySetting(std::uint8_t percent);
  static std::uint8_t holdDelaySettingToPercent(std::uint8_t hold_delay_setting);

  void writeStoredGlobalConfig();
  std::uint32_t readGlobalConfigBytes();
  void writeStoredDriverCurrent();
  void writeStoredChopperConfig();
  std::uint32_t readChopperConfigBytes();
  void writeStoredPwmConfig();
  std::uint32_t readPwmConfigBytes();

private:
  template<typename Datagram>
  void sendDatagramBidirectional(Datagram const &datagram, std::size_t datagramSize);
  template<typename Datagram>
  void sendDatagramUnidirectional(Datagram const &datagram, std::size_t datagramSize);

private:
  void write(RegisterAddress registerAddress, std::uint32_t data);
  std::uint32_t read(RegisterAddress registerAddress);

private:
  static std::uint32_t reverseBits(std::uint32_t value);
  template<typename Datagram>
  static std::uint8_t calculateCrc(Datagram &datagram, std::size_t datagramSize);
  static std::uint32_t getConstrainedValue(std::uint32_t value, std::uint32_t min, std::uint32_t max);
  static std::uint32_t map(std::uint32_t value, std::uint32_t start1, std::uint32_t stop1, std::uint32_t start2, std::uint32_t stop2);

private:
  SlaveAddress const m_slaveAddress;

private:
  gpio::OutputPin const m_stepPin;
  gpio::OutputPin const m_directionPin;

private:
  OutputPinPtr m_enablePin;

private:
  bool m_coolStepEnabled;
  std::uint8_t m_toff;
  HardwareSerial m_serial;

private:
  PwmConfig m_pwmConfig;
  CoolConfig m_coolConfig;
  GlobalConfig m_globalConfig;
  DriverCurrent m_driverCurrent;
  ChopperConfig m_chopperConfig;
};

}
