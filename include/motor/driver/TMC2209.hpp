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

#include <Arduino.h>
#include <cstdint>

enum SerialAddress {
  SERIAL_ADDRESS_0 = 0,
  SERIAL_ADDRESS_1 = 1,
  SERIAL_ADDRESS_2 = 2,
  SERIAL_ADDRESS_3 = 3,
};

enum Register {
  ADDRESS_GCONF = 0x00,
  ADDRESS_GSTAT = 0x01,
  ADDRESS_IFCNT = 0x02,
  ADDRESS_REPLYDELAY = 0x03,
  ADDRESS_IOIN = 0x06,
  ADDRESS_IHOLD_IRUN = 0x10,
  ADDRESS_TPOWERDOWN = 0x11,
  ADDRESS_TSTEP = 0x12,
  ADDRESS_TPWMTHRS = 0x13,
  ADDRESS_TCOOLTHRS = 0x14,
  ADDRESS_VACTUAL = 0x22,
  ADDRESS_SGTHRS = 0x40,
  ADDRESS_SG_RESULT = 0x41,
  ADDRESS_COOLCONF = 0x42,
  ADDRESS_MSCNT = 0x6A,
  ADDRESS_MSCURACT = 0x6B,
  ADDRESS_CHOPCONF = 0x6C,
  ADDRESS_DRV_STATUS = 0x6F,
  ADDRESS_PWMCONF = 0x70,
  ADDRESS_PWM_SCALE = 0x71,
  ADDRESS_PWM_AUTO = 0x72
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

struct Settings {
  bool is_communicating;
  bool is_setup;
  bool software_enabled;
  uint16_t microsteps_per_step;
  bool inverse_motor_direction_enabled;
  bool stealth_chop_enabled;
  uint8_t standstill_mode;
  uint8_t irun_percent;
  uint8_t irun_register_value;
  uint8_t ihold_percent;
  uint8_t ihold_register_value;
  uint8_t iholddelay_percent;
  uint8_t iholddelay_register_value;
  bool automatic_current_scaling_enabled;
  bool automatic_gradient_adaptation_enabled;
  uint8_t pwm_offset;
  uint8_t pwm_gradient;
  bool cool_step_enabled;
  bool analog_current_scaling_enabled;
  bool internal_sense_resistors_enabled;
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

struct GlobalStatus {
  std::uint32_t reset : 1;
  std::uint32_t drv_err : 1;
  std::uint32_t uv_cp : 1;
  std::uint32_t reserved : 29;
};

union WriteReadReplyDatagram {
  struct
  {
    std::uint64_t sync : 4;
    std::uint64_t reserved : 4;
    std::uint64_t serial_address : 8;
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
    std::uint32_t serial_address : 8;
    std::uint32_t register_address : 7;
    std::uint32_t rw : 1;
    std::uint32_t crc : 8;
  };
  std::uint32_t bytes;
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
const static std::uint8_t RW_READ = 0;
const static std::uint8_t RW_WRITE = 1;
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

const static std::uint8_t MRES_256 = 0b0000;
const static std::uint8_t MRES_128 = 0b0001;
const static std::uint8_t MRES_064 = 0b0010;
const static std::uint8_t MRES_032 = 0b0011;
const static std::uint8_t MRES_016 = 0b0100;
const static std::uint8_t MRES_008 = 0b0101;
const static std::uint8_t MRES_004 = 0b0110;
const static std::uint8_t MRES_002 = 0b0111;
const static std::uint8_t MRES_001 = 0b1000;
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

class TMC2209 {
public:
  explicit TMC2209(std::uint8_t uartPort, SerialAddress serialAddress, std::uint8_t rxPin = 0, std::uint8_t txPin = 0);

  // driver must be enabled before use it is disabled by default
  void setHardwareEnablePin(uint8_t hardware_enable_pin);
  void enable();
  void disable();

  // valid values = 1,2,4,8,...128,256, other values get rounded down
  void setMicrostepsPerStep(uint16_t microsteps_per_step);

  // valid values = 0-8, microsteps = 2^exponent, 0=1,1=2,2=4,...8=256
  // https://en.wikipedia.org/wiki/Power_of_two
  void setMicrostepsPerStepPowerOfTwo(uint8_t exponent);

  // range 0-100
  void setRunCurrent(uint8_t percent);
  // range 0-100
  void setHoldCurrent(uint8_t percent);
  // range 0-100
  void setHoldDelay(uint8_t percent);
  // range 0-100
  void setAllCurrentValues(uint8_t run_current_percent, uint8_t hold_current_percent, uint8_t hold_delay_percent);
  void setRMSCurrent(uint16_t mA,
                     float rSense,
                     float holdMultiplier = 0.5f);

  void enableDoubleEdge();
  void disableDoubleEdge();

  void enableVSense();
  void disableVSense();

  void enableInverseMotorDirection();
  void disableInverseMotorDirection();

  void setStandstillMode(StandstillMode mode);

  void enableAutomaticCurrentScaling();
  void disableAutomaticCurrentScaling();
  void enableAutomaticGradientAdaptation();
  void disableAutomaticGradientAdaptation();
  // range 0-255
  void setPwmOffset(uint8_t pwm_amplitude);
  // range 0-255
  void setPwmGradient(uint8_t pwm_amplitude);

  // default = 20
  // mimimum of 2 for StealthChop auto tuning
  void setPowerDownDelay(uint8_t power_down_delay);

  // mimimum of 2 when using multiple serial addresses
  // in bidirectional communication
  const static uint8_t REPLY_DELAY_MAX = 15;
  void setReplyDelay(uint8_t delay);

  void moveAtVelocity(int32_t microsteps_per_period);
  void moveUsingStepDirInterface();

  void enableStealthChop();
  void disableStealthChop();

  void setStealthChopDurationThreshold(uint32_t duration_threshold);

  void setStallGuardThreshold(uint8_t stall_guard_threshold);

  // lower_threshold: min = 1, max = 15
  // upper_threshold: min = 0, max = 15, 0-2 recommended
  void enableCoolStep(uint8_t lower_threshold = 1, uint8_t upper_threshold = 0);
  void disableCoolStep();

  void setCoolStepCurrentIncrement(CurrentIncrement current_increment);

  void setCoolStepMeasurementCount(MeasurementCount measurement_count);
  void setCoolStepDurationThreshold(uint32_t duration_threshold);

  void enableAnalogCurrentScaling();
  void disableAnalogCurrentScaling();

  void useExternalSenseResistors();
  void useInternalSenseResistors();

  // bidirectional methods
  uint8_t getVersion();

  // if driver is not communicating, check power and communication connections
  bool isCommunicating();

  // check to make sure TMC2209 is properly setup and communicating
  bool isSetupAndCommunicating();

  // driver may be communicating but not setup if driver power is lost then
  // restored after setup so that defaults are loaded instead of setup options
  bool isCommunicatingButNotSetup();

  // driver may also be disabled by the hardware enable input pin
  // this pin must be grounded or disconnected before driver may be enabled
  bool hardwareDisabled();

  uint16_t getMicrostepsPerStep();

  Settings getSettings();

  const static uint8_t CURRENT_SCALING_MAX = 31;
  Status getStatus();

  GlobalStatus getGlobalStatus();
  void clearReset();
  void clearDriveError();

  uint8_t getInterfaceTransmissionCounter();

  uint32_t getInterstepDuration();

  uint16_t getStallGuardResult();

  uint8_t getPwmScaleSum();
  int16_t getPwmScaleAuto();
  uint8_t getPwmOffsetAuto();
  uint8_t getPwmGradientAuto();

  uint16_t getMicrostepCounter();

private:
  void initialize(SerialAddress serial_address = SERIAL_ADDRESS_0);
  int serialAvailable();
  size_t serialWrite(uint8_t c);
  int serialRead();
  void serialFlush();

  void setOperationModeToSerial(SerialAddress serial_address);

  void setRegistersToDefaults();
  void readAndStoreRegisters();

  bool serialOperationMode();

  void minimizeMotorCurrent();

  uint32_t reverseData(uint32_t data);
  template<typename Datagram>
  uint8_t calculateCrc(Datagram &datagram, uint8_t datagram_size);
  template<typename Datagram>
  void sendDatagramUnidirectional(Datagram &datagram, uint8_t datagram_size);
  template<typename Datagram>
  void sendDatagramBidirectional(Datagram &datagram, uint8_t datagram_size);

  void write(uint8_t register_address, uint32_t data);
  uint32_t read(uint8_t register_address);

  uint8_t percentToCurrentSetting(uint8_t percent);
  uint8_t currentSettingToPercent(uint8_t current_setting);
  uint8_t percentToHoldDelaySetting(uint8_t percent);
  uint8_t holdDelaySettingToPercent(uint8_t hold_delay_setting);

  void writeStoredGlobalConfig();
  uint32_t readGlobalConfigBytes();
  void writeStoredDriverCurrent();
  void writeStoredChopperConfig();
  uint32_t readChopperConfigBytes();
  void writeStoredPwmConfig();
  uint32_t readPwmConfigBytes();

  uint32_t constrain_(uint32_t value, uint32_t low, uint32_t high);

private:
  HardwareSerial *hardware_serial_ptr_;
  uint8_t serial_address_;
  int16_t hardware_enable_pin_;
  GlobalConfig global_config_;
  DriverCurrent driver_current_;
  CoolConfig cool_config_;
  bool cool_step_enabled_;
  ChopperConfig chopper_config_;
  uint8_t toff_ = TOFF_DEFAULT;
  PwmConfig pwm_config_;
};
