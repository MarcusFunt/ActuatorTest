#include <Arduino.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <TMCStepper.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// XIAO ESP32-C6 actuator firmware for the Python bench tool in this repo.
//
// USB Serial speaks the binary actuator protocol at 1,000,000 baud.
// Do not add Serial.print debug logs on Serial; they will corrupt the GUI parser.

// -- Hardware I2C (AS5600 encoder 0 / output encoder) ------------------------
#define PIN_HW_I2C_SDA      22
#define PIN_HW_I2C_SCL      23
#define HW_I2C_FREQ_HZ      400000UL

// -- Software I2C (AS5600 encoder 1 / motor encoder) -------------------------
#define PIN_SW_I2C_SDA      1
#define PIN_SW_I2C_SCL      2
#define SW_I2C_FREQ_HZ      100000UL

// -- AS5600 ------------------------------------------------------------------
#define AS5600_I2C_ADDR     0x36

// -- Stepper / TMC2209 --------------------------------------------------------
#define PIN_STEP            20
#define PIN_DIR             19
#define PIN_ENABLE          18      // active LOW

// TMC2209 UART: GPIO16 TX -> 1 kOhm -> TMC PDN_UART, GPIO17 RX -> TMC PDN_UART
#define PIN_TMC_UART_RX     17
#define PIN_TMC_UART_TX     16
#define TMC_UART_BAUD       115200
#define TMC_R_SENSE         0.11f   // BTT TMC2209 v1.3 sense resistor (ohms)
#define TMC_DRIVER_ADDR     0       // MS1/MS2 address pins both LOW

// -- Motion / electrical tuning ----------------------------------------------
#define USB_SERIAL_BAUD             1000000UL
#define MOTOR_FULL_STEPS_PER_REV    200
#define MOTOR_MICROSTEPS            16
#define MOTOR_RMS_CURRENT_MA        1000
#define STEP_PULSE_US               3
#define MAX_STEP_RATE_SPS           60000.0f
#define DEFAULT_BUS_VOLTAGE         24.0f
#define DEFAULT_TEMPERATURE_C       30.0f

// These match the desktop tool's default safety limits.
#define MAX_MOVE_RAD                60.0f
#define MAX_VELOCITY_RAD_S          40.0f
#define MAX_ACCEL_RAD_S2            1000.0f

#define TELEMETRY_HZ                500UL
#define ENCODER_UPDATE_PERIOD_US    2000UL
#define ENCODER_FAILURE_FAULT_COUNT 3
#define STEP_TIMER_HZ               1000000UL
#define PLANNER_HZ                  1000UL
#define PLANNER_PERIOD_US           (1000000UL / PLANNER_HZ)
#define MAX_PLANNER_CATCHUP_TICKS   5
#define MIN_STEP_LOW_US             2UL
#define MIN_STEP_INTERVAL_US        17UL
#define MAX_SERIAL_BYTES_PER_LOOP   128UL

static const float TWO_PI_F = 6.2831853071795864769f;
static const float ENCODER_RAD_PER_COUNT = TWO_PI_F / 4096.0f;
static const float MOTOR_RAD_PER_MICROSTEP =
    TWO_PI_F / (float)(MOTOR_FULL_STEPS_PER_REV * MOTOR_MICROSTEPS);

// -- Binary protocol constants ------------------------------------------------
static const uint8_t PROTOCOL_VERSION = 1;
static const uint16_t MAX_PAYLOAD_SIZE = 4096;
static const uint16_t FRAME_OVERHEAD_SIZE = 10;
static const uint8_t MAGIC_0 = 0xA5;
static const uint8_t MAGIC_1 = 0x5A;

enum PacketType : uint8_t {
  PACKET_COMMAND = 1,
  PACKET_RESPONSE = 2,
  PACKET_TELEMETRY = 3,
  PACKET_EVENT = 4,
};

enum CommandID : uint8_t {
  CMD_PING = 1,
  CMD_INFO = 2,
  CMD_STREAM_ON = 3,
  CMD_STREAM_OFF = 4,
  CMD_SET_MODE = 5,
  CMD_MOVE_REL = 6,
  CMD_STOP = 7,
  CMD_ESTOP = 8,
  CMD_ZERO_MOTOR_ENCODER = 9,
  CMD_ZERO_OUTPUT_ENCODER = 10,
  CMD_GET_CONFIG = 11,
  CMD_SET_CONFIG = 12,
  CMD_SAVE_CONFIG = 13,
  CMD_FAULTS = 14,
  CMD_CLEAR_FAULTS = 15,
  CMD_SELF_TEST = 16,
  CMD_START_CHIRP = 17,
  CMD_MOVE_OUTPUT_REL = 18,
  CMD_SET_POSITION_TARGET = 19,
  CMD_SET_VELOCITY_TARGET = 20,
  CMD_SET_TORQUE_PROXY_TARGET = 21,
  CMD_AUTOTUNE_CONTROL = 22,
  CMD_GET_CONTROL_STATUS = 23,
};

enum ResponseStatus : uint8_t {
  RESP_OK = 0,
  RESP_ERROR = 1,
  RESP_UNSUPPORTED = 2,
  RESP_FAULT = 3,
  RESP_BAD_MODE = 4,
  RESP_BAD_PARAM = 5,
  RESP_TIMEOUT = 6,
};

enum ActuatorMode : uint8_t {
  MODE_DISABLED = 0,
  MODE_CALIBRATION = 1,
  MODE_OPEN_LOOP = 2,
  MODE_POSITION = 3,
  MODE_VELOCITY = 4,
  MODE_TORQUE_PROXY = 5,
  MODE_FAULT = 255,
};

enum FaultFlags : uint32_t {
  FAULT_NONE = 0,
  FAULT_COMMAND_TIMEOUT = 1UL << 0,
  FAULT_OVERTRAVEL = 1UL << 1,
  FAULT_OVERCURRENT = 1UL << 2,
  FAULT_ENCODER_DISAGREEMENT = 1UL << 3,
  FAULT_OVER_TEMPERATURE = 1UL << 4,
  FAULT_ESTOP_ACTIVE = 1UL << 5,
  FAULT_PROTOCOL_ERROR = 1UL << 6,
  FAULT_MISSED_STEP = 1UL << 7,
  FAULT_CONTROL_ERROR = 1UL << 8,
  FAULT_AUTOTUNE_FAILED = 1UL << 9,
};

enum ControlState : uint8_t {
  CONTROL_IDLE = 0,
  CONTROL_AUTOTUNE_RUNNING = 1,
  CONTROL_AUTOTUNE_SUCCESS = 2,
  CONTROL_AUTOTUNE_FAILED = 3,
};

enum AutotuneLoop : uint8_t {
  AUTOTUNE_VELOCITY = 1,
  AUTOTUNE_POSITION = 2,
  AUTOTUNE_BOTH = 3,
};

// -- Small bit-banged I2C bus for the second AS5600 ---------------------------
class SoftI2CBus {
 public:
  SoftI2CBus(uint8_t sdaPin, uint8_t sclPin, uint32_t frequencyHz)
      : sda_(sdaPin), scl_(sclPin) {
    halfDelayUs_ = 500000UL / frequencyHz;
    if (halfDelayUs_ < 2) {
      halfDelayUs_ = 2;
    }
  }

  void begin() {
    releaseSda();
    releaseScl();
  }

  bool readRegister(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    if (!start()) {
      stop();
      return false;
    }
    if (!writeByte(addr << 1)) {
      stop();
      return false;
    }
    if (!writeByte(reg)) {
      stop();
      return false;
    }
    if (!start()) {
      stop();
      return false;
    }
    if (!writeByte((addr << 1) | 1)) {
      stop();
      return false;
    }
    for (size_t i = 0; i < len; ++i) {
      uint8_t value = 0;
      if (!readByte(value, i + 1 < len)) {
        stop();
        return false;
      }
      data[i] = value;
    }
    stop();
    return true;
  }

 private:
  uint8_t sda_;
  uint8_t scl_;
  uint32_t halfDelayUs_;

  void i2cDelay() const {
    delayMicroseconds(halfDelayUs_);
  }

  static void driveLow(uint8_t pin) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }

  static void release(uint8_t pin) {
    pinMode(pin, INPUT_PULLUP);
  }

  void driveSdaLow() const {
    driveLow(sda_);
  }

  void driveSclLow() const {
    driveLow(scl_);
  }

  void releaseSda() const {
    release(sda_);
  }

  void releaseScl() const {
    release(scl_);
  }

  bool waitSclHigh() const {
    const uint32_t startUs = micros();
    while (digitalRead(scl_) == LOW) {
      if ((uint32_t)(micros() - startUs) > 1000UL) {
        return false;
      }
    }
    return true;
  }

  bool start() {
    releaseSda();
    releaseScl();
    if (!waitSclHigh()) {
      return false;
    }
    i2cDelay();
    driveSdaLow();
    i2cDelay();
    driveSclLow();
    i2cDelay();
    return true;
  }

  void stop() {
    driveSdaLow();
    i2cDelay();
    releaseScl();
    waitSclHigh();
    i2cDelay();
    releaseSda();
    i2cDelay();
  }

  bool writeByte(uint8_t value) {
    for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
      if (value & mask) {
        releaseSda();
      } else {
        driveSdaLow();
      }
      i2cDelay();
      releaseScl();
      if (!waitSclHigh()) {
        return false;
      }
      i2cDelay();
      driveSclLow();
      i2cDelay();
    }

    releaseSda();
    i2cDelay();
    releaseScl();
    if (!waitSclHigh()) {
      return false;
    }
    i2cDelay();
    const bool ack = (digitalRead(sda_) == LOW);
    driveSclLow();
    i2cDelay();
    return ack;
  }

  bool readByte(uint8_t &value, bool ack) {
    value = 0;
    releaseSda();
    for (uint8_t i = 0; i < 8; ++i) {
      value <<= 1;
      i2cDelay();
      releaseScl();
      if (!waitSclHigh()) {
        return false;
      }
      i2cDelay();
      if (digitalRead(sda_) == HIGH) {
        value |= 1;
      }
      driveSclLow();
      i2cDelay();
    }

    if (ack) {
      driveSdaLow();
    } else {
      releaseSda();
    }
    i2cDelay();
    releaseScl();
    if (!waitSclHigh()) {
      return false;
    }
    i2cDelay();
    driveSclLow();
    releaseSda();
    i2cDelay();
    return true;
  }
};

struct EncoderState {
  bool ok = false;
  bool magnetOk = false;
  bool haveSample = false;
  uint8_t status = 0;
  uint16_t raw = 0;
  int32_t continuousCount = 0;
  float zeroRad = 0.0f;
  float rad = 0.0f;
  float prevRad = 0.0f;
  float velocityRadS = 0.0f;
  uint32_t lastUpdateUs = 0;
  uint8_t consecutiveFailures = 0;
};

HardwareSerial TMCSerial(1);
TMC2209Stepper driver(&TMCSerial, TMC_R_SENSE, TMC_DRIVER_ADDR);
SoftI2CBus swI2c(PIN_SW_I2C_SDA, PIN_SW_I2C_SCL, SW_I2C_FREQ_HZ);
Preferences preferences;

EncoderState motorEncoder;
EncoderState outputEncoder;

static uint8_t rxBuf[FRAME_OVERHEAD_SIZE + MAX_PAYLOAD_SIZE];
static uint16_t rxLen = 0;
static uint16_t rxExpectedLen = 0;
static uint8_t txPayload[MAX_PAYLOAD_SIZE];

static bool streaming = false;
static volatile bool driverEnabled = false;
static bool tmcUartOk = false;
static ActuatorMode mode = MODE_DISABLED;
static volatile uint32_t faultFlags = FAULT_NONE;
static portMUX_TYPE motionMux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t telemetrySeq = 0;
static uint32_t lastTelemetryUs = 0;
static uint32_t lastEncoderUpdateUs = 0;
static uint32_t lastPlannerUs = 0;

static volatile int64_t currentStepPosition = 0;
static int64_t baseTargetStepPosition = 0;
static volatile int64_t targetStepPosition = 0;
static float currentSpeedSps = 0.0f;
static float maxMoveSpeedSps = 1.0f;
static float moveAccelSps2 = 1.0f;
static volatile uint32_t stepIntervalUs = 0;
static volatile uint32_t activeStepIntervalUs = 0;
static volatile bool stepSchedulerEnabled = false;
static volatile bool stepPulseHigh = false;
static hw_timer_t *stepTimer = nullptr;

static float outputPerMotor = 1.0f;
static float outputOffsetRad = 0.0f;
static bool pidEnabled = false;
static float pidKp = 0.0f;
static float pidKi = 0.0f;
static float pidKd = 0.0f;
static float pidILimitMotorRad = 0.05f;
static float pidOutputLimitMotorRad = 0.25f;
static float pidIntegral = 0.0f;
static float pidLastError = 0.0f;
static float pidLastMeasurement = 0.0f;
static float pidFilteredDerivative = 0.0f;
static float velocityPidKp = 0.2f;
static float velocityPidKi = 2.0f;
static float velocityPidILimitMotorRad = 0.2f;
static float velocityPidIntegral = 0.0f;
static bool positionTargetValid = false;
static float positionTargetOutputRad = 0.0f;
static float velocityTargetOutputRadS = 0.0f;
static int8_t lastPositionDirection = 0;
static int8_t backlashDirection = 0;
static float backlashOffsetMotorRad = 0.0f;
static float backlashMotorRad = 0.0f;
static bool backlashCompEnabled = false;
static float resonanceFrequencyHz = 0.0f;
static bool resonanceDeratingEnabled = false;

static float torqueProxyKp = 3.0f;
static float torqueProxyLimitRad = 0.12f;
static float torqueProxyTargetRad = 0.0f;
static float torqueProxyMaxMotorVelocityRadS = 4.0f;
static float torqueProxyCommandMaxVelocityRadS = 4.0f;
static float torqueProxyMaxExcursionRad = 0.5f;
static float torqueProxyStartMotorRad = 0.0f;
static uint32_t torqueProxyStartUs = 0;
static uint32_t torqueProxyTimeoutUs = 3000000UL;

static bool missedStepCorrectionEnabled = true;
static float missedStepWarnMotorRad = 0.05f;
static float missedStepFaultMotorRad = 0.25f;
static float missedStepCorrectionRate = 0.25f;
static float motorSlipRad = 0.0f;
static bool stepReferenceAligned = false;

static bool currentControlEnabled = true;
static uint16_t idleCurrentMa = 0;
static uint16_t holdCurrentMa = 350;
static uint16_t runCurrentMa = MOTOR_RMS_CURRENT_MA;
static uint16_t commandedCurrentMa = 0;
static uint16_t appliedCurrentMa = 0;
static float currentDownshiftDelayS = 0.5f;
static uint32_t lastHighCurrentDemandUs = 0;

static uint8_t controlState = CONTROL_IDLE;
static uint8_t autotuneLoopSelector = 0;
static float autotuneMaxAmplitudeRad = 0.4f;
static float autotuneMaxDurationS = 15.0f;
static float autotuneMaxDeflectionRad = 0.25f;
static float autotuneAmplitudeRad = 0.0f;
static float autotuneMaxVelocityRadS = 0.0f;
static float autotuneActiveMaxDeflectionRad = 0.25f;
static uint32_t autotuneStartUs = 0;
static uint32_t autotuneDurationUs = 0;
static char lastControlFault[32] = "";

static bool chirpActive = false;
static uint32_t chirpStartUs = 0;
static float chirpCenterMotorRad = 0.0f;
static float chirpAmplitudeRad = 0.18f;
static float chirpStartHz = 0.8f;
static float chirpEndHz = 70.0f;
static float chirpDurationS = 12.0f;
static float chirpMaxDeflectionRad = 0.25f;

static float clampFloat(float value, float lo, float hi);
static float sanitizeNonNegativeFloat(float value, float fallback);
static float sanitizePidGain(float value);
static float sanitizePidLimit(float value, float fallback);
static void stopMotion();
static void setMode(ActuatorMode nextMode);

static void loadConfig() {
  preferences.begin("actuator", true);
  outputPerMotor = preferences.getFloat("out_per_m", 1.0f);
  outputOffsetRad = preferences.getFloat("out_off", 0.0f);
  pidEnabled = preferences.getBool("pid_en", false);
  pidKp = preferences.getFloat("pid_kp", 0.0f);
  pidKi = preferences.getFloat("pid_ki", 0.0f);
  pidKd = preferences.getFloat("pid_kd", 0.0f);
  pidILimitMotorRad = preferences.getFloat("pid_i_lim", 0.05f);
  pidOutputLimitMotorRad = preferences.getFloat("pid_o_lim", 0.25f);
  velocityPidKp = preferences.getFloat("vel_kp", 0.2f);
  velocityPidKi = preferences.getFloat("vel_ki", 2.0f);
  velocityPidILimitMotorRad = preferences.getFloat("vel_i_lim", 0.2f);
  torqueProxyKp = preferences.getFloat("tq_kp", 3.0f);
  torqueProxyLimitRad = preferences.getFloat("tq_lim", 0.12f);
  torqueProxyMaxMotorVelocityRadS = preferences.getFloat("tq_vel", 4.0f);
  torqueProxyTimeoutUs = (uint32_t)(preferences.getFloat("tq_timeout", 3.0f) * 1000000.0f);
  missedStepCorrectionEnabled = preferences.getBool("slip_en", true);
  missedStepWarnMotorRad = preferences.getFloat("slip_warn", 0.05f);
  missedStepFaultMotorRad = preferences.getFloat("slip_fault", 0.25f);
  missedStepCorrectionRate = preferences.getFloat("slip_rate", 0.25f);
  currentControlEnabled = preferences.getBool("cur_en", true);
  idleCurrentMa = (uint16_t)preferences.getUInt("cur_idle", 0);
  holdCurrentMa = (uint16_t)preferences.getUInt("cur_hold", 350);
  runCurrentMa = (uint16_t)preferences.getUInt("cur_run", MOTOR_RMS_CURRENT_MA);
  currentDownshiftDelayS = preferences.getFloat("cur_delay", 0.5f);
  autotuneMaxAmplitudeRad = preferences.getFloat("at_amp", 0.4f);
  autotuneMaxDurationS = preferences.getFloat("at_dur", 15.0f);
  autotuneMaxDeflectionRad = preferences.getFloat("at_defl", 0.25f);
  backlashMotorRad = preferences.getFloat("backlash_m", 0.0f);
  backlashCompEnabled = preferences.getBool("backlash_en", false);
  resonanceFrequencyHz = preferences.getFloat("res_hz", 0.0f);
  resonanceDeratingEnabled = preferences.getBool("res_derate", false);
  preferences.end();
  if (!isfinite(outputPerMotor) || fabsf(outputPerMotor) < 1.0e-9f) {
    outputPerMotor = 1.0f;
  }
  if (!isfinite(outputOffsetRad)) {
    outputOffsetRad = 0.0f;
  }
  if (!isfinite(pidKp) || pidKp < 0.0f) {
    pidKp = 0.0f;
  }
  if (!isfinite(pidKi) || pidKi < 0.0f) {
    pidKi = 0.0f;
  }
  if (!isfinite(pidKd) || pidKd < 0.0f) {
    pidKd = 0.0f;
  }
  if (!isfinite(pidILimitMotorRad) || pidILimitMotorRad < 0.0f) {
    pidILimitMotorRad = 0.05f;
  }
  if (pidILimitMotorRad > 10.0f) {
    pidILimitMotorRad = 10.0f;
  }
  if (!isfinite(pidOutputLimitMotorRad) || pidOutputLimitMotorRad < 0.0f) {
    pidOutputLimitMotorRad = 0.25f;
  }
  if (pidOutputLimitMotorRad > 10.0f) {
    pidOutputLimitMotorRad = 10.0f;
  }
  velocityPidKp = sanitizePidGain(velocityPidKp);
  velocityPidKi = sanitizePidGain(velocityPidKi);
  velocityPidILimitMotorRad = sanitizePidLimit(velocityPidILimitMotorRad, 0.2f);
  torqueProxyKp = sanitizePidGain(torqueProxyKp);
  torqueProxyLimitRad = clampFloat(sanitizeNonNegativeFloat(torqueProxyLimitRad, 0.12f), 0.001f, 10.0f);
  torqueProxyMaxMotorVelocityRadS =
      clampFloat(sanitizeNonNegativeFloat(torqueProxyMaxMotorVelocityRadS, 4.0f), 0.01f, MAX_VELOCITY_RAD_S);
  if (torqueProxyTimeoutUs < 50000UL) {
    torqueProxyTimeoutUs = 3000000UL;
  }
  missedStepWarnMotorRad = clampFloat(sanitizeNonNegativeFloat(missedStepWarnMotorRad, 0.05f), 0.0f, 10.0f);
  missedStepFaultMotorRad = clampFloat(sanitizeNonNegativeFloat(missedStepFaultMotorRad, 0.25f), 0.001f, 10.0f);
  if (missedStepWarnMotorRad > missedStepFaultMotorRad) {
    missedStepWarnMotorRad = missedStepFaultMotorRad;
  }
  missedStepCorrectionRate = clampFloat(sanitizeNonNegativeFloat(missedStepCorrectionRate, 0.25f), 0.0f, 1.0f);
  if (holdCurrentMa > runCurrentMa) {
    holdCurrentMa = runCurrentMa;
  }
  if (runCurrentMa == 0) {
    runCurrentMa = MOTOR_RMS_CURRENT_MA;
  }
  currentDownshiftDelayS = clampFloat(sanitizeNonNegativeFloat(currentDownshiftDelayS, 0.5f), 0.0f, 30.0f);
  autotuneMaxAmplitudeRad = clampFloat(sanitizeNonNegativeFloat(autotuneMaxAmplitudeRad, 0.4f), 0.001f, 10.0f);
  autotuneMaxDurationS = clampFloat(sanitizeNonNegativeFloat(autotuneMaxDurationS, 15.0f), 0.05f, 120.0f);
  autotuneMaxDeflectionRad = clampFloat(sanitizeNonNegativeFloat(autotuneMaxDeflectionRad, 0.25f), 0.001f, 10.0f);
  if (!isfinite(backlashMotorRad) || backlashMotorRad < 0.0f) {
    backlashMotorRad = 0.0f;
  }
  if (backlashMotorRad > 10.0f) {
    backlashMotorRad = 10.0f;
  }
  if (!isfinite(resonanceFrequencyHz) || resonanceFrequencyHz < 0.0f) {
    resonanceFrequencyHz = 0.0f;
  }
}

static bool saveConfig() {
  preferences.begin("actuator", false);
  bool ok = true;
  ok = preferences.putFloat("out_per_m", outputPerMotor) > 0 && ok;
  ok = preferences.putFloat("out_off", outputOffsetRad) > 0 && ok;
  ok = preferences.putBool("pid_en", pidEnabled) > 0 && ok;
  ok = preferences.putFloat("pid_kp", pidKp) > 0 && ok;
  ok = preferences.putFloat("pid_ki", pidKi) > 0 && ok;
  ok = preferences.putFloat("pid_kd", pidKd) > 0 && ok;
  ok = preferences.putFloat("pid_i_lim", pidILimitMotorRad) > 0 && ok;
  ok = preferences.putFloat("pid_o_lim", pidOutputLimitMotorRad) > 0 && ok;
  ok = preferences.putFloat("vel_kp", velocityPidKp) > 0 && ok;
  ok = preferences.putFloat("vel_ki", velocityPidKi) > 0 && ok;
  ok = preferences.putFloat("vel_i_lim", velocityPidILimitMotorRad) > 0 && ok;
  ok = preferences.putFloat("tq_kp", torqueProxyKp) > 0 && ok;
  ok = preferences.putFloat("tq_lim", torqueProxyLimitRad) > 0 && ok;
  ok = preferences.putFloat("tq_vel", torqueProxyMaxMotorVelocityRadS) > 0 && ok;
  ok = preferences.putFloat("tq_timeout", (float)torqueProxyTimeoutUs * 1.0e-6f) > 0 && ok;
  ok = preferences.putBool("slip_en", missedStepCorrectionEnabled) > 0 && ok;
  ok = preferences.putFloat("slip_warn", missedStepWarnMotorRad) > 0 && ok;
  ok = preferences.putFloat("slip_fault", missedStepFaultMotorRad) > 0 && ok;
  ok = preferences.putFloat("slip_rate", missedStepCorrectionRate) > 0 && ok;
  ok = preferences.putBool("cur_en", currentControlEnabled) > 0 && ok;
  ok = preferences.putUInt("cur_idle", idleCurrentMa) > 0 && ok;
  ok = preferences.putUInt("cur_hold", holdCurrentMa) > 0 && ok;
  ok = preferences.putUInt("cur_run", runCurrentMa) > 0 && ok;
  ok = preferences.putFloat("cur_delay", currentDownshiftDelayS) > 0 && ok;
  ok = preferences.putFloat("at_amp", autotuneMaxAmplitudeRad) > 0 && ok;
  ok = preferences.putFloat("at_dur", autotuneMaxDurationS) > 0 && ok;
  ok = preferences.putFloat("at_defl", autotuneMaxDeflectionRad) > 0 && ok;
  ok = preferences.putFloat("backlash_m", backlashMotorRad) > 0 && ok;
  ok = preferences.putBool("backlash_en", backlashCompEnabled) > 0 && ok;
  ok = preferences.putFloat("res_hz", resonanceFrequencyHz) > 0 && ok;
  ok = preferences.putBool("res_derate", resonanceDeratingEnabled) > 0 && ok;
  preferences.end();
  return ok;
}

static uint16_t crc16Update(uint16_t crc, uint8_t data) {
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; ++i) {
    if (crc & 0x8000) {
      crc = (uint16_t)((crc << 1) ^ 0x1021);
    } else {
      crc = (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static uint16_t crc16CcittFalse(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; ++i) {
    crc = crc16Update(crc, data[i]);
  }
  return crc;
}

static uint16_t readLe16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t readLe32(const uint8_t *p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static float readFloatLe(const uint8_t *p) {
  uint32_t raw = readLe32(p);
  float value = 0.0f;
  memcpy(&value, &raw, sizeof(value));
  return value;
}

static void appendU16(uint8_t *&p, uint16_t value) {
  *p++ = (uint8_t)(value & 0xFF);
  *p++ = (uint8_t)(value >> 8);
}

static void appendU32(uint8_t *&p, uint32_t value) {
  *p++ = (uint8_t)(value & 0xFF);
  *p++ = (uint8_t)((value >> 8) & 0xFF);
  *p++ = (uint8_t)((value >> 16) & 0xFF);
  *p++ = (uint8_t)((value >> 24) & 0xFF);
}

static void appendI32(uint8_t *&p, int32_t value) {
  appendU32(p, (uint32_t)value);
}

static void appendU64(uint8_t *&p, uint64_t value) {
  for (uint8_t i = 0; i < 8; ++i) {
    *p++ = (uint8_t)((value >> (8 * i)) & 0xFF);
  }
}

static void appendFloat(uint8_t *&p, float value) {
  uint32_t raw = 0;
  memcpy(&raw, &value, sizeof(raw));
  appendU32(p, raw);
}

static float clampFloat(float value, float lo, float hi);

static int64_t abs64(int64_t value) {
  return value < 0 ? -value : value;
}

static int64_t readCurrentStepPosition() {
  portENTER_CRITICAL(&motionMux);
  const int64_t value = currentStepPosition;
  portEXIT_CRITICAL(&motionMux);
  return value;
}

static void setCurrentStepPosition(int64_t value) {
  portENTER_CRITICAL(&motionMux);
  currentStepPosition = value;
  portEXIT_CRITICAL(&motionMux);
}

static int64_t readTargetStepPosition() {
  portENTER_CRITICAL(&motionMux);
  const int64_t value = targetStepPosition;
  portEXIT_CRITICAL(&motionMux);
  return value;
}

static void setTargetStepPosition(int64_t value) {
  portENTER_CRITICAL(&motionMux);
  targetStepPosition = value;
  portEXIT_CRITICAL(&motionMux);
}

static void setBaseAndTargetStepPosition(int64_t value) {
  baseTargetStepPosition = value;
  setTargetStepPosition(value);
}

static void resetPidState() {
  pidIntegral = 0.0f;
  pidLastError = 0.0f;
  pidLastMeasurement = outputEncoder.rad;
  pidFilteredDerivative = 0.0f;
  velocityPidIntegral = 0.0f;
}

static float sanitizeNonNegativeFloat(float value, float fallback) {
  if (!isfinite(value) || value < 0.0f) {
    return fallback;
  }
  return value;
}

static float sanitizePidGain(float value) {
  return sanitizeNonNegativeFloat(value, 0.0f);
}

static float sanitizePidLimit(float value, float fallback) {
  return clampFloat(sanitizeNonNegativeFloat(value, fallback), 0.0f, 10.0f);
}

static uint32_t speedToStepIntervalUs(float speedSps) {
  if (!isfinite(speedSps) || speedSps <= 0.0f) {
    return 0;
  }
  const float clampedSpeed = clampFloat(speedSps, 0.0f, MAX_STEP_RATE_SPS);
  uint32_t intervalUs = (uint32_t)ceilf(1000000.0f / clampedSpeed);
  if (intervalUs < MIN_STEP_INTERVAL_US) {
    intervalUs = MIN_STEP_INTERVAL_US;
  }
  const uint32_t minPulseInterval = STEP_PULSE_US + MIN_STEP_LOW_US;
  if (intervalUs < minPulseInterval) {
    intervalUs = minPulseInterval;
  }
  return intervalUs;
}

static void armStepTimer(uint32_t delayUs) {
  if (stepTimer == nullptr) {
    return;
  }
  timerStop(stepTimer);
  timerWrite(stepTimer, 0);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timerAlarm(stepTimer, delayUs < 1 ? 1 : delayUs, false, 0);
#else
  timerAlarmWrite(stepTimer, delayUs < 1 ? 1 : delayUs, false);
  timerAlarmEnable(stepTimer);
#endif
  timerStart(stepTimer);
}

static void IRAM_ATTR armStepTimerFromIsr(uint32_t delayUs) {
  if (stepTimer == nullptr) {
    return;
  }
  timerWrite(stepTimer, 0);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timerAlarm(stepTimer, delayUs < 1 ? 1 : delayUs, false, 0);
#else
  timerAlarmWrite(stepTimer, delayUs < 1 ? 1 : delayUs, false);
  timerAlarmEnable(stepTimer);
#endif
}

static void setStepIntervalUs(uint32_t intervalUs) {
  bool shouldArm = false;
  bool shouldStop = false;
  portENTER_CRITICAL(&motionMux);
  stepIntervalUs = intervalUs;
  if (intervalUs == 0 || !driverEnabled || faultFlags != FAULT_NONE) {
    stepSchedulerEnabled = false;
    if (!stepPulseHigh) {
      activeStepIntervalUs = 0;
      shouldStop = true;
    }
  } else if (!stepSchedulerEnabled) {
    stepSchedulerEnabled = true;
    shouldArm = true;
  }
  portEXIT_CRITICAL(&motionMux);

  if (shouldStop) {
    if (stepTimer != nullptr) {
      timerStop(stepTimer);
    }
    gpio_set_level((gpio_num_t)PIN_STEP, 0);
  } else if (shouldArm) {
    armStepTimer(1);
  }
}

static void stopStepScheduler(bool forceLow) {
  bool shouldStop = false;
  portENTER_CRITICAL(&motionMux);
  stepIntervalUs = 0;
  activeStepIntervalUs = 0;
  stepSchedulerEnabled = false;
  if (forceLow || !stepPulseHigh) {
    stepPulseHigh = false;
    shouldStop = true;
  }
  portEXIT_CRITICAL(&motionMux);

  if (shouldStop) {
    if (stepTimer != nullptr) {
      timerStop(stepTimer);
    }
    gpio_set_level((gpio_num_t)PIN_STEP, 0);
  }
}

static void resetStepPositions() {
  stopStepScheduler(true);
  portENTER_CRITICAL(&motionMux);
  currentStepPosition = 0;
  targetStepPosition = 0;
  portEXIT_CRITICAL(&motionMux);
  baseTargetStepPosition = 0;
  positionTargetValid = false;
  velocityTargetOutputRadS = 0.0f;
  torqueProxyTargetRad = torqueProxyRad();
  chirpActive = false;
  resetPidState();
}

static float clampFloat(float value, float lo, float hi) {
  if (value < lo) {
    return lo;
  }
  if (value > hi) {
    return hi;
  }
  return value;
}

static float predictedOutputRad() {
  return outputPerMotor * motorEncoder.rad + outputOffsetRad;
}

static float torqueProxyRad() {
  return outputEncoder.rad - predictedOutputRad();
}

static float outputTargetToMotorRad(float outputTargetRad) {
  if (fabsf(outputPerMotor) < 1.0e-9f) {
    return (float)readCurrentStepPosition() * MOTOR_RAD_PER_MICROSTEP;
  }
  return (outputTargetRad - outputOffsetRad) / outputPerMotor + backlashOffsetMotorRad;
}

static void setLastControlFault(const char *message) {
  if (message == nullptr) {
    lastControlFault[0] = '\0';
    return;
  }
  strncpy(lastControlFault, message, sizeof(lastControlFault) - 1);
  lastControlFault[sizeof(lastControlFault) - 1] = '\0';
}

static void setFaultAndStop(uint32_t fault, const char *message) {
  faultFlags |= fault;
  setLastControlFault(message);
  stopMotion();
  setMode(MODE_FAULT);
}

static void applyDriverCurrent(uint16_t currentMa) {
  if (currentMa == appliedCurrentMa) {
    commandedCurrentMa = currentMa;
    return;
  }
  commandedCurrentMa = currentMa;
  appliedCurrentMa = currentMa;
  if (tmcUartOk && currentMa > 0) {
    driver.rms_current(currentMa);
  }
}

static void serviceCurrentControl(uint32_t nowUs) {
  if (!driverEnabled || mode == MODE_DISABLED || mode == MODE_FAULT || faultFlags != FAULT_NONE) {
    applyDriverCurrent(0);
    return;
  }
  uint16_t targetMa = runCurrentMa;
  if (currentControlEnabled) {
    const bool moving = fabsf(currentSpeedSps) > 0.5f || abs64(readTargetStepPosition() - readCurrentStepPosition()) > 1;
    const bool highDeflection = fabsf(torqueProxyRad()) > torqueProxyLimitRad * 0.5f;
    if (moving || highDeflection || controlState == CONTROL_AUTOTUNE_RUNNING) {
      lastHighCurrentDemandUs = nowUs;
      targetMa = runCurrentMa;
    } else {
      const uint32_t delayUs = (uint32_t)(currentDownshiftDelayS * 1000000.0f);
      targetMa = ((uint32_t)(nowUs - lastHighCurrentDemandUs) >= delayUs) ? holdCurrentMa : runCurrentMa;
    }
  }
  applyDriverCurrent(targetMa);
}

static void enableDriver(bool enable) {
  if (!enable) {
    stopStepScheduler(true);
    applyDriverCurrent(0);
  } else if (commandedCurrentMa == 0) {
    applyDriverCurrent(runCurrentMa);
    lastHighCurrentDemandUs = micros();
  }
  portENTER_CRITICAL(&motionMux);
  driverEnabled = enable;
  portEXIT_CRITICAL(&motionMux);
  digitalWrite(PIN_ENABLE, enable ? LOW : HIGH);
}

static void stopMotion() {
  stopStepScheduler(false);
  const int64_t current = readCurrentStepPosition();
  portENTER_CRITICAL(&motionMux);
  targetStepPosition = current;
  portEXIT_CRITICAL(&motionMux);
  baseTargetStepPosition = current;
  chirpActive = false;
  positionTargetValid = false;
  velocityTargetOutputRadS = 0.0f;
  torqueProxyTargetRad = torqueProxyRad();
  resetPidState();
  currentSpeedSps = 0.0f;
}

static bool hwReadRegister(uint8_t reg, uint8_t *data, size_t len) {
  Wire.beginTransmission(AS5600_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  const int count = Wire.requestFrom((uint8_t)AS5600_I2C_ADDR, (uint8_t)len);
  if (count != (int)len) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    data[i] = (uint8_t)Wire.read();
  }
  return true;
}

static bool readAS5600SnapshotHardware(uint8_t &status, uint16_t &raw) {
  uint8_t buf[3] = {0, 0, 0};
  if (!hwReadRegister(0x0B, buf, sizeof(buf))) {
    return false;
  }
  status = buf[0];
  raw = ((uint16_t)(buf[1] & 0x0F) << 8) | buf[2];
  return true;
}

static bool readAS5600SnapshotSoftware(uint8_t &status, uint16_t &raw) {
  uint8_t buf[3] = {0, 0, 0};
  if (!swI2c.readRegister(AS5600_I2C_ADDR, 0x0B, buf, sizeof(buf))) {
    return false;
  }
  status = buf[0];
  raw = ((uint16_t)(buf[1] & 0x0F) << 8) | buf[2];
  return true;
}

static bool magnetStatusOk(uint8_t status) {
  const bool magnetDetected = (status & 0x20) != 0;
  const bool magnetTooWeak = (status & 0x10) != 0;
  const bool magnetTooStrong = (status & 0x08) != 0;
  return magnetDetected && !magnetTooWeak && !magnetTooStrong;
}

static void updateEncoderState(EncoderState &enc, bool ok, uint8_t status, uint16_t raw, uint32_t nowUs) {
  // For this bench setup the AS5600 magnet strength flags are currently
  // diagnostic only; position reads are accepted as long as I2C succeeds.
  enc.ok = ok;
  enc.status = status;

  if (!ok) {
    enc.magnetOk = false;
    if (enc.consecutiveFailures < 255) {
      enc.consecutiveFailures++;
    }
    if (enc.consecutiveFailures >= ENCODER_FAILURE_FAULT_COUNT) {
      faultFlags |= FAULT_ENCODER_DISAGREEMENT;
    }
    return;
  }

  enc.consecutiveFailures = 0;
  enc.magnetOk = magnetStatusOk(status);
  enc.raw = raw;
  if (!enc.haveSample) {
    enc.continuousCount = raw;
    enc.rad = (float)enc.continuousCount * ENCODER_RAD_PER_COUNT - enc.zeroRad;
    enc.prevRad = enc.rad;
    enc.velocityRadS = 0.0f;
    enc.lastUpdateUs = nowUs;
    enc.haveSample = true;
    return;
  }

  int16_t diff = (int16_t)raw - (int16_t)(enc.continuousCount & 0x0FFF);
  if (diff > 2048) {
    diff -= 4096;
  } else if (diff < -2048) {
    diff += 4096;
  }
  enc.continuousCount += diff;

  const float dt = (nowUs - enc.lastUpdateUs) * 1.0e-6f;
  enc.lastUpdateUs = nowUs;
  enc.rad = (float)enc.continuousCount * ENCODER_RAD_PER_COUNT - enc.zeroRad;
  if (dt > 0.0f && dt < 0.25f) {
    enc.velocityRadS = (enc.rad - enc.prevRad) / dt;
  }
  enc.prevRad = enc.rad;

}

static void serviceEncoders() {
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastEncoderUpdateUs) < ENCODER_UPDATE_PERIOD_US) {
    return;
  }
  lastEncoderUpdateUs = nowUs;

  uint8_t motorStatus = 0;
  uint16_t motorRaw = 0;
  const bool motorOk = readAS5600SnapshotSoftware(motorStatus, motorRaw);
  updateEncoderState(motorEncoder, motorOk, motorStatus, motorRaw, nowUs);

  uint8_t outputStatus = 0;
  uint16_t outputRaw = 0;
  const bool outputOk = readAS5600SnapshotHardware(outputStatus, outputRaw);
  updateEncoderState(outputEncoder, outputOk, outputStatus, outputRaw, nowUs);
}

static void zeroEncoder(EncoderState &enc) {
  enc.zeroRad = (float)enc.continuousCount * ENCODER_RAD_PER_COUNT;
  enc.rad = 0.0f;
  enc.prevRad = 0.0f;
  enc.velocityRadS = 0.0f;
}

static void alignStepReferenceToMotorEncoder() {
  if (!motorEncoder.ok) {
    return;
  }
  const int64_t alignedSteps = (int64_t)roundf(motorEncoder.rad / MOTOR_RAD_PER_MICROSTEP);
  stopStepScheduler(false);
  portENTER_CRITICAL(&motionMux);
  currentStepPosition = alignedSteps;
  targetStepPosition = alignedSteps;
  portEXIT_CRITICAL(&motionMux);
  baseTargetStepPosition = alignedSteps;
  currentSpeedSps = 0.0f;
  motorSlipRad = 0.0f;
  stepReferenceAligned = true;
}

static void configureTmc2209() {
  TMCSerial.begin(TMC_UART_BAUD, SERIAL_8N1, PIN_TMC_UART_RX, PIN_TMC_UART_TX);
  driver.begin();
  driver.pdn_disable(true);
  driver.I_scale_analog(false);
  driver.mstep_reg_select(true);
  driver.toff(5);
  driver.blank_time(24);
  driver.rms_current(runCurrentMa);
  appliedCurrentMa = runCurrentMa;
  commandedCurrentMa = runCurrentMa;
  driver.microsteps(MOTOR_MICROSTEPS);
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  tmcUartOk = (driver.test_connection() == 0);
}

static void IRAM_ATTR onStepTimer() {
  uint32_t nextDelayUs = 0;

  portENTER_CRITICAL_ISR(&motionMux);
  if (stepPulseHigh) {
    gpio_set_level((gpio_num_t)PIN_STEP, 0);
    stepPulseHigh = false;
    if (stepSchedulerEnabled && driverEnabled && faultFlags == FAULT_NONE && stepIntervalUs > 0) {
      nextDelayUs = activeStepIntervalUs > STEP_PULSE_US
                        ? activeStepIntervalUs - STEP_PULSE_US
                        : MIN_STEP_LOW_US;
    } else {
      stepSchedulerEnabled = false;
    }
  } else if (stepSchedulerEnabled && driverEnabled && faultFlags == FAULT_NONE && stepIntervalUs > 0) {
    const int64_t distance = targetStepPosition - currentStepPosition;
    if (distance == 0) {
      stepSchedulerEnabled = false;
      stepIntervalUs = 0;
      activeStepIntervalUs = 0;
    } else {
      const int8_t stepDir = distance > 0 ? 1 : -1;
      activeStepIntervalUs = stepIntervalUs;
      gpio_set_level((gpio_num_t)PIN_DIR, stepDir > 0 ? 1 : 0);
      gpio_set_level((gpio_num_t)PIN_STEP, 1);
      currentStepPosition += stepDir;
      stepPulseHigh = true;
      nextDelayUs = STEP_PULSE_US;
    }
  } else {
    stepSchedulerEnabled = false;
  }
  portEXIT_CRITICAL_ISR(&motionMux);

  if (nextDelayUs > 0) {
    armStepTimerFromIsr(nextDelayUs);
  }
}

static void configureStepTimer() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  stepTimer = timerBegin(STEP_TIMER_HZ);
  timerAttachInterrupt(stepTimer, &onStepTimer);
#else
  // Core 2.x uses an 80 MHz APB timer divided to a 1 MHz microsecond clock.
  stepTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(stepTimer, &onStepTimer, true);
#endif
  timerStop(stepTimer);
}

static bool modeAllowsMotion() {
  return mode == MODE_CALIBRATION ||
         mode == MODE_OPEN_LOOP ||
         mode == MODE_POSITION ||
         mode == MODE_VELOCITY ||
         mode == MODE_TORQUE_PROXY;
}

static void setMode(ActuatorMode nextMode) {
  if (nextMode == MODE_DISABLED) {
    stopMotion();
    mode = MODE_DISABLED;
    enableDriver(false);
    return;
  }
  if (nextMode == MODE_FAULT) {
    stopMotion();
    mode = MODE_FAULT;
    enableDriver(false);
    return;
  }

  if (faultFlags != FAULT_NONE) {
    mode = MODE_FAULT;
    enableDriver(false);
    return;
  }

  if (mode != nextMode) {
    chirpActive = false;
    positionTargetValid = false;
    velocityTargetOutputRadS = 0.0f;
    resetPidState();
  }
  if ((nextMode == MODE_CALIBRATION ||
       nextMode == MODE_OPEN_LOOP ||
       nextMode == MODE_POSITION ||
       nextMode == MODE_VELOCITY ||
       nextMode == MODE_TORQUE_PROXY) &&
      !stepReferenceAligned) {
    serviceEncoders();
    alignStepReferenceToMotorEncoder();
  }
  if (nextMode == MODE_TORQUE_PROXY) {
    torqueProxyTargetRad = torqueProxyRad();
    torqueProxyStartMotorRad = motorEncoder.rad;
    torqueProxyStartUs = micros();
    torqueProxyCommandMaxVelocityRadS = torqueProxyMaxMotorVelocityRadS;
    torqueProxyMaxExcursionRad = 1000000.0f;
  }
  mode = nextMode;
  enableDriver(modeAllowsMotion());
}

static void serviceMissedStepCorrection() {
  if (!motorEncoder.ok || !missedStepCorrectionEnabled || faultFlags != FAULT_NONE) {
    return;
  }
  if (!stepReferenceAligned) {
    alignStepReferenceToMotorEncoder();
    return;
  }
  const int64_t issuedSteps = readCurrentStepPosition();
  const float issuedMotorRad = (float)issuedSteps * MOTOR_RAD_PER_MICROSTEP;
  motorSlipRad = issuedMotorRad - motorEncoder.rad;
  if (fabsf(motorSlipRad) >= missedStepFaultMotorRad) {
    setFaultAndStop(FAULT_MISSED_STEP, "MISSED_STEP");
    return;
  }
  if (fabsf(motorSlipRad) >= missedStepWarnMotorRad && missedStepCorrectionRate > 0.0f) {
    const int64_t correctionSteps = (int64_t)roundf(
        (motorSlipRad * missedStepCorrectionRate) / MOTOR_RAD_PER_MICROSTEP);
    if (correctionSteps != 0) {
      setCurrentStepPosition(issuedSteps - correctionSteps);
    }
  }
}

static void serviceAutotune(uint32_t nowUs) {
  if (controlState != CONTROL_AUTOTUNE_RUNNING) {
    return;
  }
  if (fabsf(torqueProxyRad()) > autotuneActiveMaxDeflectionRad) {
    controlState = CONTROL_AUTOTUNE_FAILED;
    setFaultAndStop(FAULT_AUTOTUNE_FAILED, "AUTOTUNE_DEFLECTION");
    return;
  }
  if ((uint32_t)(nowUs - autotuneStartUs) < autotuneDurationUs) {
    const float elapsedS = (nowUs - autotuneStartUs) * 1.0e-6f;
    const float phase = TWO_PI_F * 2.0f * elapsedS;
    const float dither = autotuneAmplitudeRad * sinf(phase);
    if (mode == MODE_POSITION && positionTargetValid) {
      const float motorTargetRad = outputTargetToMotorRad(positionTargetOutputRad + dither);
      setBaseAndTargetStepPosition((int64_t)roundf(motorTargetRad / MOTOR_RAD_PER_MICROSTEP));
    }
    return;
  }

  if (autotuneLoopSelector == AUTOTUNE_VELOCITY || autotuneLoopSelector == AUTOTUNE_BOTH) {
    velocityPidKp = fmaxf(0.05f, autotuneMaxVelocityRadS * 0.04f);
    velocityPidKi = fmaxf(0.1f, velocityPidKp * 8.0f);
  }
  if (autotuneLoopSelector == AUTOTUNE_POSITION || autotuneLoopSelector == AUTOTUNE_BOTH) {
    pidEnabled = true;
    pidKp = fmaxf(0.1f, autotuneAmplitudeRad * 3.0f);
    pidKi = fmaxf(0.01f, pidKp * 0.15f);
    pidKd = fmaxf(0.0f, pidKp * 0.01f);
  }
  controlState = CONTROL_AUTOTUNE_SUCCESS;
  setLastControlFault("");
  resetPidState();
}

static void serviceAdvancedTargets(float dt, uint32_t nowUs) {
  serviceAutotune(nowUs);

  if (chirpActive) {
    if (mode != MODE_CALIBRATION || faultFlags != FAULT_NONE) {
      chirpActive = false;
      setBaseAndTargetStepPosition(readCurrentStepPosition());
    } else {
      const float elapsedS = (nowUs - chirpStartUs) * 1.0e-6f;
      if (elapsedS >= chirpDurationS) {
        chirpActive = false;
        setBaseAndTargetStepPosition(readCurrentStepPosition());
      } else {
        const float deflection = torqueProxyRad();
        if (fabsf(deflection) > chirpMaxDeflectionRad) {
          chirpActive = false;
          setBaseAndTargetStepPosition(readCurrentStepPosition());
        } else {
          const float sweepRate = (chirpEndHz - chirpStartHz) / chirpDurationS;
          const float phase = TWO_PI_F * (chirpStartHz * elapsedS + 0.5f * sweepRate * elapsedS * elapsedS);
          const float frequency = chirpStartHz + sweepRate * elapsedS;
          const float targetMotorRad = chirpCenterMotorRad + chirpAmplitudeRad * sinf(phase);
          const int64_t targetSteps = (int64_t)roundf(targetMotorRad / MOTOR_RAD_PER_MICROSTEP);
          setBaseAndTargetStepPosition(targetSteps);
          maxMoveSpeedSps = clampFloat(
              chirpAmplitudeRad * TWO_PI_F * fmaxf(frequency, 0.1f) / MOTOR_RAD_PER_MICROSTEP * 1.25f,
              1.0f,
              MAX_STEP_RATE_SPS);
          moveAccelSps2 = clampFloat(maxMoveSpeedSps * 40.0f, 1.0f, MAX_STEP_RATE_SPS * 20.0f);
        }
      }
    }
    return;
  }

  if (mode == MODE_VELOCITY) {
    if (fabsf(outputPerMotor) < 1.0e-9f) {
      setFaultAndStop(FAULT_CONTROL_ERROR, "BAD_RATIO");
      return;
    }
    const float motorVelocityRadS = velocityTargetOutputRadS / outputPerMotor;
    const float deltaMotorRad = motorVelocityRadS * dt;
    baseTargetStepPosition += (int64_t)roundf(deltaMotorRad / MOTOR_RAD_PER_MICROSTEP);
    setTargetStepPosition(baseTargetStepPosition);
    maxMoveSpeedSps = clampFloat(fabsf(motorVelocityRadS) / MOTOR_RAD_PER_MICROSTEP, 1.0f, MAX_STEP_RATE_SPS);
    return;
  }

  if (mode == MODE_TORQUE_PROXY) {
    const uint32_t elapsedUs = nowUs - torqueProxyStartUs;
    if (elapsedUs > torqueProxyTimeoutUs) {
      setFaultAndStop(FAULT_CONTROL_ERROR, "TORQUE_TIMEOUT");
      return;
    }
    if (fabsf(motorEncoder.rad - torqueProxyStartMotorRad) > torqueProxyMaxExcursionRad) {
      setFaultAndStop(FAULT_CONTROL_ERROR, "TORQUE_EXCURSION");
      return;
    }
    const float error = torqueProxyTargetRad - torqueProxyRad();
    float motorVelocityRadS = -torqueProxyKp * error;
    motorVelocityRadS = clampFloat(
        motorVelocityRadS,
        -torqueProxyCommandMaxVelocityRadS,
        torqueProxyCommandMaxVelocityRadS);
    const float deltaMotorRad = motorVelocityRadS * dt;
    baseTargetStepPosition += (int64_t)roundf(deltaMotorRad / MOTOR_RAD_PER_MICROSTEP);
    setTargetStepPosition(baseTargetStepPosition);
    maxMoveSpeedSps = clampFloat(fabsf(motorVelocityRadS) / MOTOR_RAD_PER_MICROSTEP, 1.0f, MAX_STEP_RATE_SPS);
    moveAccelSps2 = clampFloat(maxMoveSpeedSps * 20.0f, 1.0f, MAX_STEP_RATE_SPS * 20.0f);
    return;
  }

  if (mode == MODE_POSITION && positionTargetValid && pidEnabled) {
    const float error = positionTargetOutputRad - outputEncoder.rad;
    const float measurementDelta = outputEncoder.rad - pidLastMeasurement;
    pidLastMeasurement = outputEncoder.rad;
    const float rawDerivative = dt > 0.0f ? -measurementDelta / dt : 0.0f;
    pidFilteredDerivative += 0.2f * (rawDerivative - pidFilteredDerivative);
    pidLastError = error;

    const float proportional = pidKp * error;
    const float derivative = pidKd * pidFilteredDerivative;
    const float unclampedWithoutIntegral = proportional + derivative;
    float candidateIntegral = pidIntegral;
    if (pidKi > 1.0e-9f) {
      candidateIntegral += error * dt;
      const float integralLimit = pidILimitMotorRad / pidKi;
      candidateIntegral = clampFloat(candidateIntegral, -integralLimit, integralLimit);
    }

    float correctionMotorRad = unclampedWithoutIntegral + pidKi * candidateIntegral;
    const float limitedCorrectionMotorRad =
        clampFloat(correctionMotorRad, -pidOutputLimitMotorRad, pidOutputLimitMotorRad);
    const bool saturatedHigh = correctionMotorRad > pidOutputLimitMotorRad && error > 0.0f;
    const bool saturatedLow = correctionMotorRad < -pidOutputLimitMotorRad && error < 0.0f;
    if (!saturatedHigh && !saturatedLow) {
      pidIntegral = candidateIntegral;
    }
    correctionMotorRad = limitedCorrectionMotorRad;
    const int64_t correctionSteps = (int64_t)roundf(correctionMotorRad / MOTOR_RAD_PER_MICROSTEP);
    setTargetStepPosition(baseTargetStepPosition + correctionSteps);

    const int64_t baseDistance = baseTargetStepPosition - readCurrentStepPosition();
    if (abs64(baseDistance) <= 1 && fabsf(error) < 0.002f) {
      resetPidState();
    }
    return;
  }

  setTargetStepPosition(baseTargetStepPosition);
}

static void serviceMotionPlanner() {
  const uint32_t nowUs = micros();
  uint32_t elapsedUs = nowUs - lastPlannerUs;
  if (elapsedUs < PLANNER_PERIOD_US) {
    return;
  }
  uint8_t ticks = (uint8_t)(elapsedUs / PLANNER_PERIOD_US);
  if (ticks > MAX_PLANNER_CATCHUP_TICKS) {
    ticks = MAX_PLANNER_CATCHUP_TICKS;
    lastPlannerUs = nowUs - PLANNER_PERIOD_US;
  } else {
    lastPlannerUs += (uint32_t)ticks * PLANNER_PERIOD_US;
  }

  for (uint8_t tick = 0; tick < ticks; ++tick) {
    const float dt = 1.0f / (float)PLANNER_HZ;
    const uint32_t tickUs = nowUs - (uint32_t)(ticks - 1U - tick) * PLANNER_PERIOD_US;

    if (!driverEnabled || !modeAllowsMotion() || faultFlags != FAULT_NONE) {
      currentSpeedSps = 0.0f;
      const int64_t current = readCurrentStepPosition();
      portENTER_CRITICAL(&motionMux);
      targetStepPosition = current;
      portEXIT_CRITICAL(&motionMux);
      setStepIntervalUs(0);
      baseTargetStepPosition = current;
      chirpActive = false;
      positionTargetValid = false;
      resetPidState();
      serviceCurrentControl(tickUs);
      return;
    }

    serviceMissedStepCorrection();
    if (faultFlags != FAULT_NONE) {
      serviceCurrentControl(tickUs);
      return;
    }
    serviceAdvancedTargets(dt, tickUs);
    if (faultFlags != FAULT_NONE) {
      serviceCurrentControl(tickUs);
      return;
    }
    const int64_t current = readCurrentStepPosition();
    const int64_t target = readTargetStepPosition();
    const int64_t distance = target - current;
    if (distance == 0 && fabsf(currentSpeedSps) < 0.5f) {
      currentSpeedSps = 0.0f;
      setStepIntervalUs(0);
      serviceCurrentControl(tickUs);
      continue;
    }

    const float direction = distance >= 0 ? 1.0f : -1.0f;
    float speedAbs = fabsf(currentSpeedSps);
    if (currentSpeedSps * direction < 0.0f) {
      speedAbs = 0.0f;
    }

    const float safeAccelSps2 = clampFloat(moveAccelSps2, 1.0f, MAX_STEP_RATE_SPS * 20.0f);
    const float safeMaxSpeedSps = clampFloat(maxMoveSpeedSps, 1.0f, MAX_STEP_RATE_SPS);
    const float brakingSteps = (speedAbs * speedAbs) / (2.0f * safeAccelSps2);
    const float desiredSpeed = ((float)abs64(distance) <= brakingSteps + 1.0f) ? 0.0f : safeMaxSpeedSps;

    if (speedAbs < desiredSpeed) {
      speedAbs += safeAccelSps2 * dt;
      if (speedAbs > desiredSpeed) {
        speedAbs = desiredSpeed;
      }
    } else {
      speedAbs -= safeAccelSps2 * dt;
      if (speedAbs < desiredSpeed) {
        speedAbs = desiredSpeed;
      }
    }

    if (speedAbs < 1.0f && distance != 0) {
      speedAbs = 1.0f;
    }
    currentSpeedSps = direction * speedAbs;
    setStepIntervalUs(speedToStepIntervalUs(speedAbs));
    serviceCurrentControl(tickUs);
  }
}

static void sendFrame(uint8_t packetType, uint16_t sequence, const uint8_t *payload, uint16_t payloadLen) {
  uint16_t crc = 0xFFFF;
  uint8_t magic[2] = {MAGIC_0, MAGIC_1};
  uint8_t header[6] = {
      PROTOCOL_VERSION,
      packetType,
      (uint8_t)(sequence & 0xFF),
      (uint8_t)(sequence >> 8),
      (uint8_t)(payloadLen & 0xFF),
      (uint8_t)(payloadLen >> 8),
  };

  for (uint8_t i = 0; i < sizeof(header); ++i) {
    crc = crc16Update(crc, header[i]);
  }
  for (uint16_t i = 0; i < payloadLen; ++i) {
    crc = crc16Update(crc, payload[i]);
  }

  Serial.write(magic, sizeof(magic));
  Serial.write(header, sizeof(header));
  if (payloadLen > 0) {
    Serial.write(payload, payloadLen);
  }
  uint8_t crcBytes[2] = {(uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8)};
  Serial.write(crcBytes, sizeof(crcBytes));
}

static void sendResponse(uint8_t command, ResponseStatus status, const uint8_t *data, uint16_t dataLen, uint16_t sequence) {
  if (dataLen + 2 > MAX_PAYLOAD_SIZE) {
    data = nullptr;
    dataLen = 0;
    status = RESP_ERROR;
  }
  txPayload[0] = command;
  txPayload[1] = (uint8_t)status;
  if (dataLen > 0 && data != nullptr) {
    memcpy(&txPayload[2], data, dataLen);
  }
  sendFrame(PACKET_RESPONSE, sequence, txPayload, dataLen + 2);
}

static void sendResponse(uint8_t command, ResponseStatus status, uint16_t sequence) {
  sendResponse(command, status, nullptr, 0, sequence);
}

static uint16_t appendProtocolString(uint8_t *p, const char *s) {
  const size_t len = strlen(s);
  const uint8_t n = len > 255 ? 255 : (uint8_t)len;
  p[0] = n;
  memcpy(&p[1], s, n);
  return (uint16_t)n + 1;
}

static void sendInfoResponse(uint16_t sequence) {
  uint8_t data[96];
  uint8_t *p = data;
  appendU16(p, 2);
  p += appendProtocolString(p, "xiao_esp32c6_actuator");
  p += appendProtocolString(p, "fw-0.2.0");
  p += appendProtocolString(p, "xiao_esp32c6_tmc2209_as5600x2");
  sendResponse(CMD_INFO, RESP_OK, data, (uint16_t)(p - data), sequence);
}

static void sendConfigResponse(uint16_t sequence) {
  char json[1536];
  const int n = snprintf(
      json,
      sizeof(json),
      "{\"output_per_motor\":%.8g,\"output_offset_rad\":%.8g,"
      "\"pid_enabled\":%s,\"pid_kp\":%.8g,\"pid_ki\":%.8g,\"pid_kd\":%.8g,"
      "\"pid_i_limit_motor_rad\":%.8g,\"pid_output_limit_motor_rad\":%.8g,"
      "\"velocity_pid_kp\":%.8g,\"velocity_pid_ki\":%.8g,"
      "\"velocity_pid_i_limit_motor_rad\":%.8g,"
      "\"torque_proxy_kp\":%.8g,\"torque_proxy_limit_rad\":%.8g,"
      "\"torque_proxy_max_motor_velocity_rad_s\":%.8g,"
      "\"torque_proxy_timeout_s\":%.8g,"
      "\"missed_step_correction_enabled\":%s,"
      "\"missed_step_warn_motor_rad\":%.8g,"
      "\"missed_step_fault_motor_rad\":%.8g,"
      "\"missed_step_correction_rate\":%.8g,"
      "\"current_control_enabled\":%s,\"idle_current_ma\":%u,"
      "\"hold_current_ma\":%u,\"run_current_ma\":%u,"
      "\"current_downshift_delay_s\":%.8g,"
      "\"autotune_max_amplitude_rad\":%.8g,"
      "\"autotune_max_duration_s\":%.8g,"
      "\"autotune_max_deflection_rad\":%.8g,"
      "\"backlash_motor_rad\":%.8g,\"backlash_comp_enabled\":%s,"
      "\"resonance_frequency_hz\":%.8g,\"resonance_derating_enabled\":%s}",
      outputPerMotor,
      outputOffsetRad,
      pidEnabled ? "true" : "false",
      pidKp,
      pidKi,
      pidKd,
      pidILimitMotorRad,
      pidOutputLimitMotorRad,
      velocityPidKp,
      velocityPidKi,
      velocityPidILimitMotorRad,
      torqueProxyKp,
      torqueProxyLimitRad,
      torqueProxyMaxMotorVelocityRadS,
      (float)torqueProxyTimeoutUs * 1.0e-6f,
      missedStepCorrectionEnabled ? "true" : "false",
      missedStepWarnMotorRad,
      missedStepFaultMotorRad,
      missedStepCorrectionRate,
      currentControlEnabled ? "true" : "false",
      idleCurrentMa,
      holdCurrentMa,
      runCurrentMa,
      currentDownshiftDelayS,
      autotuneMaxAmplitudeRad,
      autotuneMaxDurationS,
      autotuneMaxDeflectionRad,
      backlashMotorRad,
      backlashCompEnabled ? "true" : "false",
      resonanceFrequencyHz,
      resonanceDeratingEnabled ? "true" : "false");
  uint16_t len = 0;
  if (n > 0) {
    len = (n >= (int)sizeof(json)) ? (uint16_t)(sizeof(json) - 1) : (uint16_t)n;
  }
  sendResponse(CMD_GET_CONFIG, RESP_OK, reinterpret_cast<const uint8_t *>(json), len, sequence);
}

static void sendFaultsResponse(uint16_t sequence) {
  uint8_t data[4];
  uint8_t *p = data;
  appendU32(p, faultFlags);
  sendResponse(CMD_FAULTS, RESP_OK, data, sizeof(data), sequence);
}

static void sendSelfTestResponse(uint16_t sequence) {
  char result[96];
  if (motorEncoder.ok && outputEncoder.ok && tmcUartOk) {
    strcpy(result, "OK");
  } else if (!tmcUartOk) {
    strcpy(result, "TMC_UART_FAIL");
  } else {
    snprintf(
        result,
        sizeof(result),
        "ENCODER_FAIL motor_ok=%u motor_status=0x%02X output_ok=%u output_status=0x%02X",
        motorEncoder.ok ? 1U : 0U,
        motorEncoder.status,
        outputEncoder.ok ? 1U : 0U,
        outputEncoder.status);
  }
  sendResponse(CMD_SELF_TEST, RESP_OK, reinterpret_cast<const uint8_t *>(result), (uint16_t)strlen(result), sequence);
}

static void sendTelemetry() {
  uint8_t payload[78];
  uint8_t *p = payload;
  telemetrySeq++;
  const int64_t issuedStepPosition = readTargetStepPosition();

  appendU64(p, (uint64_t)micros());
  appendU32(p, telemetrySeq);
  appendFloat(p, (float)issuedStepPosition * MOTOR_RAD_PER_MICROSTEP);
  appendFloat(p, currentSpeedSps * MOTOR_RAD_PER_MICROSTEP);
  appendI32(p, (int32_t)motorEncoder.continuousCount);
  appendI32(p, (int32_t)outputEncoder.continuousCount);
  appendFloat(p, motorEncoder.rad);
  appendFloat(p, outputEncoder.rad);
  appendFloat(p, motorEncoder.velocityRadS);
  appendFloat(p, outputEncoder.velocityRadS);
  appendFloat(p, driverEnabled ? (float)commandedCurrentMa / 1000.0f : 0.0f);
  appendFloat(p, DEFAULT_BUS_VOLTAGE);
  appendFloat(p, DEFAULT_TEMPERATURE_C);
  appendU32(p, faultFlags);
  *p++ = (uint8_t)mode;
  appendFloat(p, positionTargetOutputRad);
  appendFloat(p, torqueProxyRad());
  appendFloat(p, motorSlipRad);
  appendFloat(p, driverEnabled ? (float)commandedCurrentMa / 1000.0f : 0.0f);
  *p++ = controlState;

  sendFrame(PACKET_TELEMETRY, (uint16_t)(telemetrySeq & 0xFFFF), payload, (uint16_t)(p - payload));
}

static void serviceTelemetry() {
  if (!streaming) {
    return;
  }
  const uint32_t periodUs = 1000000UL / TELEMETRY_HZ;
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastTelemetryUs) < periodUs) {
    return;
  }
  lastTelemetryUs = nowUs;
  sendTelemetry();
}

static bool parseSetConfigPayload(const uint8_t *payload, uint16_t len, uint8_t command, uint16_t sequence) {
  JsonDocument doc;
  const DeserializationError error = deserializeJson(doc, payload, len);
  if (error) {
    sendResponse(command, RESP_BAD_PARAM, sequence);
    return false;
  }

  const char *key = doc["key"].as<const char *>();
  JsonVariantConst valueField = doc["value"];
  if (key == nullptr || valueField.isUnbound()) {
    sendResponse(command, RESP_BAD_PARAM, sequence);
    return false;
  }

  bool boolValue = false;
  const bool valueIsBool = valueField.is<bool>();
  if (valueIsBool) {
    boolValue = valueField.as<bool>();
  }
  const bool valueIsNumber = valueField.is<float>() || valueField.is<int>() || valueField.is<long>();
  const float value = valueIsNumber ? valueField.as<float>() : 0.0f;

  if (strcmp(key, "output_per_motor") == 0) {
    if (!valueIsNumber || !isfinite(value) || fabsf(value) < 1.0e-9f) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    outputPerMotor = value;
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "output_offset_rad") == 0) {
    if (!valueIsNumber || !isfinite(value)) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    outputOffsetRad = value;
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "pid_enabled") == 0) {
    if (!valueIsBool) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    pidEnabled = boolValue;
    resetPidState();
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "pid_kp") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    pidKp = sanitizePidGain(value);
    resetPidState();
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "pid_ki") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    pidKi = sanitizePidGain(value);
    resetPidState();
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "pid_kd") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    pidKd = sanitizePidGain(value);
    resetPidState();
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "pid_i_limit_motor_rad") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    pidILimitMotorRad = sanitizePidLimit(fabsf(value), 0.05f);
    resetPidState();
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "pid_output_limit_motor_rad") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    pidOutputLimitMotorRad = sanitizePidLimit(fabsf(value), 0.25f);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "velocity_pid_kp") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    velocityPidKp = sanitizePidGain(value);
    resetPidState();
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "velocity_pid_ki") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    velocityPidKi = sanitizePidGain(value);
    resetPidState();
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "velocity_pid_i_limit_motor_rad") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    velocityPidILimitMotorRad = sanitizePidLimit(fabsf(value), 0.2f);
    resetPidState();
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "torque_proxy_kp") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    torqueProxyKp = sanitizePidGain(value);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "torque_proxy_limit_rad") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    torqueProxyLimitRad = clampFloat(fabsf(value), 0.001f, 10.0f);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "torque_proxy_max_motor_velocity_rad_s") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    torqueProxyMaxMotorVelocityRadS = clampFloat(fabsf(value), 0.01f, MAX_VELOCITY_RAD_S);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "torque_proxy_timeout_s") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    const float seconds = clampFloat(fabsf(value), 0.05f, 120.0f);
    torqueProxyTimeoutUs = (uint32_t)(seconds * 1000000.0f);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "missed_step_correction_enabled") == 0) {
    if (!valueIsBool) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    missedStepCorrectionEnabled = boolValue;
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "missed_step_warn_motor_rad") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    missedStepWarnMotorRad = clampFloat(fabsf(value), 0.0f, missedStepFaultMotorRad);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "missed_step_fault_motor_rad") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    missedStepFaultMotorRad = clampFloat(fabsf(value), 0.001f, 10.0f);
    if (missedStepWarnMotorRad > missedStepFaultMotorRad) {
      missedStepWarnMotorRad = missedStepFaultMotorRad;
    }
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "missed_step_correction_rate") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    missedStepCorrectionRate = clampFloat(value, 0.0f, 1.0f);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "current_control_enabled") == 0) {
    if (!valueIsBool) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    currentControlEnabled = boolValue;
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "idle_current_ma") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    idleCurrentMa = (uint16_t)clampFloat(fabsf(value), 0.0f, 2000.0f);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "hold_current_ma") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    holdCurrentMa = (uint16_t)clampFloat(fabsf(value), 0.0f, (float)runCurrentMa);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "run_current_ma") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    runCurrentMa = (uint16_t)clampFloat(fabsf(value), 1.0f, 2000.0f);
    if (holdCurrentMa > runCurrentMa) {
      holdCurrentMa = runCurrentMa;
    }
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "current_downshift_delay_s") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    currentDownshiftDelayS = clampFloat(fabsf(value), 0.0f, 30.0f);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "autotune_max_amplitude_rad") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    autotuneMaxAmplitudeRad = clampFloat(fabsf(value), 0.001f, 10.0f);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "autotune_max_duration_s") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    autotuneMaxDurationS = clampFloat(fabsf(value), 0.05f, 120.0f);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "autotune_max_deflection_rad") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    autotuneMaxDeflectionRad = clampFloat(fabsf(value), 0.001f, 10.0f);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "backlash_motor_rad") == 0) {
    if (!valueIsNumber) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    backlashMotorRad = clampFloat(fabsf(value), 0.0f, 10.0f);
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "backlash_comp_enabled") == 0) {
    if (!valueIsBool) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    backlashCompEnabled = boolValue;
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "resonance_frequency_hz") == 0) {
    if (valueField.isNull()) {
      resonanceFrequencyHz = 0.0f;
    } else if (valueIsNumber && isfinite(value)) {
      resonanceFrequencyHz = clampFloat(fabsf(value), 0.0f, 100.0f);
    } else {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  if (strcmp(key, "resonance_derating_enabled") == 0) {
    if (!valueIsBool) {
      sendResponse(command, RESP_BAD_PARAM, sequence);
      return false;
    }
    resonanceDeratingEnabled = boolValue;
    sendResponse(command, RESP_OK, sequence);
    return true;
  }

  sendResponse(command, RESP_BAD_PARAM, sequence);
  return false;
}

static ResponseStatus configureOutputPositionTarget(
    float targetOrDeltaOutputRad,
    float velocityOutputRadS,
    float accelOutputRadS2,
    bool relativeTarget) {
  if (faultFlags != FAULT_NONE) {
    return RESP_FAULT;
  }
  if (mode != MODE_POSITION) {
    return RESP_BAD_MODE;
  }
  if (fabsf(outputPerMotor) < 1.0e-9f ||
      !isfinite(targetOrDeltaOutputRad) ||
      !isfinite(velocityOutputRadS) ||
      !isfinite(accelOutputRadS2)) {
    return RESP_BAD_PARAM;
  }

  float targetOutputRad =
      relativeTarget ? outputEncoder.rad + targetOrDeltaOutputRad : targetOrDeltaOutputRad;
  float outputDeltaRad = targetOutputRad - outputEncoder.rad;
  if (relativeTarget) {
    outputDeltaRad = clampFloat(outputDeltaRad, -MAX_MOVE_RAD, MAX_MOVE_RAD);
    targetOutputRad = outputEncoder.rad + outputDeltaRad;
  }
  velocityOutputRadS = clampFloat(fabsf(velocityOutputRadS), 0.01f, MAX_VELOCITY_RAD_S);
  accelOutputRadS2 = clampFloat(fabsf(accelOutputRadS2), 0.01f, MAX_ACCEL_RAD_S2);
  if (resonanceDeratingEnabled && resonanceFrequencyHz > 0.1f && fabsf(outputDeltaRad) > 1.0e-6f) {
    const float minDurationS = 2.0f / resonanceFrequencyHz;
    velocityOutputRadS = fminf(velocityOutputRadS, fmaxf(0.01f, fabsf(outputDeltaRad) / minDurationS));
    accelOutputRadS2 = fminf(accelOutputRadS2, fmaxf(0.01f, velocityOutputRadS * resonanceFrequencyHz * 2.0f));
  }

  const float currentMotorRad = (float)readCurrentStepPosition() * MOTOR_RAD_PER_MICROSTEP;
  float desiredMotorRad = (targetOutputRad - outputOffsetRad) / outputPerMotor;
  const int8_t direction = desiredMotorRad >= currentMotorRad ? 1 : -1;
  if (backlashCompEnabled) {
    if (backlashDirection != 0 && direction != backlashDirection) {
      backlashOffsetMotorRad = direction > 0 ? backlashMotorRad : -backlashMotorRad;
    } else if (backlashDirection == 0) {
      backlashOffsetMotorRad = direction > 0 ? backlashMotorRad : -backlashMotorRad;
    }
    desiredMotorRad += backlashOffsetMotorRad;
  }
  backlashDirection = direction;
  lastPositionDirection = direction;

  chirpActive = false;
  positionTargetValid = true;
  positionTargetOutputRad = targetOutputRad;
  resetPidState();
  setBaseAndTargetStepPosition((int64_t)roundf(desiredMotorRad / MOTOR_RAD_PER_MICROSTEP));
  maxMoveSpeedSps = clampFloat(
      velocityOutputRadS / fabsf(outputPerMotor) / MOTOR_RAD_PER_MICROSTEP,
      1.0f,
      MAX_STEP_RATE_SPS);
  moveAccelSps2 = clampFloat(
      accelOutputRadS2 / fabsf(outputPerMotor) / MOTOR_RAD_PER_MICROSTEP,
      1.0f,
      MAX_STEP_RATE_SPS * 20.0f);
  enableDriver(true);
  return RESP_OK;
}

static void handleMoveRel(const uint8_t *payload, uint16_t len, uint16_t sequence) {
  if (len != 12) {
    sendResponse(CMD_MOVE_REL, RESP_BAD_PARAM, sequence);
    return;
  }
  if (faultFlags != FAULT_NONE) {
    sendResponse(CMD_MOVE_REL, RESP_FAULT, sequence);
    return;
  }
  if (mode != MODE_CALIBRATION) {
    sendResponse(CMD_MOVE_REL, RESP_BAD_MODE, sequence);
    return;
  }

  float deltaRad = readFloatLe(payload);
  float velocityRadS = readFloatLe(payload + 4);
  float accelRadS2 = readFloatLe(payload + 8);

  if (!isfinite(deltaRad) || !isfinite(velocityRadS) || !isfinite(accelRadS2)) {
    sendResponse(CMD_MOVE_REL, RESP_BAD_PARAM, sequence);
    return;
  }

  deltaRad = clampFloat(deltaRad, -MAX_MOVE_RAD, MAX_MOVE_RAD);
  velocityRadS = clampFloat(fabsf(velocityRadS), 0.01f, MAX_VELOCITY_RAD_S);
  accelRadS2 = clampFloat(fabsf(accelRadS2), 0.01f, MAX_ACCEL_RAD_S2);

  const int32_t deltaSteps = (int32_t)roundf(deltaRad / MOTOR_RAD_PER_MICROSTEP);
  const int64_t current = readCurrentStepPosition();
  chirpActive = false;
  positionTargetValid = false;
  resetPidState();
  setBaseAndTargetStepPosition(current + deltaSteps);
  maxMoveSpeedSps = clampFloat(velocityRadS / MOTOR_RAD_PER_MICROSTEP, 1.0f, MAX_STEP_RATE_SPS);
  moveAccelSps2 = clampFloat(accelRadS2 / MOTOR_RAD_PER_MICROSTEP, 1.0f, MAX_STEP_RATE_SPS * 20.0f);
  enableDriver(true);
  sendResponse(CMD_MOVE_REL, RESP_OK, sequence);
}

static void handleMoveOutputRel(const uint8_t *payload, uint16_t len, uint16_t sequence) {
  if (len != 12) {
    sendResponse(CMD_MOVE_OUTPUT_REL, RESP_BAD_PARAM, sequence);
    return;
  }

  float deltaOutputRad = readFloatLe(payload);
  float velocityOutputRadS = readFloatLe(payload + 4);
  float accelOutputRadS2 = readFloatLe(payload + 8);
  const ResponseStatus status =
      configureOutputPositionTarget(deltaOutputRad, velocityOutputRadS, accelOutputRadS2, true);
  sendResponse(CMD_MOVE_OUTPUT_REL, status, sequence);
}

static void handleStartChirp(const uint8_t *payload, uint16_t len, uint16_t sequence) {
  if (len != 20) {
    sendResponse(CMD_START_CHIRP, RESP_BAD_PARAM, sequence);
    return;
  }
  if (faultFlags != FAULT_NONE) {
    sendResponse(CMD_START_CHIRP, RESP_FAULT, sequence);
    return;
  }
  if (mode != MODE_CALIBRATION) {
    sendResponse(CMD_START_CHIRP, RESP_BAD_MODE, sequence);
    return;
  }

  float amplitudeRad = readFloatLe(payload);
  float startHz = readFloatLe(payload + 4);
  float endHz = readFloatLe(payload + 8);
  float durationS = readFloatLe(payload + 12);
  float maxDeflectionRad = readFloatLe(payload + 16);
  if (!isfinite(amplitudeRad) || !isfinite(startHz) || !isfinite(endHz) ||
      !isfinite(durationS) || !isfinite(maxDeflectionRad)) {
    sendResponse(CMD_START_CHIRP, RESP_BAD_PARAM, sequence);
    return;
  }

  chirpAmplitudeRad = clampFloat(fabsf(amplitudeRad), 0.001f, 0.5f);
  chirpStartHz = clampFloat(fabsf(startHz), 0.05f, 70.0f);
  chirpEndHz = clampFloat(fabsf(endHz), chirpStartHz, 70.0f);
  chirpDurationS = clampFloat(fabsf(durationS), 1.0f, 120.0f);
  chirpMaxDeflectionRad = clampFloat(fabsf(maxDeflectionRad), 0.001f, 1.0f);
  chirpCenterMotorRad = (float)readCurrentStepPosition() * MOTOR_RAD_PER_MICROSTEP;
  chirpStartUs = micros();
  chirpActive = true;
  positionTargetValid = false;
  resetPidState();
  setBaseAndTargetStepPosition(readCurrentStepPosition());
  maxMoveSpeedSps = clampFloat(
      chirpAmplitudeRad * TWO_PI_F * chirpEndHz / MOTOR_RAD_PER_MICROSTEP * 1.25f,
      1.0f,
      MAX_STEP_RATE_SPS);
  moveAccelSps2 = clampFloat(maxMoveSpeedSps * 40.0f, 1.0f, MAX_STEP_RATE_SPS * 20.0f);
  enableDriver(true);
  sendResponse(CMD_START_CHIRP, RESP_OK, sequence);
}

static void handleSetPositionTarget(const uint8_t *payload, uint16_t len, uint16_t sequence) {
  if (len != 13) {
    sendResponse(CMD_SET_POSITION_TARGET, RESP_BAD_PARAM, sequence);
    return;
  }
  const float targetOutputRad = readFloatLe(payload);
  const float velocityOutputRadS = readFloatLe(payload + 4);
  const float accelOutputRadS2 = readFloatLe(payload + 8);
  const bool relativeTarget = (payload[12] & 0x01) != 0;
  const ResponseStatus status =
      configureOutputPositionTarget(targetOutputRad, velocityOutputRadS, accelOutputRadS2, relativeTarget);
  sendResponse(CMD_SET_POSITION_TARGET, status, sequence);
}

static void handleSetVelocityTarget(const uint8_t *payload, uint16_t len, uint16_t sequence) {
  if (len != 8) {
    sendResponse(CMD_SET_VELOCITY_TARGET, RESP_BAD_PARAM, sequence);
    return;
  }
  if (faultFlags != FAULT_NONE) {
    sendResponse(CMD_SET_VELOCITY_TARGET, RESP_FAULT, sequence);
    return;
  }
  if (mode != MODE_VELOCITY) {
    sendResponse(CMD_SET_VELOCITY_TARGET, RESP_BAD_MODE, sequence);
    return;
  }
  if (fabsf(outputPerMotor) < 1.0e-9f) {
    sendResponse(CMD_SET_VELOCITY_TARGET, RESP_BAD_PARAM, sequence);
    return;
  }
  float outputVelocityRadS = readFloatLe(payload);
  float accelOutputRadS2 = readFloatLe(payload + 4);
  if (!isfinite(outputVelocityRadS) || !isfinite(accelOutputRadS2)) {
    sendResponse(CMD_SET_VELOCITY_TARGET, RESP_BAD_PARAM, sequence);
    return;
  }
  outputVelocityRadS = clampFloat(outputVelocityRadS, -MAX_VELOCITY_RAD_S, MAX_VELOCITY_RAD_S);
  accelOutputRadS2 = clampFloat(fabsf(accelOutputRadS2), 0.01f, MAX_ACCEL_RAD_S2);
  velocityTargetOutputRadS = outputVelocityRadS;
  chirpActive = false;
  positionTargetValid = false;
  resetPidState();
  baseTargetStepPosition = readCurrentStepPosition();
  setTargetStepPosition(baseTargetStepPosition);
  maxMoveSpeedSps = clampFloat(fabsf(outputVelocityRadS / outputPerMotor) / MOTOR_RAD_PER_MICROSTEP, 1.0f, MAX_STEP_RATE_SPS);
  moveAccelSps2 = clampFloat(accelOutputRadS2 / fabsf(outputPerMotor) / MOTOR_RAD_PER_MICROSTEP, 1.0f, MAX_STEP_RATE_SPS * 20.0f);
  enableDriver(true);
  sendResponse(CMD_SET_VELOCITY_TARGET, RESP_OK, sequence);
}

static void handleSetTorqueProxyTarget(const uint8_t *payload, uint16_t len, uint16_t sequence) {
  if (len != 16) {
    sendResponse(CMD_SET_TORQUE_PROXY_TARGET, RESP_BAD_PARAM, sequence);
    return;
  }
  if (faultFlags != FAULT_NONE) {
    sendResponse(CMD_SET_TORQUE_PROXY_TARGET, RESP_FAULT, sequence);
    return;
  }
  if (mode != MODE_TORQUE_PROXY) {
    sendResponse(CMD_SET_TORQUE_PROXY_TARGET, RESP_BAD_MODE, sequence);
    return;
  }
  const float targetDeflectionRad = readFloatLe(payload);
  const float maxMotorVelocityRadS = readFloatLe(payload + 4);
  const float maxMotorExcursionRad = readFloatLe(payload + 8);
  const float timeoutS = readFloatLe(payload + 12);
  if (!isfinite(targetDeflectionRad) || !isfinite(maxMotorVelocityRadS) ||
      !isfinite(maxMotorExcursionRad) || !isfinite(timeoutS)) {
    sendResponse(CMD_SET_TORQUE_PROXY_TARGET, RESP_BAD_PARAM, sequence);
    return;
  }
  torqueProxyTargetRad = clampFloat(targetDeflectionRad, -torqueProxyLimitRad, torqueProxyLimitRad);
  torqueProxyCommandMaxVelocityRadS =
      clampFloat(fabsf(maxMotorVelocityRadS), 0.01f, torqueProxyMaxMotorVelocityRadS);
  torqueProxyMaxExcursionRad = clampFloat(fabsf(maxMotorExcursionRad), 0.001f, 10.0f);
  torqueProxyTimeoutUs = (uint32_t)(clampFloat(fabsf(timeoutS), 0.05f, 120.0f) * 1000000.0f);
  torqueProxyStartUs = micros();
  torqueProxyStartMotorRad = motorEncoder.rad;
  chirpActive = false;
  positionTargetValid = false;
  resetPidState();
  baseTargetStepPosition = readCurrentStepPosition();
  setTargetStepPosition(baseTargetStepPosition);
  enableDriver(true);
  sendResponse(CMD_SET_TORQUE_PROXY_TARGET, RESP_OK, sequence);
}

static void handleAutotuneControl(const uint8_t *payload, uint16_t len, uint16_t sequence) {
  if (len != 17) {
    sendResponse(CMD_AUTOTUNE_CONTROL, RESP_BAD_PARAM, sequence);
    return;
  }
  if (faultFlags != FAULT_NONE) {
    sendResponse(CMD_AUTOTUNE_CONTROL, RESP_FAULT, sequence);
    return;
  }
  if (mode != MODE_POSITION && mode != MODE_VELOCITY) {
    sendResponse(CMD_AUTOTUNE_CONTROL, RESP_BAD_MODE, sequence);
    return;
  }
  const uint8_t loopSelector = payload[0];
  const float amplitudeRad = readFloatLe(payload + 1);
  const float maxVelocityRadS = readFloatLe(payload + 5);
  const float durationS = readFloatLe(payload + 9);
  const float maxDeflectionRad = readFloatLe(payload + 13);
  if ((loopSelector != AUTOTUNE_VELOCITY &&
       loopSelector != AUTOTUNE_POSITION &&
       loopSelector != AUTOTUNE_BOTH) ||
      !isfinite(amplitudeRad) ||
      !isfinite(maxVelocityRadS) ||
      !isfinite(durationS) ||
      !isfinite(maxDeflectionRad)) {
    sendResponse(CMD_AUTOTUNE_CONTROL, RESP_BAD_PARAM, sequence);
    return;
  }
  autotuneLoopSelector = loopSelector;
  autotuneAmplitudeRad = clampFloat(fabsf(amplitudeRad), 0.001f, autotuneMaxAmplitudeRad);
  autotuneMaxVelocityRadS = clampFloat(fabsf(maxVelocityRadS), 0.01f, MAX_VELOCITY_RAD_S);
  autotuneDurationUs = (uint32_t)(clampFloat(fabsf(durationS), 0.05f, autotuneMaxDurationS) * 1000000.0f);
  autotuneActiveMaxDeflectionRad = clampFloat(fabsf(maxDeflectionRad), 0.001f, autotuneMaxDeflectionRad);
  autotuneStartUs = micros();
  controlState = CONTROL_AUTOTUNE_RUNNING;
  setLastControlFault("");
  enableDriver(true);
  sendResponse(CMD_AUTOTUNE_CONTROL, RESP_OK, sequence);
}

static const char *modeName(ActuatorMode value) {
  switch (value) {
    case MODE_DISABLED: return "DISABLED";
    case MODE_CALIBRATION: return "CALIBRATION";
    case MODE_OPEN_LOOP: return "OPEN_LOOP";
    case MODE_POSITION: return "POSITION";
    case MODE_VELOCITY: return "VELOCITY";
    case MODE_TORQUE_PROXY: return "TORQUE_PROXY";
    case MODE_FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

static void sendControlStatusResponse(uint16_t sequence) {
  char json[512];
  const int n = snprintf(
      json,
      sizeof(json),
      "{\"mode\":%u,\"mode_name\":\"%s\","
      "\"target_motor_rad\":%.8g,\"target_output_rad\":%.8g,"
      "\"velocity_target_output_rad_s\":%.8g,"
      "\"torque_proxy_target_rad\":%.8g,\"torque_proxy_rad\":%.8g,"
      "\"motor_slip_rad\":%.8g,\"commanded_current_a\":%.8g,"
      "\"autotune_state\":%u,\"autotune_loop_selector\":%u,"
      "\"last_control_fault\":\"%s\",\"fault_flags\":%lu}",
      (unsigned)mode,
      modeName(mode),
      (float)readTargetStepPosition() * MOTOR_RAD_PER_MICROSTEP,
      positionTargetOutputRad,
      velocityTargetOutputRadS,
      torqueProxyTargetRad,
      torqueProxyRad(),
      motorSlipRad,
      (float)commandedCurrentMa / 1000.0f,
      (unsigned)controlState,
      (unsigned)autotuneLoopSelector,
      lastControlFault,
      (unsigned long)faultFlags);
  uint16_t len = 0;
  if (n > 0) {
    len = (n >= (int)sizeof(json)) ? (uint16_t)(sizeof(json) - 1) : (uint16_t)n;
  }
  sendResponse(CMD_GET_CONTROL_STATUS, RESP_OK, reinterpret_cast<const uint8_t *>(json), len, sequence);
}

static void handleCommand(uint16_t sequence, const uint8_t *payload, uint16_t len) {
  if (len < 1) {
    faultFlags |= FAULT_PROTOCOL_ERROR;
    return;
  }

  const uint8_t command = payload[0];
  const uint8_t *data = payload + 1;
  const uint16_t dataLen = len - 1;

  switch (command) {
    case CMD_PING: {
      static const uint8_t pong[] = {'P', 'O', 'N', 'G'};
      sendResponse(command, RESP_OK, pong, sizeof(pong), sequence);
      break;
    }

    case CMD_INFO:
      sendInfoResponse(sequence);
      break;

    case CMD_STREAM_ON:
      streaming = true;
      lastTelemetryUs = micros();
      sendResponse(command, RESP_OK, sequence);
      break;

    case CMD_STREAM_OFF:
      streaming = false;
      sendResponse(command, RESP_OK, sequence);
      break;

    case CMD_SET_MODE:
      if (dataLen != 1) {
        sendResponse(command, RESP_BAD_PARAM, sequence);
        break;
      }
      if (data[0] != MODE_DISABLED &&
          data[0] != MODE_CALIBRATION &&
          data[0] != MODE_OPEN_LOOP &&
          data[0] != MODE_POSITION &&
          data[0] != MODE_VELOCITY &&
          data[0] != MODE_TORQUE_PROXY &&
          data[0] != MODE_FAULT) {
        sendResponse(command, RESP_BAD_PARAM, sequence);
        break;
      }
      if (faultFlags != FAULT_NONE && data[0] != MODE_DISABLED) {
        setMode(MODE_FAULT);
        sendResponse(command, RESP_FAULT, sequence);
        break;
      }
      setMode((ActuatorMode)data[0]);
      sendResponse(command, RESP_OK, sequence);
      break;

    case CMD_MOVE_REL:
      handleMoveRel(data, dataLen, sequence);
      break;

    case CMD_START_CHIRP:
      handleStartChirp(data, dataLen, sequence);
      break;

    case CMD_MOVE_OUTPUT_REL:
      handleMoveOutputRel(data, dataLen, sequence);
      break;

    case CMD_SET_POSITION_TARGET:
      handleSetPositionTarget(data, dataLen, sequence);
      break;

    case CMD_SET_VELOCITY_TARGET:
      handleSetVelocityTarget(data, dataLen, sequence);
      break;

    case CMD_SET_TORQUE_PROXY_TARGET:
      handleSetTorqueProxyTarget(data, dataLen, sequence);
      break;

    case CMD_AUTOTUNE_CONTROL:
      handleAutotuneControl(data, dataLen, sequence);
      break;

    case CMD_GET_CONTROL_STATUS:
      sendControlStatusResponse(sequence);
      break;

    case CMD_STOP:
      stopMotion();
      sendResponse(command, RESP_OK, sequence);
      break;

    case CMD_ESTOP:
      stopMotion();
      faultFlags |= FAULT_ESTOP_ACTIVE;
      setMode(MODE_FAULT);
      sendResponse(command, RESP_OK, sequence);
      break;

    case CMD_ZERO_MOTOR_ENCODER:
      serviceEncoders();
      zeroEncoder(motorEncoder);
      motorSlipRad = 0.0f;
      currentSpeedSps = 0.0f;
      resetStepPositions();
      stepReferenceAligned = true;
      sendResponse(command, RESP_OK, sequence);
      break;

    case CMD_ZERO_OUTPUT_ENCODER:
      serviceEncoders();
      zeroEncoder(outputEncoder);
      positionTargetValid = false;
      resetPidState();
      sendResponse(command, RESP_OK, sequence);
      break;

    case CMD_GET_CONFIG:
      sendConfigResponse(sequence);
      break;

    case CMD_SET_CONFIG:
      parseSetConfigPayload(data, dataLen, command, sequence);
      break;

    case CMD_SAVE_CONFIG:
      sendResponse(command, saveConfig() ? RESP_OK : RESP_ERROR, sequence);
      break;

    case CMD_FAULTS:
      sendFaultsResponse(sequence);
      break;

    case CMD_CLEAR_FAULTS:
      faultFlags = FAULT_NONE;
      controlState = CONTROL_IDLE;
      setLastControlFault("");
      if (!stepReferenceAligned) {
        serviceEncoders();
        alignStepReferenceToMotorEncoder();
      }
      if (mode == MODE_FAULT) {
        setMode(MODE_DISABLED);
      }
      sendResponse(command, RESP_OK, sequence);
      break;

    case CMD_SELF_TEST:
      sendSelfTestResponse(sequence);
      break;

    default:
      sendResponse(command, RESP_UNSUPPORTED, sequence);
      break;
  }
}

static void resetParser() {
  rxLen = 0;
  rxExpectedLen = 0;
}

static void handleCompleteFrame() {
  const uint8_t version = rxBuf[2];
  const uint8_t packetType = rxBuf[3];
  const uint16_t sequence = readLe16(&rxBuf[4]);
  const uint16_t payloadLen = readLe16(&rxBuf[6]);
  const uint16_t expectedCrc = readLe16(&rxBuf[8 + payloadLen]);
  const uint16_t actualCrc = crc16CcittFalse(&rxBuf[2], (uint16_t)(6 + payloadLen));

  if (expectedCrc != actualCrc) {
    faultFlags |= FAULT_PROTOCOL_ERROR;
    return;
  }
  if (version != PROTOCOL_VERSION) {
    faultFlags |= FAULT_PROTOCOL_ERROR;
    return;
  }
  if (packetType != PACKET_COMMAND) {
    return;
  }
  handleCommand(sequence, &rxBuf[8], payloadLen);
}

static void feedProtocolByte(uint8_t byte) {
  if (rxLen == 0) {
    if (byte == MAGIC_0) {
      rxBuf[rxLen++] = byte;
    }
    return;
  }

  if (rxLen == 1) {
    if (byte == MAGIC_1) {
      rxBuf[rxLen++] = byte;
    } else if (byte == MAGIC_0) {
      rxBuf[0] = MAGIC_0;
      rxLen = 1;
    } else {
      resetParser();
    }
    return;
  }

  rxBuf[rxLen++] = byte;

  if (rxLen == 8) {
    const uint16_t payloadLen = readLe16(&rxBuf[6]);
    if (payloadLen > MAX_PAYLOAD_SIZE) {
      faultFlags |= FAULT_PROTOCOL_ERROR;
      resetParser();
      return;
    }
    rxExpectedLen = (uint16_t)(FRAME_OVERHEAD_SIZE + payloadLen);
  }

  if (rxExpectedLen > 0 && rxLen >= rxExpectedLen) {
    handleCompleteFrame();
    resetParser();
  }
}

static void serviceProtocol() {
  uint16_t bytesRead = 0;
  while (Serial.available() > 0 && bytesRead < MAX_SERIAL_BYTES_PER_LOOP) {
    feedProtocolByte((uint8_t)Serial.read());
    bytesRead++;
  }
}

void setup() {
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH);
  pinMode(PIN_STEP, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_DIR, LOW);

  Serial.begin(USB_SERIAL_BAUD);

  Wire.begin(PIN_HW_I2C_SDA, PIN_HW_I2C_SCL, HW_I2C_FREQ_HZ);
  swI2c.begin();
  loadConfig();
  configureTmc2209();

  lastPlannerUs = micros();
  lastTelemetryUs = micros();
  lastEncoderUpdateUs = 0;

  serviceEncoders();
  configureStepTimer();
  setMode(MODE_DISABLED);
}

void loop() {
  serviceProtocol();
  serviceEncoders();
  serviceMotionPlanner();
  serviceTelemetry();
}
