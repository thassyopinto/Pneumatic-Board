/*
Pneumatic Platform v2.0
Revision: August 2020
Author: Thassyo Pinto
*/

#include <PID_v1.h>

// Pressure Sensor Parameters
const int PS_PIN = 9; // Pressure Sensor Pin Number (0-15)
const int PS_CP = 1; // Clock Prescaler (1, 8, 64, 256, 1024) [Timer 1]
const double PS_SF = 1000; // Sampling Frequency (Hz)

// System Parameters
const double SYS_DF = 10; // System Discretization Scaling Factor
const double SYS_DT = (1 / PS_SF) * SYS_DF; // System Delta Time (seconds)

// First-Order Low-Pass Filter Parameters
const double LPF_COF_H = 0.75; // Cut-off Frequency (Hz)
const double LPF_COF_R = 2 * PI * LPF_COF_H; // Cut-off Frequency (rad/s)
const double LPF_ST = 1 / PS_SF; // Sampling Time (seconds)

// Pulse Width Modulation (PWM) Parameters
const int PWM_PIN = 6; // PWM Pin Number (6 = OC4A, 7 = OC4B, 8 = OC4C)
const int PWM_CP = 8; // Clock Prescaler (1, 8, 64, 256, 1024) [Timer 4]
const double PWM_F = 50; // Wave Frequency (Hz)
const double PWM_DC = 0; // Duty Cycle (%)

// Sinusoidal Wave Input Parameters
const double SW_A = 3; // Wave Amplitude (psi) [Half Max Pressure]
const double SW_F = 0.25; // Wave Frequency (Hz)
const double SW_P = 0; // Wave Phase
const double SW_O = SW_A; // Wave Offset (psi)

// Ramp Input Parameters
const double RP_A = 6; // Ramp Top (psi)
const double RP_B = 0; // Ramp Bottom (psi)
const double RP_P = 2; // Ramp Period (seconds)
const double RP_I = (RP_A - RP_B) / ((RP_P / 2) / SYS_DT); // Ramp Increment (psi)

// Proportional-Integral-Derivative (PID) Controller Parameters
// Sample-1: 0.25Hz / 0-40psi / Kp = 10 / Ki = 30 / Kd = 0.01 (sine)
// Sample-2: 0.50Hz / 0-6psi / Kp = 10 / Ki = 5 / Kd = 0.01 (ramp)
// Sample-3: 0.50Hz / 0-6psi / Kp = 10 / Ki = 30 / Kd = 0.01 (sine)
// Sample-4: 10psi / Kp = 2 / Ki = 30 / Kd = 0.01 (const)
const double PID_KP = 5.0; // PID Kp Gain = 50
const double PID_KI = 50.0; // PID Ki Gain = 5
const double PID_KD = 0.01; // PID Kd Gain = 0.1
const double PID_MIN = 0; // PID Minimum Output Value
const double PID_MAX = 100; // PID Maximum Output Value
const double PID_SP = 6; // PID Setpoint Value
const double PID_ST = 5; // PID Sampling Time (ms)

// System Variables
bool updateSystem = false;
int scaleSystem = 0, rampSign = 1;
double voltageValue, pressureValue, filteredPressureValue, scaledFilteredPressureValue, dutyCycleValue, timeValue, sineWave, rampValue;

// 1. Sine Input
PID pressurePID(&scaledFilteredPressureValue, &dutyCycleValue, &sineWave, PID_KP, PID_KI, PID_KD, DIRECT);

// 2. Ramp Input
// PID pressurePID(&scaledFilteredPressureValue, &dutyCycleValue, &rampValue, PID_KP, PID_KI, PID_KD, DIRECT);

// 3. Constant Input
// PID pressurePID(&scaledFilteredPressureValue, &dutyCycleValue, &PID_SP, PID_KP, PID_KI, PID_KD, DIRECT);

void setup() {
  // Serial Communication Setup
  Serial.begin(1000000);

  cli(); // Disable Global Interrupts

  pinMode(5, OUTPUT); // Set PWM Output Pin
  pinMode(6, OUTPUT); // Set PWM Output Pin
  pinMode(7, OUTPUT); // Set PWM Output Pin
  pinMode(8, OUTPUT); // Set PWM Output Pin

  // Pressure Sensor Setup
  TCCR1A = 0; // Reset Timer/Counter Control Register - Channel A
  TCCR1B = 0; // Reset Timer/Counter Control Register - Channel B
  TCCR1B |= (1 << WGM12); // Enable CTC Mode
  TCCR1B |= (1 << CS10); // Set Clock Prescaler = 1
  TIMSK1 |= (1 << OCIE1A); // Enable Timer Interrupt
  OCR1A = (F_CPU / (PS_CP * PS_SF)) - 1; // Set Output Compare Register

  // Pulse Width Modulation (PWM) Setup
  TCCR4A = 0; // Reset Timer/Counter Control Register - Channel A
  TCCR4B = 0; // Reset Timer/Counter Control Register - Channel B
  // ----- Set Clock Prescaler -----
  //TCCR4B |= (1 << CS40); // Set Clock Prescaler = 1
  TCCR4B |= (1 << CS41); // Set Clock Prescaler = 8
  //TCCR4B |= (1 << CS41) | (1 << CS40); // Set Clock Prescaler = 64
  //TCCR4B |= (1 << CS42); // Set Clock Prescaler = 256
  //TCCR4B |= (1 << CS42) | (1 << CS40); // Set Clock Prescaler = 1024
  // ----- Set Non-Inverting / Inverting Mode -----
  TCCR4A |= (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1); // Set Non-inverting PWM Mode
  //TCCR4A |= (1 << COM4C1) | (1 << COM4C0); // Set Inverting PWM Mode
  // ----- Set Fast / Phase and Frequency Correct PWM Mode ------
  //TCCR4B |= (1 << WGM43); // PFC-PWM (Mode 8, TOP = ICRn)
  //ICR4 = F_CPU / (2 * PWM_CP * PWM_F); // PFC-PWM (Input Capture Register)
  TCCR4A |= (1 << WGM41); // Fast-PWM (Mode 14, TOP = ICRn) - Part I
  TCCR4B |= (1 << WGM43) | (1 << WGM42); // Fast-PWM (Mode 14, TOP = ICRn) - Part II
  ICR4 = F_CPU / (PWM_CP * PWM_F) - 1; // Fast-PWM (Input Capture Register)
  OCR4A = ICR4 * (PWM_DC / 100.0); // Set Output Compare Register A
  OCR4B = ICR4 * (PWM_DC / 100.0); // Set Output Compare Register B
  OCR4C = ICR4 * (PWM_DC / 100.0); // Set Output Compare Register C

  // PID Controller Setup
  pressurePID.SetMode(AUTOMATIC); // Turn On the PID Controller
  pressurePID.SetOutputLimits(PID_MIN, PID_MAX); // Set PID Controller Output Limits
  pressurePID.SetSampleTime(PID_ST); // Set PID Controller Sample Time (ms)

  sei(); // Enable Global Interrupts
}

void loop() {
  if (updateSystem) {
    updateSystem = false;
    doInput();
    doPID();
    doPWM();
    doPlot();
  }
}

void doInput() {
  timeValue = timeValue + SYS_DT; // Update Time Value
  doWave();
  //doRamp();
}

void doWave() {
  sineWave = SW_A * sin(2 * PI * SW_F * (timeValue) + SW_P) + SW_O; // Update Sinusoidal Wave
}

void doRamp() {
  rampValue = rampValue + (RP_I * rampSign); // Update Ramp Signal
  if (rampValue >= RP_A) {
    rampSign = -1;
  }
  else if (rampValue <= RP_B) {
    rampSign = 1;
  }
}

void doPID() {
  scaledFilteredPressureValue = filteredPressureValue; // Update PID Input Value (Pressure [psi])
  pressurePID.Compute(); // Compute PID Output Value (PWM Duty Cycle [%])
}

void doPWM() {
  OCR4A = ICR4 * (dutyCycleValue / 100.0); // Update Output Compare Register (PWM Duty Cycle [bits])
  //OCR4B = ICR4 * (dutyCycleValue / 100.0); // Update Output Compare Register (PWM Duty Cycle [bits])
  //OCR4C = ICR4 * (dutyCycleValue / 100.0); // Update Output Compare Register (PWM Duty Cycle [bits])
}

void doPlot() {
  //  Serial.print(timeValue); // Current Time
  //  Serial.print(" ");
  Serial.print(sineWave); // 1. Sine Input
  //  Serial.print(rampValue); // 2. Ramp Input
  //  Serial.print(PID_SP); // 3. Constant Input
  Serial.print(" ");
  Serial.println(scaledFilteredPressureValue); // Current Pressure
}

void readSensor() {
  voltageValue = analogRead(PS_PIN) * (5.0 / 1023.0); // Digital to Analog Conversion (Volts)
  pressureValue = (((voltageValue / 5.0) - 0.1) * (100.0 / 0.8)); // Voltage to Pressure Conversion (psi)
  filteredPressureValue = exp(-LPF_COF_R * LPF_ST) * filteredPressureValue + (1 - exp(-LPF_COF_R * LPF_ST)) * pressureValue; // Discretized First-Order Low-Pass Filter
}

ISR(TIMER1_COMPA_vect) {
  readSensor();
  scaleSystem++;
  if (scaleSystem >= SYS_DF) {
    scaleSystem = 0;
    updateSystem = true;
  }
}
