// ============================================================================
// GLASS FURNACE CONTROLLER
// ============================================================================
// Inputs: K-type thermocouple, flame eye, e-stop, foot pedal, start button,
//         mode selector, manual control knob
// Outputs: Gas solenoid valve (NC), blower PWM control
// Control: PI temperature control with anti-windup and minimum fire
// ============================================================================

#include <SPI.h>
#include "Adafruit_MAX31855.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
// Thermocouple (MAX31855) - SPI interface
#define MAXDO   12  // Data Out (MISO)
#define MAXCS   10  // Chip Select
#define MAXCLK  13  // Clock (SCK)

// Digital Inputs (active LOW with pullup, except E_STOP and FLAME_EYE)
#define E_STOP_PIN      2   // Emergency stop (NC button, HIGH when pressed/open)
#define FOOT_PEDAL_PIN  3   // Momentary foot pedal
#define START_BTN_PIN   4   // Start button for lighting and fault reset
#define FLAME_EYE_PIN   5   // Flame detector (HIGH = flame present)
#define MODE_SELECT_PIN 6   // Mode selector (LOW = AUTO, HIGH = MANUAL)

// Analog Inputs
#define MANUAL_KNOB_PIN A0  // Potentiometer for manual control

// Digital Outputs
#define GAS_VALVE_PIN   7   // Gas solenoid (HIGH = open, LOW = closed/safe)
#define BLOWER_PWM_PIN  9   // PWM for blower speed control

// Status LEDs (optional)
#define LED_FAULT_PIN   8   // Red LED for faults
#define LED_STATUS_PIN  11  // Green LED for normal operation

// ============================================================================
// SYSTEM STATES
// ============================================================================
enum SystemState {
  STARTUP,
  AUTO,
  LOW_FIRE,
  MANUAL,
  E_STOP,
  FAULT,
  IGNITION  // Ignition with pre-purge sequence
};

// ============================================================================
// TUNABLE PARAMETERS STRUCT
// ============================================================================
struct FurnaceParams {
  float targetTemp;      // Target temperature in Fahrenheit
  float Kp;              // Proportional gain
  float Ki;              // Integral gain
  int integralMax;       // Anti-windup limit
  int lowFireDuty;       // Low fire duty cycle percentage
  int eStopPurgeDuty;    // E-stop purge duty cycle percentage
  int minFireDuty;       // Minimum fire to prevent blowout
};

// Initialize with default values
FurnaceParams params = {
  2200.0,  // targetTemp (F)
  2.0,     // Kp
  0.1,     // Ki
  50,      // integralMax
  20,      // lowFireDuty
  10,      // eStopPurgeDuty
  20       // minFireDuty
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
// State management
SystemState currentState = STARTUP;
SystemState previousState = STARTUP;

// Thermocouple interface
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// Sensor readings
float currentTemp = 0.0;

// Input states
bool flameDetected = false;
bool eStopPressed = false;
bool footPedalPressed = false;
bool startButtonPressed = false;
int manualKnobValue = 0;
bool modeIsAuto = true;  // true = AUTO, false = MANUAL

// Output states
bool gasValveOpen = false;
int blowerDutyCycle = 0;  // 0-100%

// Control state
float integralSum = 0.0;  // Integral accumulator

// Timing
unsigned long lastControlUpdate = 0;
const unsigned long CONTROL_INTERVAL = 100;  // 100ms for PI control

// Ignition sequence timing
unsigned long ignitionStartTime = 0;
const unsigned long PREPURGE_DURATION = 5000;   // 5 seconds pre-purge
const unsigned long IGNITION_TIMEOUT = 10000;   // 10 seconds after gas opens

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Glass Furnace Controller Starting...");
  Serial.println("Version 1.0");
  
  // Initialize digital input pins
  pinMode(E_STOP_PIN, INPUT);          // NC button, no pullup (external pulldown expected)
  pinMode(FOOT_PEDAL_PIN, INPUT_PULLUP);
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(FLAME_EYE_PIN, INPUT);
  pinMode(MODE_SELECT_PIN, INPUT_PULLUP);
  
  // Initialize output pins
  pinMode(GAS_VALVE_PIN, OUTPUT);
  pinMode(BLOWER_PWM_PIN, OUTPUT);
  pinMode(LED_FAULT_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  
  // Set safe initial state
  setGasValve(false);  // Closed
  setBlowerDuty(0);    // Off
  digitalWrite(LED_FAULT_PIN, LOW);
  digitalWrite(LED_STATUS_PIN, LOW);
  
  // Initialize thermocouple
  Serial.println("Initializing MAX31855 thermocouple...");
  delay(500);  // Wait for MAX31855 to stabilize
  
  // Test thermocouple reading
  float testTemp = thermocouple.readFahrenheit();
  if (isnan(testTemp)) {
    Serial.println("ERROR: Thermocouple not detected!");
    checkThermocoupleErrors();  // Print specific error
    currentState = FAULT;
    previousState = STARTUP;
  } else {
    Serial.print("Thermocouple OK. Current temp: ");
    Serial.print(testTemp);
    Serial.println(" F");
  }
  
  // Print configuration
  Serial.println("\n=== Furnace Configuration ===");
  Serial.print("Target Temp: "); Serial.print(params.targetTemp); Serial.println(" F");
  Serial.print("Kp: "); Serial.println(params.Kp);
  Serial.print("Ki: "); Serial.println(params.Ki);
  Serial.print("Min Fire: "); Serial.print(params.minFireDuty); Serial.println("%");
  Serial.print("Low Fire: "); Serial.print(params.lowFireDuty); Serial.println("%");
  Serial.println("=============================\n");
  
  currentState = STARTUP;
  previousState = STARTUP;
  Serial.println("Setup complete. Waiting for pilot flame or START button...");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Read all inputs
  readInputs();
  
  // CRITICAL SAFETY CHECKS - Highest priority
  if (checkEmergencyStop()) {
    return;  // Safety function handles everything
  }
  
  if (checkFlameFailure()) {
    return;  // Safety function handles everything
  }
  
  if (checkSensorError()) {
    return;  // Safety function handles everything
  }
  
  // State machine
  switch (currentState) {
    case STARTUP:
      handleStartup();
      break;
      
    case IGNITION:
      handleIgnition();
      break;
      
    case AUTO:
      handleAutoMode();
      break;
      
    case LOW_FIRE:
      handleLowFireMode();
      break;
      
    case MANUAL:
      handleManualMode();
      break;
      
    case E_STOP:
      handleEStopMode();
      break;
      
    case FAULT:
      handleFaultMode();
      break;
  }
  
  // Update status LEDs
  updateStatusLEDs();
  
  delay(10);  // Small delay for loop stability
}

// ============================================================================
// INPUT READING
// ============================================================================
void readInputs() {
  // Read temperature in Fahrenheit
  currentTemp = thermocouple.readFahrenheit();
  
  // Read digital inputs
  eStopPressed = digitalRead(E_STOP_PIN);  // NC button: HIGH = pressed/open = fault
  footPedalPressed = !digitalRead(FOOT_PEDAL_PIN);      // Active LOW
  startButtonPressed = !digitalRead(START_BTN_PIN);     // Active LOW
  flameDetected = digitalRead(FLAME_EYE_PIN);           // HIGH = flame present
  
  // Read mode selector
  modeIsAuto = !digitalRead(MODE_SELECT_PIN);  // LOW = AUTO
  
  // Read manual control knob
  manualKnobValue = analogRead(MANUAL_KNOB_PIN);
}

// ============================================================================
// CRITICAL SAFETY FUNCTIONS
// ============================================================================
bool checkEmergencyStop() {
  if (eStopPressed) {
    if (currentState != E_STOP) {
      // E-stop just activated
      Serial.println("*** E-STOP ACTIVATED ***");
      previousState = currentState;
      currentState = E_STOP;
      
      // Immediate safe actions
      setGasValve(false);
      setBlowerDuty(params.eStopPurgeDuty);
      
      // Reset integral
      integralSum = 0.0;
    }
    // Stay in E_STOP and maintain purge while button is pressed
    return true;
  }
  return false;
}

bool checkFlameFailure() {
  // Only check for flame loss if gas should be on
  // Ignore during IGNITION state until gas is actually open
  if (gasValveOpen && !flameDetected && currentState != IGNITION) {
    if (currentState != FAULT && currentState != E_STOP) {
      Serial.println("*** FAULT: FLAME LOST ***");
      previousState = currentState;
      currentState = FAULT;
      
      // Immediate safe actions
      setGasValve(false);
      setBlowerDuty(params.eStopPurgeDuty);
      
      // Reset integral
      integralSum = 0.0;
    }
    return true;
  }
  return false;
}

bool checkSensorError() {
  // Check for thermocouple errors
  if (isnan(currentTemp)) {
    if (currentState != FAULT && currentState != E_STOP) {
      Serial.println("*** FAULT: THERMOCOUPLE ERROR ***");
      checkThermocoupleErrors();  // Print specific fault bits
      previousState = currentState;
      currentState = FAULT;
      
      // Safe state
      setGasValve(false);
      setBlowerDuty(0);
      
      integralSum = 0.0;
    }
    return true;
  }
  
  // Check for unreasonable temperatures (Fahrenheit range)
  if (currentTemp < -50 || currentTemp > 2700) {
    if (currentState != FAULT && currentState != E_STOP) {
      Serial.println("*** FAULT: TEMPERATURE OUT OF RANGE ***");
      Serial.print("Temperature reading: ");
      Serial.print(currentTemp);
      Serial.println(" F");
      previousState = currentState;
      currentState = FAULT;
      
      setGasValve(false);
      setBlowerDuty(0);
      
      integralSum = 0.0;
    }
    return true;
  }
  
  return false;
}

void checkThermocoupleErrors() {
  // Read MAX31855 fault bits for detailed diagnostics
  uint8_t fault = thermocouple.readError();
  
  if (fault) {
    Serial.println("Thermocouple Fault Details:");
    
    if (fault & MAX31855_FAULT_OPEN) {
      Serial.println("  - OPEN CIRCUIT: Thermocouple not connected");
    }
    if (fault & MAX31855_FAULT_SHORT_GND) {
      Serial.println("  - SHORT TO GROUND: Thermocouple shorted to ground");
    }
    if (fault & MAX31855_FAULT_SHORT_VCC) {
      Serial.println("  - SHORT TO VCC: Thermocouple shorted to VCC");
    }
  }
}

// ============================================================================
// STATE HANDLER FUNCTIONS
// ============================================================================
void handleStartup() {
  // Startup sequence - wait for pilot flame or start button
  setGasValve(false);
  setBlowerDuty(0);
  
  // Check if start button pressed to enter ignition mode
  if (startButtonPressed) {
    Serial.println("START button pressed - entering IGNITION mode");
    currentState = IGNITION;
    previousState = STARTUP;
    ignitionStartTime = millis();
    return;
  }
  
  // Check if flame already present (pilot lit manually)
  if (flameDetected) {
    Serial.println("Pilot flame detected, ready for operation");
    
    // Transition to appropriate operating mode
    if (modeIsAuto) {
      previousState = STARTUP;
      currentState = AUTO;
      Serial.println("Entering AUTO mode");
    } else {
      previousState = STARTUP;
      currentState = MANUAL;
      Serial.println("Entering MANUAL mode");
    }
  }
}

void handleIgnition() {
  // Ignition sequence with pre-purge
  // Phase 1 (0-5 sec): Pre-purge with blower, button must be held
  // Phase 2 (5+ sec): Open gas valve, wait for flame
  
  unsigned long elapsedTime = millis() - ignitionStartTime;
  
  // Check if start button is still being held
  if (!startButtonPressed) {
    Serial.println("START button released during ignition - aborting");
    currentState = STARTUP;
    setGasValve(false);
    setBlowerDuty(0);
    return;
  }
  
  // Phase 1: Pre-purge
  if (elapsedTime < PREPURGE_DURATION) {
    setGasValve(false);  // Gas off during pre-purge
    setBlowerDuty(params.lowFireDuty);  // Blower on to purge
    
    // Print progress every second
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      Serial.print("Pre-purge: ");
      Serial.print((PREPURGE_DURATION - elapsedTime) / 1000);
      Serial.println(" seconds remaining (hold START button)");
    }
    return;
  }
  
  // Phase 2: Gas valve opens, wait for flame
  setGasValve(true);  // Open gas valve after pre-purge
  setBlowerDuty(params.lowFireDuty);
  
  // Check for flame establishment
  if (flameDetected) {
    Serial.println("Flame established!");
    
    // Transition to operating mode
    if (modeIsAuto) {
      previousState = IGNITION;
      currentState = AUTO;
      Serial.println("Entering AUTO mode");
    } else {
      previousState = IGNITION;
      currentState = MANUAL;
      Serial.println("Entering MANUAL mode");
    }
    return;
  }
  
  // Check for timeout (total time including pre-purge)
  if (elapsedTime > (PREPURGE_DURATION + IGNITION_TIMEOUT)) {
    Serial.println("IGNITION TIMEOUT - flame not established");
    previousState = IGNITION;
    currentState = FAULT;
    setGasValve(false);
    setBlowerDuty(params.eStopPurgeDuty);  // Purge
    return;
  }
  
  // Display ignition countdown
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    unsigned long gasOpenTime = elapsedTime - PREPURGE_DURATION;
    Serial.print("Ignition: Waiting for flame... ");
    Serial.print((IGNITION_TIMEOUT - gasOpenTime) / 1000);
    Serial.println(" seconds remaining");
  }
}

void handleAutoMode() {
  // PI temperature control mode with minimum fire
  
  // Check for mode transitions
  if (footPedalPressed) {
    previousState = AUTO;
    currentState = LOW_FIRE;
    Serial.println("Foot pedal pressed - entering LOW_FIRE mode");
    return;
  }
  
  if (!modeIsAuto) {
    previousState = AUTO;
    currentState = MANUAL;
    integralSum = 0.0;  // Reset integral
    Serial.println("Mode switch to MANUAL");
    return;
  }
  
  // Run PI control at specified interval
  if (millis() - lastControlUpdate >= CONTROL_INTERVAL) {
    lastControlUpdate = millis();
    
    float controlOutput = calculatePIControl();
    
    // Apply minimum fire constraint to prevent blowout
    if (controlOutput > 5) {  // Minimum threshold to open gas
      setGasValve(true);
      // Clamp to minimum fire when valve is open
      int outputDuty = (int)controlOutput;
      if (outputDuty < params.minFireDuty) {
        outputDuty = params.minFireDuty;
      }
      setBlowerDuty(outputDuty);
    } else {
      setGasValve(false);
      setBlowerDuty(0);
    }
  }
}

void handleLowFireMode() {
  // Fixed low fire output while foot pedal held
  
  setGasValve(true);
  setBlowerDuty(params.lowFireDuty);
  
  // Reset integral to prevent windup
  integralSum = 0.0;
  
  // Check for pedal release
  if (!footPedalPressed) {
    // Return to previous mode
    if (modeIsAuto) {
      previousState = LOW_FIRE;
      currentState = AUTO;
      Serial.println("Foot pedal released - returning to AUTO");
    } else {
      previousState = LOW_FIRE;
      currentState = MANUAL;
      Serial.println("Foot pedal released - returning to MANUAL");
    }
  }
}

void handleManualMode() {
  // User manual control via knob
  
  // Check for mode transitions
  if (footPedalPressed) {
    previousState = MANUAL;
    currentState = LOW_FIRE;
    Serial.println("Foot pedal pressed - entering LOW_FIRE mode");
    return;
  }
  
  if (modeIsAuto) {
    previousState = MANUAL;
    currentState = AUTO;
    Serial.println("Mode switch to AUTO");
    return;
  }
  
  // Map knob value (0-1023) to duty cycle (0-100%)
  int desiredDuty = map(manualKnobValue, 0, 1023, 0, 100);
  
  // Set outputs
  if (desiredDuty > 5) {
    setGasValve(true);
    setBlowerDuty(desiredDuty);
  } else {
    setGasValve(false);
    setBlowerDuty(0);
  }
}

void handleEStopMode() {
  // Emergency stop - maintain purge until button released
  
  setGasValve(false);  // Ensure gas is closed
  setBlowerDuty(params.eStopPurgeDuty);  // Maintain purge
  
  // E-stop released?
  if (!eStopPressed) {
    Serial.println("E-STOP released - entering FAULT state");
    Serial.println("Press START button to return to previous mode");
    currentState = FAULT;
  }
}

void handleFaultMode() {
  // Fault state - stay safe until manual reset via START button
  
  setGasValve(false);
  setBlowerDuty(0);
  
  // Check for reset via START button
  if (startButtonPressed) {
    Serial.println("START button pressed - resetting to previous state");
    Serial.print("Returning to: ");
    
    // Return to previous state before fault
    SystemState returnState = previousState;
    
    // Don't return to fault states
    if (returnState == FAULT || returnState == E_STOP) {
      returnState = STARTUP;
    }
    
    switch(returnState) {
      case AUTO:
        Serial.println("AUTO mode");
        break;
      case MANUAL:
        Serial.println("MANUAL mode");
        break;
      case LOW_FIRE:
        Serial.println("LOW_FIRE mode");
        break;
      default:
        Serial.println("STARTUP mode");
        returnState = STARTUP;
        break;
    }
    
    currentState = returnState;
    integralSum = 0.0;  // Reset integral for fresh start
    
    // Small delay to prevent immediate re-trigger
    delay(500);
  }
}

// ============================================================================
// CONTROL ALGORITHM
// ============================================================================
float calculatePIControl() {
  // PI controller with anti-windup
  
  float error = params.targetTemp - currentTemp;
  
  // Proportional term
  float pTerm = params.Kp * error;
  
  // Integral term with anti-windup
  integralSum = integralSum + (error * (CONTROL_INTERVAL / 1000.0));
  
  // Clamp integral to prevent windup
  if (integralSum > params.integralMax) {
    integralSum = params.integralMax;
  } else if (integralSum < -params.integralMax) {
    integralSum = -params.integralMax;
  }
  
  float iTerm = params.Ki * integralSum;
  
  // Calculate output
  float output = pTerm + iTerm;
  
  // Constrain to valid range (0-100%)
  if (output < 0) output = 0;
  if (output > 100) output = 100;
  
  // Debug output every 2 seconds to avoid spam
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    Serial.print("Temp: ");
    Serial.print(currentTemp);
    Serial.print(" F | Target: ");
    Serial.print(params.targetTemp);
    Serial.print(" F | Error: ");
    Serial.print(error);
    Serial.print(" | P: ");
    Serial.print(pTerm);
    Serial.print(" | I: ");
    Serial.print(iTerm);
    Serial.print(" | Output: ");
    Serial.print(output);
    Serial.println("%");
  }
  
  return output;
}

// ============================================================================
// OUTPUT CONTROL FUNCTIONS
// ============================================================================
void setGasValve(bool open) {
  // NC valve: HIGH = open, LOW = closed (safe default)
  if (open) {
    digitalWrite(GAS_VALVE_PIN, HIGH);
    gasValveOpen = true;
  } else {
    digitalWrite(GAS_VALVE_PIN, LOW);
    gasValveOpen = false;
  }
}

void setBlowerDuty(int dutyCycle) {
  // dutyCycle is 0-100%
  // Constrain to valid range
  dutyCycle = constrain(dutyCycle, 0, 100);
  
  // Map to PWM value (0-255 for Arduino)
  int pwmValue = map(dutyCycle, 0, 100, 0, 255);
  analogWrite(BLOWER_PWM_PIN, pwmValue);
  
  blowerDutyCycle = dutyCycle;
}

void updateStatusLEDs() {
  // Update status LEDs based on current state
  
  switch (currentState) {
    case FAULT:
    case E_STOP:
      // Fault LED on, status LED off
      digitalWrite(LED_FAULT_PIN, HIGH);
      digitalWrite(LED_STATUS_PIN, LOW);
      break;
      
    case STARTUP:
    case IGNITION:
      // Flash status LED
      digitalWrite(LED_FAULT_PIN, LOW);
      digitalWrite(LED_STATUS_PIN, (millis() / 500) % 2);  // Flash every 500ms
      break;
      
    default:
      // Normal operation - status LED on, fault LED off
      digitalWrite(LED_FAULT_PIN, LOW);
      digitalWrite(LED_STATUS_PIN, HIGH);
      break;
  }
}