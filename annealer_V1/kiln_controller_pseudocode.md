# Kiln Controller V1 - Pseudo Code

## Hardware Configuration
```
INPUTS:
- Thermocouple 1 (K-type, 50-1200°F) on SPI/MAX31855 chip select pin 1
- Thermocouple 2 (K-type, 50-1200°F) on SPI/MAX31855 chip select pin 2
- Door limit switch (digital input, pulled high, LOW = door open)
- Pulse generator/rotary encoder (2 pins: CLK, DT)
- Button array (TBD - mode select, confirm, etc.)

OUTPUTS:
- SSR control (PWM output pin)
- Status LEDs (optional - power, heating, alarm)
- Serial TX/RX (to Raspberry Pi)

STORAGE:
- EEPROM (built-in Arduino)
- SD Card (optional expansion via SPI)
```

## Constants & Configuration
```
// Temperature limits
MAX_TEMP = 1200          // °F - absolute hardware limit
DEFAULT_HOLD_TEMP = 950  // °F - default hold temperature
TEMP_PRECISION = 5.0     // °F - control precision (adjustable)
RECOVERY_TEMP_DROP = 25  // °F - max drop before recovery soak
RECOVERY_SOAK_TIME = 30  // minutes

// Control parameters
PWM_PERIOD = 1000        // milliseconds (1 second)
FAILSAFE_DUTY = 5        // percent - duty cycle if sensors lost
MIN_DUTY = 0             // percent
MAX_DUTY = 100           // percent

// PI Control parameters (tunable)
KP = 2.0                 // Proportional gain
KI = 0.5                 // Integral gain
INTEGRAL_MAX = 100       // Anti-windup limit
INTEGRAL_MIN = -100      // Anti-windup limit

// Timing
TEMP_READ_INTERVAL = 500    // ms - how often to read thermocouples
CONTROL_UPDATE_INTERVAL = 1000  // ms - PI control update rate
STATE_SAVE_INTERVAL = 60000     // ms - 1 minute EEPROM save
SERIAL_REPORT_INTERVAL = 1000   // ms - report to Pi

// Alarm codes
ALARM_NONE = 0
ALARM_OVER_TEMP = 1
ALARM_SENSOR1_FAIL = 2
ALARM_SENSOR2_FAIL = 3
ALARM_BOTH_SENSORS_FAIL = 4
ALARM_EMERGENCY_SHUTDOWN = 5
```

## System States (Enum)
```
enum SystemMode {
    STARTUP,
    HOLD_TEMP,
    DOOR_OPEN,
    ANNEALING,          // Future implementation
    MANUAL_OVERRIDE,    // Future implementation
    EMERGENCY_SHUTDOWN
}
```

## Global Variables
```
// Current state
SystemMode currentMode = STARTUP
SystemMode previousMode = STARTUP  // For returning after door close
float currentTemp = 0.0            // °F - max of two sensors
float setpointTemp = DEFAULT_HOLD_TEMP  // °F
int dutyPct = 0                    // 0-100%
int alarmStatus = ALARM_NONE

// Sensor status
bool sensor1Valid = false
bool sensor2Valid = false
float sensor1Temp = 0.0
float sensor2Temp = 0.0

// Door status
bool doorOpen = false

// PI Control variables
float integralError = 0.0
float lastError = 0.0

// Timing
unsigned long lastTempRead = 0
unsigned long lastControlUpdate = 0
unsigned long lastStateSave = 0
unsigned long lastSerialReport = 0
unsigned long modeStartTime = 0

// Recovery
float lastKnownTemp = 0.0
unsigned long lastKnownTime = 0

// PWM control
unsigned long pwmStartTime = 0
bool pwmState = false
```

## Main Program Structure

### setup()
```
FUNCTION setup():
    // Initialize hardware
    INIT serial communication (115200 baud)
    INIT SPI for thermocouples
    INIT MAX31855 chips
    INIT pin modes (SSR output, door switch input with pullup, encoder pins)
    INIT SD card (if available)

    // Load saved state from EEPROM (if valid)
    IF (EEPROM has valid data):
        LOAD previousMode, setpointTemp, lastKnownTemp, lastKnownTime

        // Check if power outage recovery needed
        currentTemp = READ_SENSORS()
        IF (abs(currentTemp - lastKnownTemp) > RECOVERY_TEMP_DROP):
            PRINT "Power outage detected - temp dropped >25°F"
            // Recovery logic will be handled in annealing mode (future)
            currentMode = EMERGENCY_SHUTDOWN  // Safe default for now
        ELSE:
            currentMode = previousMode
    ELSE:
        currentMode = STARTUP
        setpointTemp = DEFAULT_HOLD_TEMP

    // Initialize timers
    lastTempRead = millis()
    lastControlUpdate = millis()
    lastStateSave = millis()
    lastSerialReport = millis()
    modeStartTime = millis()

    PRINT "Kiln Controller V1 Starting..."
    PRINT "Mode:", currentMode
    PRINT "Setpoint:", setpointTemp, "°F"
END FUNCTION
```

### loop()
```
FUNCTION loop():
    unsigned long now = millis()

    // 1. READ SENSORS (every 500ms)
    IF (now - lastTempRead >= TEMP_READ_INTERVAL):
        READ_TEMPERATURE_SENSORS()
        lastTempRead = now

    // 2. CHECK DOOR STATUS (every loop - critical safety)
    CHECK_DOOR_STATUS()

    // 3. READ USER INPUT (encoder, buttons)
    READ_USER_INPUT()

    // 4. UPDATE CONTROL LOGIC (every 1 second)
    IF (now - lastControlUpdate >= CONTROL_UPDATE_INTERVAL):
        UPDATE_PI_CONTROL()
        lastControlUpdate = now

    // 5. UPDATE PWM OUTPUT (every loop - timing critical)
    UPDATE_PWM_OUTPUT()

    // 6. CHECK ALARMS & SAFETY
    CHECK_SAFETY_CONDITIONS()

    // 7. STATE MACHINE
    EXECUTE_CURRENT_MODE()

    // 8. SAVE STATE (every 1 minute or on setpoint change)
    IF (now - lastStateSave >= STATE_SAVE_INTERVAL):
        SAVE_STATE_TO_EEPROM()
        lastStateSave = now

    // 9. SERIAL COMMUNICATION (every 1 second)
    IF (now - lastSerialReport >= SERIAL_REPORT_INTERVAL):
        SEND_SERIAL_REPORT()
        lastSerialReport = now

    PROCESS_SERIAL_COMMANDS()

END FUNCTION
```

## Core Functions

### 1. Temperature Sensing
```
FUNCTION READ_TEMPERATURE_SENSORS():
    // Read sensor 1
    sensor1Temp = READ_MAX31855(CS_PIN_1)
    sensor1Valid = (sensor1Temp != NAN AND sensor1Temp >= 50 AND sensor1Temp <= 1200)

    // Read sensor 2
    sensor2Temp = READ_MAX31855(CS_PIN_2)
    sensor2Valid = (sensor2Temp != NAN AND sensor2Temp >= 50 AND sensor2Temp <= 1200)

    // Determine current temp - use MAX of valid sensors
    IF (sensor1Valid AND sensor2Valid):
        currentTemp = MAX(sensor1Temp, sensor2Temp)
        alarmStatus = ALARM_NONE  // Clear sensor alarms

    ELSE IF (sensor1Valid AND NOT sensor2Valid):
        currentTemp = sensor1Temp
        alarmStatus = ALARM_SENSOR2_FAIL
        PRINT "WARNING: Sensor 2 failed, using Sensor 1"

    ELSE IF (sensor2Valid AND NOT sensor1Valid):
        currentTemp = sensor2Temp
        alarmStatus = ALARM_SENSOR1_FAIL
        PRINT "WARNING: Sensor 1 failed, using Sensor 2"

    ELSE:  // Both sensors failed
        currentTemp = -999  // Invalid marker
        alarmStatus = ALARM_BOTH_SENSORS_FAIL
        PRINT "CRITICAL: Both sensors failed!"
        // Will trigger emergency failsafe in safety check

    RETURN currentTemp
END FUNCTION
```

### 2. PI Control
```
FUNCTION UPDATE_PI_CONTROL():
    // Skip control if in unsafe mode
    IF (currentMode == DOOR_OPEN OR currentMode == EMERGENCY_SHUTDOWN):
        dutyPct = 0
        integralError = 0  // Reset integral
        RETURN

    // Calculate error
    float error = setpointTemp - currentTemp

    // Proportional term
    float pTerm = KP * error

    // Integral term with anti-windup
    integralError = integralError + (error * (CONTROL_UPDATE_INTERVAL / 1000.0))
    IF (integralError > INTEGRAL_MAX):
        integralError = INTEGRAL_MAX
    ELSE IF (integralError < INTEGRAL_MIN):
        integralError = INTEGRAL_MIN

    float iTerm = KI * integralError

    // Calculate duty cycle (0-100%)
    dutyPct = pTerm + iTerm

    // Clamp to valid range
    IF (dutyPct > MAX_DUTY):
        dutyPct = MAX_DUTY
    ELSE IF (dutyPct < MIN_DUTY):
        dutyPct = MIN_DUTY

    // Debug output
    PRINT "Error:", error, "°F | P:", pTerm, "| I:", iTerm, "| Duty:", dutyPct, "%"

END FUNCTION
```

### 3. PWM Output (Software PWM)
```
FUNCTION UPDATE_PWM_OUTPUT():
    unsigned long now = millis()
    unsigned long elapsedInCycle = (now - pwmStartTime) % PWM_PERIOD
    unsigned long onTime = (PWM_PERIOD * dutyPct) / 100

    // Check for both sensors failed - emergency 5% duty
    IF (alarmStatus == ALARM_BOTH_SENSORS_FAIL):
        onTime = (PWM_PERIOD * FAILSAFE_DUTY) / 100

    // Update PWM state
    IF (elapsedInCycle < onTime):
        IF (NOT pwmState):
            digitalWrite(SSR_PIN, HIGH)
            pwmState = true
    ELSE:
        IF (pwmState):
            digitalWrite(SSR_PIN, LOW)
            pwmState = false

    // Reset cycle timer
    IF (elapsedInCycle < 10):  // Cycle just restarted
        pwmStartTime = now

END FUNCTION
```

### 4. Safety Checks
```
FUNCTION CHECK_SAFETY_CONDITIONS():
    // Over-temperature shutdown
    IF (currentTemp >= MAX_TEMP):
        alarmStatus = ALARM_OVER_TEMP
        currentMode = EMERGENCY_SHUTDOWN
        dutyPct = 0
        PRINT "EMERGENCY: Over-temperature! Shutting down."
        TRIGGER_ALARM()

    // Both sensors failed - use failsafe mode
    IF (alarmStatus == ALARM_BOTH_SENSORS_FAIL):
        PRINT "FAILSAFE MODE: 5% duty cycle"
        // Duty is handled in UPDATE_PWM_OUTPUT()

END FUNCTION
```

### 5. Door Monitoring
```
FUNCTION CHECK_DOOR_STATUS():
    bool currentDoorState = digitalRead(DOOR_SWITCH_PIN)

    // Door opened (LOW signal)
    IF (currentDoorState == LOW AND NOT doorOpen):
        doorOpen = true
        previousMode = currentMode  // Save mode to return to
        currentMode = DOOR_OPEN
        dutyPct = 0  // Immediate power cut
        integralError = 0  // Reset PI
        PRINT "Door opened - heating stopped"

    // Door closed (HIGH signal)
    ELSE IF (currentDoorState == HIGH AND doorOpen):
        doorOpen = false
        currentMode = previousMode  // Auto-resume
        PRINT "Door closed - resuming", previousMode

END FUNCTION
```

### 6. User Input
```
FUNCTION READ_USER_INPUT():
    // Read rotary encoder
    int encoderDelta = READ_ENCODER()

    IF (encoderDelta != 0):
        // Adjust setpoint by 5°F increments
        setpointTemp = setpointTemp + (encoderDelta * TEMP_PRECISION)

        // Clamp to safe range
        IF (setpointTemp > MAX_TEMP - 50):
            setpointTemp = MAX_TEMP - 50
        ELSE IF (setpointTemp < 50):
            setpointTemp = 50

        PRINT "Setpoint adjusted to:", setpointTemp, "°F"

        // Save immediately on setpoint change
        SAVE_STATE_TO_EEPROM()
        lastStateSave = millis()

    // Read buttons (future - mode selection, etc.)
    // TBD

END FUNCTION

FUNCTION READ_ENCODER():
    // Standard rotary encoder reading logic
    // Returns: -1 (CCW), 0 (no change), +1 (CW)
    // Implementation depends on encoder library or direct pin reading
    RETURN 0  // Placeholder
END FUNCTION
```

### 7. State Machine
```
FUNCTION EXECUTE_CURRENT_MODE():
    SWITCH (currentMode):

        CASE STARTUP:
            // Initialization mode
            PRINT "Starting up..."
            PRINT "Current temp:", currentTemp, "°F"
            PRINT "Setpoint:", setpointTemp, "°F"

            // Transition to HOLD_TEMP after 2 seconds
            IF (millis() - modeStartTime > 2000):
                currentMode = HOLD_TEMP
                modeStartTime = millis()
            BREAK

        CASE HOLD_TEMP:
            // Normal operation - maintain setpoint temperature
            // PI control handles this automatically
            // No additional logic needed
            BREAK

        CASE DOOR_OPEN:
            // Heating is already stopped in CHECK_DOOR_STATUS()
            // Just wait for door to close
            BREAK

        CASE ANNEALING:
            // Future implementation
            // Will handle ramp rates, soak times, cooling schedules
            PRINT "Annealing mode not yet implemented"
            BREAK

        CASE MANUAL_OVERRIDE:
            // Future implementation
            // Direct duty cycle control, ignore PI
            PRINT "Manual override not yet implemented"
            BREAK

        CASE EMERGENCY_SHUTDOWN:
            // All heating stopped
            dutyPct = 0
            integralError = 0

            // Wait for manual reset or temperature to drop
            IF (currentTemp < 100):
                PRINT "Temperature safe - ready to restart"
                // Could auto-transition or wait for user
            BREAK

    END SWITCH
END FUNCTION
```

### 8. EEPROM Storage
```
EEPROM Memory Map:
Address 0-1:   Magic number (0xAB, 0xCD) - indicates valid data
Address 2:     currentMode (byte)
Address 3-6:   setpointTemp (float, 4 bytes)
Address 7-10:  lastKnownTemp (float, 4 bytes)
Address 11-14: lastKnownTime (unsigned long, 4 bytes)
Address 15-18: Reserved for future use

FUNCTION SAVE_STATE_TO_EEPROM():
    EEPROM.write(0, 0xAB)  // Magic number
    EEPROM.write(1, 0xCD)
    EEPROM.write(2, currentMode)
    EEPROM.put(3, setpointTemp)
    EEPROM.put(7, currentTemp)
    EEPROM.put(11, millis())

    PRINT "State saved to EEPROM"
END FUNCTION

FUNCTION LOAD_STATE_FROM_EEPROM():
    IF (EEPROM.read(0) == 0xAB AND EEPROM.read(1) == 0xCD):
        previousMode = EEPROM.read(2)
        EEPROM.get(3, setpointTemp)
        EEPROM.get(7, lastKnownTemp)
        EEPROM.get(11, lastKnownTime)
        RETURN true
    ELSE:
        RETURN false  // No valid data
END FUNCTION
```

### 9. Serial Communication
```
FUNCTION SEND_SERIAL_REPORT():
    // JSON format for easy parsing by Raspberry Pi
    PRINT "{"
    PRINT "  \"mode\":", currentMode, ","
    PRINT "  \"temp\":", currentTemp, ","
    PRINT "  \"setpoint\":", setpointTemp, ","
    PRINT "  \"duty\":", dutyPct, ","
    PRINT "  \"sensor1\":", sensor1Temp, ","
    PRINT "  \"sensor2\":", sensor2Temp, ","
    PRINT "  \"sensor1_valid\":", sensor1Valid, ","
    PRINT "  \"sensor2_valid\":", sensor2Valid, ","
    PRINT "  \"door_open\":", doorOpen, ","
    PRINT "  \"alarm\":", alarmStatus
    PRINT "}"
END FUNCTION

FUNCTION PROCESS_SERIAL_COMMANDS():
    IF (Serial.available()):
        String command = Serial.readStringUntil('\n')

        // Parse commands from Raspberry Pi
        IF (command.startsWith("SET_TEMP:")):
            float newSetpoint = parseFloat(command.substring(9))
            setpointTemp = CONSTRAIN(newSetpoint, 50, MAX_TEMP - 50)
            PRINT "Setpoint updated via serial:", setpointTemp

        ELSE IF (command == "GET_STATUS"):
            SEND_SERIAL_REPORT()

        ELSE IF (command == "EMERGENCY_STOP"):
            currentMode = EMERGENCY_SHUTDOWN
            PRINT "Emergency stop activated via serial"

        // Future: commands for annealing programs, etc.

END FUNCTION

FUNCTION TRIGGER_ALARM():
    // Could pulse a buzzer, flash LED, send serial alert
    Serial.println("ALARM!")
    // Add hardware alarm output here
END FUNCTION
```

## Memory Estimation

### Arduino Uno (ATmega328P):
- Flash: 32KB (program storage)
- SRAM: 2KB (variables)
- EEPROM: 1KB (persistent storage)

### Estimated Usage:
- Core program: ~8-12KB flash
- Libraries (MAX31855, EEPROM): ~2-4KB flash
- Global variables: ~200-300 bytes SRAM
- **Verdict: Uno should work for Phase 1 (HOLD_TEMP mode)**

### Arduino Mega (ATmega2560):
- Flash: 256KB
- SRAM: 8KB
- EEPROM: 4KB
- **Verdict: Use Mega if adding SD card logging, complex annealing programs, or display**

## Future Expansion Notes

### Annealing Cycle Implementation:
```
STRUCT AnnealingProgram:
    int numSteps
    float stepTemps[MAX_STEPS]      // Target temps
    float rampRates[MAX_STEPS]      // °F per hour
    int soakTimes[MAX_STEPS]        // minutes

// Example 8-hour program:
// Step 1: Ramp to 960°F at 300°F/hr, soak 120 min
// Step 2: Cool to 800°F at 80°F/hr, soak 60 min
// Step 3: Cool to 400°F at 200°F/hr, soak 0 min
// Step 4: Cool to room temp at natural rate
```

### Recovery Logic for Annealing:
```
IF (power_outage_detected AND in_annealing_mode):
    currentTemp = READ_SENSORS()

    // Find which step we should be at based on current temp
    FOR each step in program:
        IF (currentTemp is within this step's range):
            SET current_step to this step
            SOAK for 30 minutes at currentTemp
            RESUME program from current_step
```

---

## Next Steps:
1. Review and approve pseudo code
2. Implement Arduino C code from pseudo code
3. Test with simulated inputs (potentiometer instead of thermocouple)
4. Integrate real MAX31855 thermocouple amplifiers
5. Tune PI control parameters
6. Add annealing cycle functionality
7. Integrate with Raspberry Pi
