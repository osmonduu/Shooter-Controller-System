/* Arcade buttons */
#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>
#include <Keyboard.h>
/* Gyromouse */
#include <Adafruit_LSM6DSO32.h>
#include <Mouse.h>

/* Include watchdog library so we can force clear the interrupts */
#include <avr/wdt.h>

//////////////////////////////////////////////////////////////
///                                                        ///
///                     Arcade Buttons                     ///
///                                                        ///
//////////////////////////////////////////////////////////////
#define FIRST_I2C_ADDR 0x3A
#define SECOND_I2C_ADDR 0x3B

// Define switch and LED pins on the breakout board (applies to both)
#define SWITCH1  18  // PA01
#define SWITCH2  19  // PA02
#define SWITCH3  20  // PA03
#define SWITCH4  2   // PA06
#define PWM1  12     // PC00 (LED)
#define PWM2  13     // PC01 (LED)
#define PWM3  0      // PA04 (LED)
#define PWM4  1      // PA05 (LED)

// Subclass Seesaw library
class MySeesaw : public Adafruit_seesaw {
public:
  using Adafruit_seesaw::read;   // bring the protected read() into public scope
};

// Declare breakout board instances
MySeesaw ss1, ss2;

// Breakout board interrupt pins
#define BREAKOUT1_INT_PIN 7
#define BREAKOUT2_INT_PIN 1

// Define pin bit mask used to select which pins on breakout board to read for interrupts
uint32_t mask = ((uint32_t)1 << SWITCH1) | ((uint32_t)1 << SWITCH2) | ((uint32_t)1 << SWITCH3) | ((uint32_t)1 << SWITCH4);

// Flags for seesaw boards ISR
volatile bool seesawInterruptFlag1 = false;
volatile bool seesawInterruptFlag2 = false;
/*----------------------------------------------------------*/

//////////////////////////////////////////////////////////////
///                                                        ///
///                     Gyromouse                          ///
///                                                        ///
//////////////////////////////////////////////////////////////
// Define the primary and secondary fire pins
#define MOUSE1_PIN 4
#define MOUSE2_PIN 5

// Define the recalibration pin
#define RECALIBRATION_PIN 0

// Create sensor instance
Adafruit_LSM6DSO32 dso32;

// Scale factor to convert angular velocity (rad/s) to mouse movement
const float mouseFactor = 100.0;

// Dynamic deadband filter - ignores values (changes) smaller than this (after scaling) 
const float baseDeadband = 0.1;     // Minimum threshold
const float maxDeadband  = 0.8;     // Maximum threshold
const float maxRawExpected = 250;   // Maximum possible reading (after scaling)

// Sensitivity for the mouse movement (applied at the end) - default sensitivity
float sensitivity = 0.25;

// Dynamic scaling (not used right now so kept at default 1.0 values)
// For low speeds, we use a higher scale (for finer control) and at high speeds a lower scale (to reduce overshoot)
const float lowSpeedScale = 1.0;   // Scale factor when the sensor reading is low
const float highSpeedScale = 1.0;  // Scale factor when the sensor reading is high
/*----------------------------------------------------------*/


void setup() {
  Serial.begin(115200);
  // while (!Serial) delay(10);  // <-!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! REMOVE IN FINAL IMPLEMENTATION 
  //////////////////////////////////////////////////////////////
  ///                                                        ///
  ///                     Arcade Buttons                     ///
  ///                                                        ///
  //////////////////////////////////////////////////////////////
  Keyboard.begin(); // Initialize Keyboard
  
  Serial.println(F("Adafruit PID 5296 I2C QT 4x LED Arcade Buttons test!"));

  // Initialize Seesaw devices
  if (!ss1.begin(FIRST_I2C_ADDR)) {
    Serial.println(F("Seesaw 1 not found!"));
    while (1) delay(10);
  }
  if (!ss2.begin(SECOND_I2C_ADDR)) {
    Serial.println(F("Seesaw 2 not found!"));
    while (1) delay(10);
  }

  Serial.println(F("Seesaw started OK!"));

  // Initialize button input pins
  ss1.pinModeBulk(mask, INPUT_PULLUP);
  ss2.pinModeBulk(mask, INPUT_PULLUP);

  // Turn off all LEDs initially
  setVariableLEDs(ss1, 0);
  setVariableLEDs(ss2, 0);

  // Enable GPIO interrupts on seesaw boards using pin mask
  ss1.setGPIOInterrupts(mask, 1);
  ss2.setGPIOInterrupts(mask, 1);
  
  // Attach seesaw boards interrupts to Leonardo's hardware interrupt
  pinMode(BREAKOUT1_INT_PIN, INPUT_PULLUP);
  pinMode(BREAKOUT2_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BREAKOUT1_INT_PIN), seesawISR1, FALLING); 
  attachInterrupt(digitalPinToInterrupt(BREAKOUT2_INT_PIN), seesawISR2, FALLING);
  /*----------------------------------------------------------*/

  //////////////////////////////////////////////////////////////
  ///                                                        ///
  ///                     Gyromouse                          ///
  ///                                                        ///
  //////////////////////////////////////////////////////////////
  // Start USB mouse functionality
  Mouse.begin();

  Serial.println("Adafruit LSM6DSO32 Mouse Begin");

  // Initialize sensor using I2C
  if (!dso32.begin_I2C()) {
    Serial.println("Failed to find LSM6DSO32 chip!");
    while(1) delay(10);
  }
  Serial.println("LSM6DSO32 found!");

  // Set ranges and data rates for the gyroscope
  dso32.setAccelDataRate(LSM6DS_RATE_SHUTDOWN);   // shutdown the accelerometer since we are not using it
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
  // dso32.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
  dso32.setGyroDataRate(LSM6DS_RATE_833_HZ);

  // Init the pins for the primary and secondary fire microswitches and the recalibration microswitch
  pinMode(MOUSE1_PIN, INPUT_PULLUP);
  pinMode(MOUSE2_PIN, INPUT_PULLUP);
  pinMode(RECALIBRATION_PIN, INPUT_PULLUP);
  /*----------------------------------------------------------*/
}

void seesawISR1() {
  seesawInterruptFlag1 = true;
}
void seesawISR2() {
  seesawInterruptFlag2 = true;
}

// Global variables to track previous mouse button states
bool prevMouse1State = HIGH;
bool prevMouse2State = HIGH;

// Used to reset the controller whenever the arcade buttons get stuck. This bypasses the need to unplug and replug the controller.
void reboot() {
  // Force reset the arduino using watchdog timer
  wdt_disable();
  wdt_enable(WDTO_15MS);
  // Software reset the seesaw boards to clear the GPIO interrupts
  bool ss1Reset = ss1.SWReset();
  bool ss2Reset = ss2.SWReset();
  // Read GPIO INTFLAG to read and clear the remaining interrupts from both seesaw boards
  uint8_t byte_buf[4];
  ss1.read(SEESAW_GPIO_BASE, SEESAW_GPIO_INTFLAG, byte_buf, (uint8_t)4, 0);
  ss2.read(SEESAW_GPIO_BASE, SEESAW_GPIO_INTFLAG, byte_buf, (uint8_t)4, 0);
  setup();
  // while (1) {}
}

void loop() {
  //////////////////////////////////////////////////////////////
  ///                                                        ///
  ///                     Arcade Buttons                     ///
  ///                                                        ///
  //////////////////////////////////////////////////////////////
  if (seesawInterruptFlag1) {
    checkButtons(ss1, BREAKOUT1_INT_PIN, true);
    seesawInterruptFlag1 = false;
  }
  if (seesawInterruptFlag2) {
    checkButtons(ss2, BREAKOUT2_INT_PIN, false);
    seesawInterruptFlag2 = false;
  }
  /*----------------------------------------------------------*/

  //////////////////////////////////////////////////////////////
  ///                                                        ///
  ///                     Gyromouse                          ///
  ///                                                        ///
  //////////////////////////////////////////////////////////////
  bool currMouse1State = digitalRead(MOUSE1_PIN);
  bool currMouse2State = digitalRead(MOUSE2_PIN);


  // Mouse cursor movement logic
  if (digitalRead(RECALIBRATION_PIN) == LOW) {
    // Press the reclibration, mouse1, and mouse2 buttons at the same time to reset the gun. Useful for getting the arcade buttons to become unstuck.
    if (currMouse1State == LOW && currMouse2State == LOW) {
      reboot();
    }
    // Don't update mouse movement to stop mouse cursor from moving while recalibration button is held down.
    // Only change sensitivity on a transition from HIGH -> LOW
    if (currMouse1State == LOW && prevMouse1State == HIGH) {
      sensitivity += 0.05;
      // Clamp to prevent too high sens
      if (sensitivity > 3.0) sensitivity = 3.0;
      delay(5);
      // Debug
      // Serial.print("Sensitivity increased: "); Serial.println(sensitivity);
    }
    // Only change sensitivity on a transition from HIGH -> LOW
    if (currMouse2State == LOW && prevMouse2State == HIGH) {
      sensitivity -= 0.05;
      // Clamp to prevent negative sens
      if (sensitivity < 0.05) sensitivity = 0.05;
      delay(5);
      // Debug
      // Serial.print("Sensitivity decreased: "); Serial.println(sensitivity);
    }

    // Disable mouse buttons by forcibly releasing them
    Mouse.release(MOUSE_LEFT);
    Mouse.release(MOUSE_RIGHT);
  } else {
      // Process mouse movement
      updateMouseMovement();

      // Check mouse button microswitches
      if (currMouse1State == LOW) {
        Mouse.press(MOUSE_LEFT);
      } else {
          Mouse.release(MOUSE_LEFT);
      }
      if (currMouse2State == LOW) {
          Mouse.press(MOUSE_RIGHT);
      } else {
          Mouse.release(MOUSE_RIGHT);
      }
  }


  // Store the current states for next iteration
  prevMouse1State = currMouse1State;
  prevMouse2State = currMouse2State;
  /*----------------------------------------------------------*/

  delay(0); // lower delay for the smoother mouse movement
}

void updateMouseMovement() {
  // Get new normalized sensor events
  sensors_event_t accel, gyro, temp;
  dso32.getEvent(&accel, &gyro, &temp);

  // Read gyroscope values (angular velocity in rad/s) and scale
  int rawX = gyro.gyro.z * mouseFactor;
  int rawY = gyro.gyro.y * mouseFactor;

  // Calculate dynamic deadband for each axis based on current speed
  float dynDeadbandX = baseDeadband + (maxDeadband - baseDeadband) * (min(abs(rawX), maxRawExpected) / maxRawExpected);
  float dynDeadbandY = baseDeadband + (maxDeadband - baseDeadband) * (min(abs(rawY), maxRawExpected) / maxRawExpected);

  // Calculate dynamic scaling factors
  float speedFactorX = min(abs(rawX), maxRawExpected) / maxRawExpected;
  float dynamicScaleX = lowSpeedScale + (highSpeedScale - lowSpeedScale) * speedFactorX;
  
  float speedFactorY = min(abs(rawY), maxRawExpected) / maxRawExpected;
  float dynamicScaleY = lowSpeedScale + (highSpeedScale - lowSpeedScale) * speedFactorY;

  // Apply the dynamic deadband filtering and scaling - if the absolute value is less than the dynamic threshold set to 0. Otherwise, scale.
  int xMovement = (abs(rawX) > dynDeadbandX) ? int(rawX * dynamicScaleX) : 0;
  int yMovement = (abs(rawY) > dynDeadbandY) ? int(rawY * dynamicScaleY) : 0;

  // Invert axis to correct mouse orientation - adjust depending on how you hold the device
  if (xMovement != 0) xMovement = -xMovement;
  // if (yMovement != 0) yMovement = -yMovement;

  // Move the mouse based off the gyroscope readings, no mouse wheel
  Mouse.move(xMovement*sensitivity, yMovement*sensitivity, 0);

  // Debug output
  // Serial.print("Raw Gyro X: "); Serial.print(gyro.gyro.z);
  // Serial.print(" | Raw Gyro Y: "); Serial.print(gyro.gyro.y);
  // Serial.print(" | DynDeadbandX: "); Serial.print(dynDeadbandX);
  // Serial.print(" | DynDeadbandY: "); Serial.print(dynDeadbandY);
  // Serial.print(" | DynamicScaleX: "); Serial.print(dynamicScaleX);
  // Serial.print(" | DynamicScaleY: "); Serial.print(dynamicScaleY);
  // Serial.print(" | MouseX: "); Serial.print(xMovement * sensitivity);
  // Serial.print(" | MouseY: "); Serial.print(yMovement * sensitivity);
  // Serial.print("\n");
}

// Used as a debounce to fix single button spam 'sticking' and freezing entire breakout board
bool lastSwitch1 = false, lastSwitch2 = false, lastSwitch3 = false, lastSwitch4 = false;


void checkButtons(MySeesaw &ss, int intPin, bool isFirst) {
  // digitalReadBulk will return mask if no button is pressed. If a button is pressed, its bit position will be 0.
  uint32_t buttonStates = ss.digitalReadBulk(mask);
  
  // Serial.print("Entered checkButtons(), Returned Mask: "); Serial.print(buttonStates); Serial.print("\n");

  bool switch1 = !(buttonStates & (1UL << SWITCH1));
  bool switch2 = !(buttonStates & (1UL << SWITCH2));
  bool switch3 = !(buttonStates & (1UL << SWITCH3));
  bool switch4 = !(buttonStates & (1UL << SWITCH4));

  // Mapping button pressed to keyboard inputs
  if (switch1 != lastSwitch1) {
    if (switch1) {
      Keyboard.press(isFirst ? 'q' : '4');
    } else {
      Keyboard.release(isFirst ? 'q' : '4');
    }
    lastSwitch1 = switch1;
  }

  if (switch2 != lastSwitch2) {
    if (switch1 && switch2) {
      // Keyboard.press(KEY_UP_ARROW);
      // Keyboard.release(KEY_UP_ARROW);
      Mouse.move(0,0,1);
    } 
    else {
        if (switch2) {
          Keyboard.press(isFirst ? '1' : 'f');
        } else {
          Keyboard.release(isFirst ? '1' : 'f');
        }
    }
    lastSwitch1 = switch1;
    lastSwitch2 = switch2;
  }

  if (switch3 != lastSwitch3) {
    if (switch1 && switch3) {
      // Keyboard.press(KEY_DOWN_ARROW);
      // Keyboard.release(KEY_DOWN_ARROW);
      Mouse.move(0,0,-1);
    }
    else if (switch3 && switch4) {
      Keyboard.press(' ');
      Keyboard.release(' ');
      delay(15);
      Keyboard.press(' ');
      Keyboard.release(' ');
    }
    else {
        if (switch3) {
          Keyboard.press(isFirst ? '2' : 'r');
        } else {
          Keyboard.release(isFirst ? '2' : 'r');
        }
    }
    lastSwitch1 = switch1;
    lastSwitch3 = switch3;
    lastSwitch4 = switch4;
  }

  if (switch4 != lastSwitch4) {
    // Double jump whenever switch 3 and switch 4 is pressed
    if (switch3 && switch4) {
      Keyboard.press(' ');
      Keyboard.release(' ');
      delay(15);
      Keyboard.press(' ');
      Keyboard.release(' ');
    }
    else {
      if (switch4) {
        Keyboard.press(isFirst ? '3' : 'e');
      } else {
        Keyboard.release(isFirst ? '3' : 'e');
      }
    }
    lastSwitch3 = switch3; 
    lastSwitch4 = switch4;
  }

  // LED behavior: turn on when pressed, off otherwise
  setLEDs(ss, switch1, switch2, switch3, switch4);

  // Serial.print("Button states "); Serial.print("BB"); Serial.print(isFirst ? "1: " : "2: ");
  // Serial.print(switch1); Serial.print(" ");
  // Serial.print(switch2); Serial.print(" ");
  // Serial.print(switch3); Serial.print(" ");
  // Serial.print(switch4); Serial.println();
}


// Function to set LED states
void setLEDs(MySeesaw &ss, bool led1, bool led2, bool led3, bool led4) {
  ss.analogWrite(PWM1, led1 ? 127 : 0);
  ss.analogWrite(PWM2, led2 ? 127 : 0);
  ss.analogWrite(PWM3, led3 ? 127 : 0);
  ss.analogWrite(PWM4, led4 ? 127 : 0);
}

// Function to turn off all LEDs
void setVariableLEDs(MySeesaw &ss, int brightness) {
  ss.analogWrite(PWM1, brightness);
  ss.analogWrite(PWM2, brightness);
  ss.analogWrite(PWM3, brightness);
  ss.analogWrite(PWM4, brightness);
}