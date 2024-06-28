/*

MarIOnette template file for Arduino-compatible boards

Tested on Arduino Nano (ATMega 328P), Arduino Uno, Teensy 3.2, Teensy 3.6, and
Teensy 4.1

*/

#define DEBUG_SETUP 1
#define DEBUG_SERIAL 0
#define DEBUG_SD_PLAYBACK_TEST 0

#include "config.h"

#include "Arduino.h"

// For AVR - based boards without a dedicated Serial buffer, we need to add a
// small delay between Serial reads Teensy appears to use a default 12-bit
// resolution for Analog writes...
#if defined(__AVR__)
#define IS_AVR 1
#define SERIAL_DELAY 1
#define ANALOG_MAX 255
#else
#define IS_AVR 0
#define SERIAL_DELAY 5
#define ANALOG_MAX 4095
#endif

// MarIOnette serial
unsigned int counter = 0;
unsigned int howManyBytes = 0;

// Expected packet
const int expectedMotorBytes = TOTAL_MOTORS * 2;
const int expectedSpeedBytes = 2;

// Servos: use PWMServo library if we have neopixels, otherwise use regular
#if TOTAL_SERVOS > 0 && TOTAL_NEOPIXELS == 0
#include <Servo.h>
Servo servos[TOTAL_SERVOS];
#elif TOTAL_SERVOS > 0 && TOTAL_NEOPIXELS > 0
#include "PWMServo.h"
PWMServo servos[TOTAL_SERVOS];
#endif

// Stepper library
#if TOTAL_STEPPERS > 0
#include <AccelStepper.h>
AccelStepper steppers[TOTAL_STEPPERS];
#endif

// LX16A
#if TOTAL_BUS_SERVOS > 0
#include "LSServo.h"
LSServo BusServos;
#endif

unsigned int busServoSpeed;
unsigned int readingPositions = 0;

// Dynamixel
#if TOTAL_DYNAMIXELS > 0:
#include "Dynamixel2Arduino.h"
Dynamixel2Arduino dxl(DYNAMIXEL_SERIAL_PORT, DYNAMIXEL_DIR_PIN);
// This namespace is required to use Control table item names
using namespace ControlTableItem;
int oldDynamixelSpeed = DYNAMIXEL_SPEED;
#endif

// SD card
#if SD_ENABLE
#include <SD.h>
#include <SPI.h>
#if USE_SD_INTERNAL
const int SDChipSelect = BUILTIN_SDCARD;

#else
const int SDChipSelect = SD_CS_PIN;
#endif
File animFile;
// SD Animation
unsigned long currentFrame;
unsigned int FPS;
unsigned long totalFrames;
unsigned int frameByteLength;
unsigned long frameInterval;
unsigned long animationTimer;
char filename[20];
#endif

unsigned int playingAnimation;

// Setup function based on all included motors and leds
void setupAll() {
#if SD_ENABLE
  while (!SD.begin(SDChipSelect)) {
    Serial.println(
        "No SD card found or incorrect wiring! Retrying in 3 seconds...");
    delay(3000);
  }
#endif

#if TOTAL_MOTORS > 0
  for (int i = 0; i < TOTAL_MOTORS; i++) {
#if TOTAL_SERVOS > 0
    // Servos
    if (motor_values[i][0] == 1) {
      servos[i].attach(motor_values[i][1]);
    }
#endif
    // PWM Pin
    if (motor_values[i][0] == 2) {
      pinMode(motor_values[i][1], OUTPUT);
    }
    // ON/OFF Pin
    if (motor_values[i][0] == 3) {
      pinMode(motor_values[i][1], OUTPUT);
    }
    // PWM Bi-Directional
    if (motor_values[i][0] == 4) {
      pinMode(motor_values[i][1], OUTPUT);
      pinMode(motor_values[i][2], OUTPUT);
    }
#if TOTAL_BUS_SERVOS > 0
    // Bus Servos
    if (motor_values[i][0] == 7) {
      BUS_SERVO_SERIAL_PORT.begin(BUS_SERVO_BAUD);
      BusServos.pSerial = &BUS_SERVO_SERIAL_PORT;
      Serial.println("Connected to bus servo on Serial1");
    }
#endif
#if TOTAL_DYNAMIXELS > 0
    // Dynamixels
    dxl.begin(DYNAMIXEL_BAUD);
    dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VER);

    for (int i = 0; i < TOTAL_DYNAMIXELS + 1; i++) {
      dxl.torqueOff(i);
      dxl.setOperatingMode(i, OP_POSITION);
      dxl.torqueOn(i);
      dxl.writeControlTableItem(PROFILE_VELOCITY, i, DYNAMIXEL_SPEED);
    }
#endif

#if TOTAL_STEPPERS > 0
    // Steppers
    Serial.println("Starting stepper initialization...");
    for (int i = 0; i < TOTAL_STEPPERS; i++) {
      for (int j = 0; j < TOTAL_MOTORS; j++) {

        if (motor_values[j][0] == 5) {
          steppers[i] = AccelStepper(steppers[i].DRIVER, motor_values[j][1],
                                     motor_values[j][2]);
          steppers[i].setMaxSpeed(motor_values[j][4]); // 100mm/s @ 80 steps/mm
          steppers[i].setAcceleration(motor_values[j][5]); // 2000mm/s^2
          steppers[i].setPinsInverted(false, false, true);
          steppers[i].enableOutputs();
          i++;
        }
      }
    }
#endif // TOTAL_STEPPERS > 0

#if TOTAL_LEDS > 0
    // PWM LEDs
    for (int i = 0; i < TOTAL_LEDS; i++) {
      if (led_values[i][0] == 11) {
        pinMode(led_values[i][2], OUTPUT);
        analogWrite(led_values[i][2], 0);
      }
    }
#endif

    // Neopixels and others
#if (TOTAL_NEOPIXEL + TOTAL_DOTSTARS) > 0:
#error "Neopixel and Dotstar init support not migrated from the python code!"
    // TODO FIXME
    // numz = 0
    // for blah in range(0, len(scn.my_list2)):
    //     if scn.my_list2[blah].LEDType == "Neopixel":

    //         outputfile.write(
    //             (
    //                 "\n\n FastLED.addLeds<NEOPIXEL, "
    //                 + str(scn.my_list2[blah].DataPin)
    //                 + ">(neopixels["
    //                 + str(numz)
    //                 + "], "
    //                 + str(max_neopixels)
    //                 + ");\n"
    //             ).encode()
    //         )
    //         numz += 1
    //     elif scn.my_list2[blah].LEDType == "Dotstar":
    //         outputfile.write(
    //             (
    //                 "\n\n FastLED.addLeds<DOTSTAR, "
    //                 + str(scn.my_list2[blah].DataPin)
    //                 + ", "
    //                 + str(scn.my_list2[blah].ClockPin)
    //                 + ", "
    //                 + str(scn.my_list2[blah].NeopixelOrder)
    //                 + ">(neopixels["
    //                 + str(numz)
    //                 + "], "
    //                 + str(max_neopixels)
    //                 + ");\n"
    //             ).encode()
    //         )
    //         numz += 1

// outputfile.write(
//     ("\n FastLED.clear();\n FastLED.show();\n").encode()
// )
#endif
  }
#endif // TOTAL_MOTORS > 0
}

void readSerialBytes() {
  int debugOut = 0;
  char inByte;

  if (Serial.available() > 0) {
    char mode = Serial.read();

    // Blender sending data...
    if (mode == 'A' || mode == 'B') {
      counter = 0;

      if (IS_AVR) {
        delay(SERIAL_DELAY);
      }

      else {
        delayMicroseconds(SERIAL_DELAY);
      }

      // Read in first bytes to get message length
      char one = Serial.read();

      if (IS_AVR) {
        delay(SERIAL_DELAY);
      }

      else {
        delayMicroseconds(SERIAL_DELAY);
      }

      char two = Serial.read();

      if (mode == 'A') {
        howManyBytes = word(one, two) + expectedSpeedBytes;
      }

      else {
        howManyBytes = word(one, two);
      }

      if (debugOut) {
        Serial.print("Expecting bytes: ");
        Serial.println(howManyBytes);
      }

      char tempBuffer[howManyBytes];

      while (Serial.available()) {
        if (IS_AVR) {
          delay(SERIAL_DELAY);
        }

        inByte = Serial.read();

        // Keep reading in case last byte is the stop char
        if (counter < howManyBytes) {
          tempBuffer[counter] = inByte;

          counter++;
          if (debugOut) {
            // Serial.println(counter);
          }
        }

        // Received too many bytes but no stop character...
        else if (counter >= howManyBytes && inByte != '.') {
          Serial.print("Too many bytes, expected ");
          while (Serial.available()) {
            Serial.read();
            counter++;
          }

          if (debugOut) {
            Serial.print("Too many bytes, expected ");
            Serial.print(howManyBytes);
            Serial.print(" and got ");
            Serial.println(counter);
          }

          counter = 0;
          break;
        }

        // Right amount of bytes received, set motors and LEDs
        else if (counter == howManyBytes && (inByte == '.' || inByte == 46)) {
          if (debugOut) {
            Serial.println("Success!");
          }

          if (mode == 'A') {
            updateMotorsAndLEDs(tempBuffer, 0);
            busServoSpeed = word(tempBuffer[0], tempBuffer[1]);
          }

          else {
            updateMotorsAndLEDs(tempBuffer, 1);
            busServoSpeed = 0;
          }
        }
      }
    }

    // Play back file from SD card
    else if (mode == 'P' && !playingAnimation && SD_ENABLE) {
      SDHelper(1);
    }

    // Stop file playback
    else if (mode == 'S' && playingAnimation && SD_ENABLE) {
      SDHelper(0);
      Serial.println("Animation stopped by serial");
    }

    // IN PROGRESS!!!
    // Read values from dynamixels or bus servos
    else if (mode == 'R' && !readingPositions && SD_ENABLE) {
      if (TOTAL_BUS_SERVOS > 0 || TOTAL_DYNAMIXELS > 0) {
        // Read how long the animation is and at what FPS it runs
        // readingPositions = 1;

        // Serial.parseInt();
        // totalFrames = Serial.parseInt();
        // FPS = Serial.parseInt();

        // frameInterval = 1000*1000/FPS; // In microseconds

        // animationTimer = 0;
      }

      else {
        Serial.println("No bus servos or dynamixels configured!");
      }
    }

    // Cancel position reads
    else if (mode == 'Z' && readingPositions) {
      readingPositions = 0;
    }
  }
}

void setup() {
  // Start Serial monitor
  Serial.begin(BAUD_RATE_SERIAL);
  Serial.println("Starting MarIOnette initialization...");

  // Perform setup, which gets dynamically generated by MarIOnette
  setupAll();

  Serial.println("Setup finished!");

  /*
  // Uncomment this block to test SD card animation playback
  // Change the testFile name to whatever you exported the animation as, and
  then trigger it to run if(DEBUG_SD_PLAYBACK_TEST){ String testFile =
  "test8"; int length = testFile.length() + 1; testFile.toCharArray(filename,
  length); playingAnimation = 1; readAnimationFile();
  }
  */
}

void playAnimationFile() {
#if SD_ENABLE
  if (playingAnimation) {
    if (micros() - animationTimer > frameInterval) {
      if (currentFrame == totalFrames) {
        playingAnimation = 0;
        animFile.close();
        Serial.println("Animation done!");
        return;
      }

      char frame_buffer[frameByteLength];
      for (unsigned int i = 0; i < frameByteLength; i++) {
        frame_buffer[i] = animFile.read();
      }

      updateMotorsAndLEDs(frame_buffer, 1);
      currentFrame++;
      animationTimer = micros();
    }
  }
#endif
}

void readAnimationFile() {
#if SD_ENABLE
  // If file is found, parse the header and begin playback
  if (SD.exists(filename)) {
    Serial.print("File '");
    Serial.print(filename);
    Serial.print("' found! Size: ");
    animFile = SD.open(filename);
    Serial.println(animFile.size());

    animFile.read(); // LED mode
    totalFrames = animFile.parseInt();
    FPS = animFile.parseInt();
    frameByteLength = animFile.parseInt();

    animFile.read(); // newline
    animFile.read(); // carriage return

    frameInterval = 1000 * 1000 / FPS; // In microseconds

    Serial.print("Total frames: ");
    Serial.print(totalFrames);
    Serial.print(" | FPS: ");
    Serial.print(FPS);
    Serial.print(" | Bytes per frame: ");
    Serial.println(frameByteLength);

    playingAnimation = 1;
    currentFrame = 0;
    animationTimer = micros();
  }

  // File not found
  else {
    Serial.println("File not found on SD card!");
    playingAnimation = 0;
  }
#endif
}

void SDHelper(int mode) {
#if SD_ENABLE
  if (mode == 0) {
    playingAnimation = 0;
    animFile.close();
  }

  else {
    int i = 0;

    // Reset filename
    for (int j = 0; j < 20; j++) {
      filename[j] = '\0';
    }

    while (Serial.available()) {
      char c = Serial.read();

      if (c != '\n') {
        filename[i] += c;
        i++;
      }

      else {
        filename[i] = '\0';
        Serial.flush();
      }

      if (IS_AVR) {
        delay(SERIAL_DELAY);
      }

      else {
        delayMicroseconds(SERIAL_DELAY);
      }
    }

    Serial.print("Filename: ");
    Serial.println(filename);

    readAnimationFile();
  }
#endif
}
// Update function based on all included motors and leds
void updateMotorsAndLEDs(char frame_buffer[], int mode) {
#if TOTAL_MOTORS > 0 // if len(scn.my_list) > 0

  int servo_index = 0;
  int stepper_index = 0;

  for (int i = 0; i < TOTAL_MOTORS; i++) {
    int offset = i * 2;

    // Speed bytes are sent in case we have bus servos...
    if (mode == 0) {
      offset = i * 2 + expectedSpeedBytes;
    }

    unsigned int motor_value =
        word(frame_buffer[offset], frame_buffer[offset + 1]);

    // Min and Max values
    if (motor_value > motor_values[i][9] && motor_values[i][8] != 0 &&
        motor_values[i][9] != 0) {
      motor_value = motor_values[i][9];
    }

    else if (motor_value < motor_values[i][8] && motor_values[i][8] != 0 &&
             motor_values[i][9] != 0) {
      motor_value = motor_values[i][8];
    }

// if num_servos > 0 and num_neopixel == 0
#if TOTAL_SERVOS > 0 && TOTAL_NEOPIXELS == 0
    // Set servo position
    if (motor_values[i][0] == 1) {
      servos[i].writeMicroseconds(motor_value);
      servo_index++;
    }
#elif TOTAL_SERVOS > 0 && TOTAL_NEOPIXELS > 0
    // elif num_servos > 0 and num_neopixel > 0
    // Set servo position
    if (motor_values[i][0] == 1) {
      servos[i].write(motor_value);
      servo_index++;
    }
#endif
    // Set PWM value
    if (motor_values[i][0] == 2) {
      analogWrite(motor_values[i][1], map(motor_value, 0, 4000, 0, ANALOG_MAX));
    }

    // Set ON/OFF value
    if (motor_values[i][0] == 3) {
      if (motor_value > 0) {
        digitalWrite(motor_values[i][1], HIGH);
      } else {
        digitalWrite(motor_values[i][1], LOW);
      }
    }

// if num_steppers > 0:
#if TOTAL_STEPPERS > 0
    // Set stepper position
    if (motor_values[i][0] == 5) {
      steppers[stepper_index].moveTo(motor_value);
      stepper_index++;
    }
#endif

// if num_lewansoul > 0:
#if TOTAL_BUS_SERVOS > 0
    // Set Lewansoul servos
    if (motor_values[i][0] == 7) {
      BusServos.SetPos(motor_values[i][1], motor_value,
                       busServoSpeed); // Might need to remap the speed...
    }
#endif

// if num_dynamixel > 0:
#if TOTAL_DYNAMIXELS > 0
    // Set Dynamixel
    if (motor_values[i][0] == 8) {
      if (busServoSpeed != oldDynamixelSpeed) {
        for (int i = 0; i < TOTAL_DYNAMIXELS + 1; i++) {
          dxl.writeControlTableItem(PROFILE_VELOCITY, i, busServoSpeed);
        }
        oldDynamixelSpeed = busServoSpeed;
      }
      dxl.setGoalPosition(motor_values[i][1], motor_value);
    }
#endif

    // Set Bi-directional PWM
    if (motor_values[i][0] == 4) {
      if (motor_values[i][3] == 1) {
        if (motor_value > ANALOG_MAX / 2) {
          analogWrite(motor_values[i][1], map(motor_value, ANALOG_MAX / 2,
                                              ANALOG_MAX, 0, ANALOG_MAX));
          digitalWrite(motor_values[i][2], HIGH);
        }

        else {
          analogWrite(motor_values[i][1],
                      map(motor_value, ANALOG_MAX / 2, 0, 0, ANALOG_MAX));
          digitalWrite(motor_values[i][2], LOW);
        }
      }

      else {
        if (motor_value > ANALOG_MAX / 2) {
          analogWrite(motor_values[i][2], map(motor_value, ANALOG_MAX / 2,
                                              ANALOG_MAX, 0, ANALOG_MAX));
          digitalWrite(motor_values[i][1], LOW);
        }

        else {
          analogWrite(motor_values[i][1],
                      map(motor_value, ANALOG_MAX / 2, 0, 0, ANALOG_MAX));
          digitalWrite(motor_values[i][2], LOW);
        }
      }
    }
  }
#endif // TOTAL_MOTORS > 0

// if len(scn.my_list2) > 0
#if TOTAL_LEDS > 0

  long offset = expectedMotorBytes;

  // Speed bytes are sent in case we have bus servos...
  if (mode == 0) {
    offset = expectedMotorBytes + expectedSpeedBytes;
  }

  unsigned int neopixel_index = 0;

  for (int i = 0; i < TOTAL_LEDS; i++) {

    // PWM LED
    if (led_values[i][0] == 11) {
      analogWrite(led_values[i][1],
                  map(int(frame_buffer[offset]), 0, 254, 0, ANALOG_MAX));
      offset++;
    }
    // if num_neopixel + num_dotstars > 0
#if (TOTAL_NEOPIXEL + TOTAL_DOTSTARS) > 0

    // Neopixel single color
    else if (led_values[i][0] == 10 && frame_buffer[offset] == 1) {
      offset++;
      int red = int(frame_buffer[offset]);
      int green = int(frame_buffer[offset + 1]);
      int blue = int(frame_buffer[offset + 2]);
      int white = 0;

      if (led_values[i][4] == 1) {
        white = int(frame_buffer[offset + 3]);
        offset += 4;
      }

      else {
        offset += 3;
      }

      if (led_values[i][4] == 1) {
        fill_solid(neopixels[neopixel_index], MAX_NEOPIXELS,
                   CRGB(red, green, blue));
      } else {
        fill_solid(neopixels[neopixel_index], MAX_NEOPIXELS,
                   CRGB(red, green, blue));
      }

      FastLED.show();
      neopixel_index++;
    }

    // Individually addressible
    else if (led_values[i][0] == 10 && frame_buffer[offset] == 0) {
      offset++;

      for (unsigned int j = 0; j < led_values[i][3]; j++) {
        int red = int(frame_buffer[offset]);
        int green = int(frame_buffer[offset + 1]);
        int blue = int(frame_buffer[offset + 2]);

        if (led_values[i][4] == 1) {
          int white = int(frame_buffer[offset + 3]);
          neopixels[neopixel_index][j].setRGB(red, green, blue);
          offset += 4;
        }

        else {
          neopixels[neopixel_index][j].setRGB(red, green, blue);
          offset += 3;
        }
      }

      FastLED.show();
      neopixel_index++;
    }
#endif
  }
#endif
}

void updateSteppers() {
#if TOTAL_STEPPERS > 0
  for (int i = 0; i < TOTAL_STEPPERS; i++) {
    steppers[i].run();
  }
#endif
}

void loop() {
  // Begin reading serial port for incoming commands
  readSerialBytes();
  if (playingAnimation) {
    playAnimationFile();
  }
  updateSteppers();
}
