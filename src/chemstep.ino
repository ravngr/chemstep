// ChemStep
// Controls stepper motor from LCD keypad shield.
// Some serial control, not done yet.

#include <limits.h>

#include <EEPROM.h>
#include <LiquidCrystal.h>

#include <Bounce2.h>

/* -- Pin definitions -- */
#define PIN_MOTOR_DIR     2
#define PIN_MOTOR_STEP    3
//#define PIN_MOTOR_nEN     4
//#define PIN_MOTOR_STEP_SEL  5

#define PIN_TRIGGER       A2
#define PIN_TRIGGER_BIG   A3
#define PIN_TRIGGER_STEP  A4
#define PIN_TRIGGER_DIR   A5
#define PIN_ENDSTOP_MIN   12
#define PIN_ENDSTOP_MAX   13

#define PIN_LCD_BUTTON  A0
#define PIN_LCD_BACKLIGHT 3
#define PIN_LCD_RS  8
#define PIN_LCD_EN  9
#define PIN_LCD_D4  4
#define PIN_LCD_D5  5
#define PIN_LCD_D6  6
#define PIN_LCD_D7  7

/* -- Application -- */
#define STR_HELPER(x)     #x
#define STR(x)            STR_HELPER(x)

#define APP_NAME          "ChemStep"
#define APP_VERSION_MAJOR 2
#define APP_VERSION_MINOR 2
#define APP_STR           APP_NAME " V" STR(APP_VERSION_MAJOR) "." STR(APP_VERSION_MINOR)

/* -- Configuration -- */
#define SERIAL_SPEED      115200
#define SERIAL_BUFFER_LEN 128
//#define SERIAL_LAG_DETECT

/* -- Constants -- */
#define LCD_WIDTH         16
#define LCD_HEIGHT        2

#define MECH_THREAD_PITCH 5.0f
#define MECH_STEP_REV     200.0f
//#define MECH_STEP_MICRO   16.0f
#define MECH_STEP_MICRO   1.0f
#define MECH_DIR_INVERT

#define MECH_STEPS_PER_MM (MECH_STEP_REV * MECH_STEP_MICRO / MECH_THREAD_PITCH)

#define SPEED_MICROSECONDS_DELAY 100

/* -- Enums and constants -- */
#define KEYPAD_COUNT      5
#define KEYPAD_DEBOUNCE   10
#define KEYPAD_HOLD_TIME  1000

// Thresholds for button press on LCD shield
const static int keyAdcThreshold[KEYPAD_COUNT] = { 50, 250, 450, 650, 850 };

typedef enum { KEY_RIGHT, KEY_UP, KEY_DOWN, KEY_LEFT, KEY_SELECT, KEY_NONE } key_t;

#define UI_STATE_COUNT  2
typedef enum { UI_STEP = 0, UI_SPEED } ui_state_t;

bool ui_redraw = true;
ui_state_t ui_state = UI_STEP;

#define STEP_SIZE_COUNT 10
#define STEP_SPD_COUNT 9

#define EEPROM_STEP   0x00
#define EEPROM_SPEED  0x01

//const static float stepSize[STEP_SIZE_COUNT] = {0.10f, 0.25f, 0.50f, 1.00f, 2.50f, 5.00f, 10.00f, 25.00f, 50.00f};
const static float stepSize[STEP_SIZE_COUNT] = {1.00f, 2.50f, 5.00f, 10.00f, 25.00f, 50.00f, 75.0f, 100.0f, 125.0f, 150.0f};
static int8_t stepSizeSelect = 3;
static unsigned long stepInterval = 0;

//const static float stepSpeed[STEP_SPD_COUNT] = {0.1f, 0.2f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f};
const static float stepSpeed[STEP_SPD_COUNT] = {5.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f, 80.0f};
static int8_t stepSpeedSelect = 3;

/* -- Default Parameters -- */
typedef enum {STEP_FWD, STEP_REV} stepper_dir_t;

stepper_dir_t stepper_dir = STEP_REV;
bool stepperState = false;
long stepperPosition = 0;
long stepperTarget = 0;
unsigned long nextStep = 0;

/* -- Debounce -- */
Bounce triggerBounce = Bounce();
Bounce triggerBigBounce = Bounce();
Bounce triggerDirBounce = Bounce();
Bounce triggerStepBounce = Bounce();

/* -- Initialize libraries -- */
LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);

static uint8_t moveChar[8] = {
  0b01000,
  0b01100,
  0b01110,
  0b01111,
  0b01110,
  0b01100,
  0b01000,
  0b00000
};

static uint8_t stopChar[8] = {
  0b00000,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b00000,
  0b00000
};

void updateInterval() {
  stepInterval = 1000000UL / (MECH_STEPS_PER_MM * stepSpeed[stepSpeedSelect] * 2.0f);
}

void get_lcd_key(key_t *current_key, bool *updated) {
  static key_t lastKey = KEY_NONE;
  static unsigned long lastKeyMillis = 0;
  key_t key = KEY_NONE;

  *updated = false;

  int adcValue = analogRead(PIN_LCD_BUTTON);

  for (unsigned char n = 0; n < KEYPAD_COUNT; n++) {
    if (adcValue < keyAdcThreshold[n]) {
      key = (key_t) n;
      break;
    }
  }

  if (key != lastKey) {
    // Debounce value
    if (millis() > (lastKeyMillis + KEYPAD_DEBOUNCE)) {
      lastKey = key;
      *updated = true;
    } else {
      key = lastKey;
    }
  } else {
    lastKeyMillis = millis();
  }

  *current_key = key;
}


void updateSerial() {
  static char serialBuffer[SERIAL_BUFFER_LEN];
  static unsigned int serialCount = 0;

  char serialChar;
  bool serialProcess = false;

  while (Serial.available()) {
    serialChar = Serial.read();

    if (serialChar == '\r') {
      // Process string
      serialProcess = true;
      break;
    } else if (serialChar == '\n') {
      // Skip newlines
      continue;
    } else if (serialChar == 127) {
      // Backspace
      if (serialCount > 0) {
        serialCount--;
        Serial.write(serialChar);
      }
    } else {
      serialBuffer[serialCount++] = serialChar;
      Serial.write(serialChar);
    }

    // Force processing when buffer is full
    if (serialCount >= (SERIAL_BUFFER_LEN - 1)) {
      serialProcess = true;
      break;
    }
  }

  if (serialProcess) {
    Serial.println();

    // Terminate string
    serialBuffer[serialCount] = '\0';

    if (serialCount == 1) {
      // Single character commands
      switch (serialBuffer[0]) {
        case 's':
        case 'S':
          Serial.println("STOP");
          stepperTarget = stepperPosition;
          break;

        case 'i':
          // Incremental step
          break;

        case 'a':
          // Absolute target
          break;

        case '+':
          Serial.println("FWD");
          stepperTarget++;
          break;

        case '-':
          Serial.println("REV");
          stepperTarget--;
          break;
      }
    } else {
      // Numerical input

    }

    /*Serial.print("Process [");
    Serial.print(serialBuffer);
    Serial.print("] (");
    Serial.print(serialCount, DEC);
    Serial.println(")");*/

    // Reset state
    serialCount = 0;
  }
}


void updateInput() {
  key_t key;
  bool updated;
  static long held_time;
  static boolean held = false;

  // Get currently pressed LCD key
  get_lcd_key(&key, &updated);

  if (updated) {
    held_time = millis();

    if (key != KEY_NONE) {
      switch (key) {
      case KEY_LEFT:
        // Move backward
        stepperTarget -= (long) (MECH_STEPS_PER_MM * stepSize[stepSizeSelect]);
        break;

      case KEY_RIGHT:
        // Move forward
        stepperTarget += (long) (MECH_STEPS_PER_MM * stepSize[stepSizeSelect]);
        break;

      case KEY_UP:
        // Increment setting
        switch (ui_state) {
        case UI_STEP:
          if (++stepSizeSelect >= STEP_SIZE_COUNT)
            stepSizeSelect = 0;

          EEPROM.write(EEPROM_STEP, stepSizeSelect);

          break;

        case UI_SPEED:
          if (++stepSpeedSelect >= STEP_SPD_COUNT)
            stepSpeedSelect = 0;

          EEPROM.write(EEPROM_SPEED, stepSpeedSelect);

          updateInterval();

          break;
        }

        break;

      case KEY_DOWN:
        // Decrement setting
        switch (ui_state) {
        case UI_STEP:
          if (stepSizeSelect-- == 0)
            stepSizeSelect = STEP_SIZE_COUNT - 1;

          EEPROM.write(EEPROM_STEP, stepSizeSelect);

          break;

        case UI_SPEED:
          if (stepSpeedSelect-- == 0)
            stepSpeedSelect = STEP_SPD_COUNT - 1;

          EEPROM.write(EEPROM_SPEED, stepSpeedSelect);

          updateInterval();

          break;
        }

        break;

      case KEY_SELECT:
        // Switch display modes
        ui_state = (ui_state_t) ((int) ui_state + 1);

        if (ui_state >= UI_STATE_COUNT)
          ui_state = (ui_state_t) 0;

        break;

      default:
        // Do nothing (prevents compiler warning)
        break;
      }

      // Update UI
      ui_redraw = true;
    }
  }

  if ((millis() - held_time) > KEYPAD_HOLD_TIME && (key == KEY_LEFT || key == KEY_RIGHT || key == KEY_SELECT)) {
    // Move while button is held
    if (key == KEY_RIGHT) {
      stepperTarget = LONG_MAX;
    } else if (key == KEY_LEFT) {
      stepperTarget = LONG_MIN;
    } else if (key == KEY_SELECT) {
      // Reset position
      stepperTarget = 0;
      stepperPosition = 0;

      ui_redraw = true;
      held_time = millis();
    }

    held = true;
  } else if (held) {
    // Stop movment
    stepperTarget = stepperPosition;

    held = false;

    ui_redraw = true;
  }
}


void updateLCD() {
  if (!ui_redraw)
    return;

  lcd.clear();

  // Display position
  lcd.print("! ");
  lcd.print(((float) stepperTarget) / MECH_STEPS_PER_MM, 2);
  lcd.print("mm");

  lcd.setCursor(0, 1);

  // Display setting
  switch (ui_state) {
  case UI_STEP:
    lcd.print("Inc: ");
    lcd.print(stepSize[stepSizeSelect], 2);
    lcd.print("mm");
    break;

  case UI_SPEED:
    lcd.print("Spd: ");
    lcd.print(stepSpeed[stepSpeedSelect], 2);
    lcd.print("mm/s");
    break;
  }

  ui_redraw = false;
}


void setup(){
  int8_t x;

  /* Serial setup */
  Serial.begin(SERIAL_SPEED);

  /* LCD setup */
  lcd.begin(LCD_WIDTH, LCD_HEIGHT);

  lcd.createChar(0, stopChar);
  lcd.createChar(1, moveChar);

  // Print application info
  lcd.setCursor(0, 0);
  lcd.print(APP_STR);
  lcd.setCursor(0, 1);
  lcd.print(__DATE__);

  delay(1000);

  /* Triggering setup */
  triggerBounce.attach(PIN_TRIGGER, INPUT_PULLUP);
  triggerBigBounce.attach(PIN_TRIGGER_BIG, INPUT_PULLUP);
  triggerDirBounce.attach(PIN_TRIGGER_DIR, INPUT_PULLUP);
  triggerStepBounce.attach(PIN_TRIGGER_STEP, INPUT_PULLUP);

  pinMode(PIN_ENDSTOP_MIN, INPUT_PULLUP);
  pinMode(PIN_ENDSTOP_MAX, INPUT_PULLUP);

  pinMode(PIN_MOTOR_STEP, OUTPUT);
  pinMode(PIN_MOTOR_DIR, OUTPUT);
  //pinMode(PIN_MOTOR_nEN, OUTPUT);
  //pinMode(PIN_MOTOR_STEP_SEL, OUTPUT);

  //digitalWrite(PIN_MOTOR_nEN, LOW);
  //digitalWrite(PIN_MOTOR_STEP_SEL, LOW);

  // Load settings
  x = EEPROM.read(EEPROM_STEP);

  if (x >= 0 && x < STEP_SIZE_COUNT)
    stepSizeSelect = x;

  x = EEPROM.read(EEPROM_SPEED);

  if (x >= 0 && x < STEP_SPD_COUNT)
    stepSpeedSelect = x;

  // Initial speed calculation
  updateInterval();
}

void loop() {

  bool moving = false;

  while (1) {
    // Process any waiting serial commands
    updateSerial();

    // Check external inputs
    updateInput();

    // Restrict motion when endstops are hit
    if (stepperTarget > stepperPosition && digitalRead(PIN_ENDSTOP_MIN) == LOW)
      stepperTarget = stepperPosition;

    if (stepperTarget < stepperPosition && digitalRead(PIN_ENDSTOP_MAX) == LOW)
      stepperTarget = stepperPosition;

    // Run motor
    if (stepperTarget != stepperPosition) {
      if (!moving) {
        lcd.setCursor(0, 0);
        lcd.print(">");
        moving = true;
      }

      if (micros() >= nextStep) {
        // Calculate time for next step
        nextStep = micros() + stepInterval;

        if (!stepperState) {
          // Check for direction change on positive edge
          if (stepper_dir == STEP_FWD) {
            if (stepperTarget < stepperPosition) {
              stepper_dir = STEP_REV;

#ifndef MECH_DIR_INVERT
              digitalWrite(PIN_MOTOR_DIR, HIGH);
#else
              digitalWrite(PIN_MOTOR_DIR, LOW);
#endif
            }
          } else {
            if (stepperTarget > stepperPosition) {
              stepper_dir = STEP_FWD;

#ifndef MECH_DIR_INVERT
              digitalWrite(PIN_MOTOR_DIR, LOW);
#else
              digitalWrite(PIN_MOTOR_DIR, HIGH);
#endif
            }
          }

          digitalWrite(PIN_MOTOR_STEP, HIGH);
        } else {
          digitalWrite(PIN_MOTOR_STEP, LOW);

          // Count step
          stepperPosition += (stepper_dir == STEP_FWD ? 1 : -1);
        }

        // Update state
        stepperState = !stepperState;

        // Check for loop lag (which will make the lag worse...)
#ifdef SERIAL_LAG_DETECT
        if (micros() > nextStep) {
          Serial.println("LAG!");
        }
#endif
      }
    } else {
      if (moving) {
        ui_redraw = true;
        moving = false;
      }

      // Force next step if necessary
      nextStep = 0;

      // Update LCD
      updateLCD();
    }
  }
}
