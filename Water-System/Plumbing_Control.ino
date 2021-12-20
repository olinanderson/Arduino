#include <Arduino.h>
#include <BLESerial.h>

BLESerial bleSerial;

// Relay board 1
const int pumpOnePin = 13;           // in 1
const int pumpTwoPin = 14;           // in 3
const int overallLEDPin = 26;        // in 5
const int everythingElseLEDPin = 25; // in 6
const int recircLEDPin = 33;         // in 7

// Relay board 2
const int recircFlushLEDPin = 22; // in 1
const int valveOnePin = 21;       // in 2
const int valveTwoPin = 19;       // in 3
const int valveThreePin = 18;     // in 4
const int valveFourPin = 16;      // in 5
const int valveFivePin = 23;      // in 6
const int valveSixPin = 17;       // in 7
const int valveSevenPin = 15;     // in 8

// Button pins
const int overallButtonPin = 35;        // Button State 1
const int everythingElseButtonPin = 34; // Button State 2
const int recircButtonPin = 32;         // Button State 3
const int recircFlushButtonPin = 36;    // Button State 4

int overallButtonState = 0; // Will be 1 when a button is pressed down
int everythingElseButtonState = 0;
int recircButtonState = 0;
int recircFlushButtonState = 0;

// Default state is 2
int overallButtonOn = false;
int everythingElseButtonOn = true;
int recircButtonOn = false;
int recircFlushButtonOn = false;

int lastOverallButtonState = 0;
int lastEverythingElseButtonState = 0;
int lastRecircButtonState = 0;
int lastRecircFlushButtonState = 0;

// State for last and current button pressed
int lastButtonPressed = 35;
int buttonPressed = 0;

unsigned long lastValveChange = 0;

int lastState = 2;
int currentState = 2;

// LED blinking states
unsigned long currentBlinkMillis = 0;
unsigned long previousBlinkMillis = 0;

unsigned long blinkDuration = 1000;
int ledState = LOW;

unsigned long currentFlushMillis = 0;
unsigned long previousFlushMillis = 0;

unsigned long flushDuration =
    70000; // Flush duration of two minutes after valves change 1:10
unsigned long valveDuration = 13000; // Valve duration of twenty seconds

unsigned long debounceDelay = 100;

unsigned long lastOverallDebounceTime = 0;
unsigned long lastEverythingElseDebounceTime = 0;
unsigned long lastRecircDebounceTime = 0;
unsigned long lastRecircFlushDebounceTime = 0;

boolean startup = true;

void setup() {
  pinMode(pumpOnePin, OUTPUT);
  pinMode(pumpTwoPin, OUTPUT);
  pinMode(overallLEDPin, OUTPUT);
  pinMode(everythingElseLEDPin, OUTPUT);
  pinMode(recircLEDPin, OUTPUT);

  pinMode(recircFlushLEDPin, OUTPUT);
  pinMode(valveOnePin, OUTPUT);
  pinMode(valveTwoPin, OUTPUT);
  pinMode(valveThreePin, OUTPUT);
  pinMode(valveFourPin, OUTPUT);
  pinMode(valveFivePin, OUTPUT);
  pinMode(valveSixPin, OUTPUT);
  pinMode(valveSevenPin, OUTPUT);

  pinMode(overallButtonPin, INPUT);
  pinMode(everythingElseButtonPin, INPUT);
  pinMode(recircButtonPin, INPUT);
  pinMode(recircFlushButtonPin, INPUT);

  Serial.begin(115200);
  bleSerial.begin("A-WATER-SYSTEM");

  Serial.println("End of setup");
}

String command;

void loop() {

  // Bluetooth commands

  // If we're connected
  if (bleSerial.available()) {
    command = bleSerial.readStringUntil('\n');

    if (command.length() > 0) {

      if (command == "overall_on") {
        overallButtonOn = true;
        Serial.println("Virtual overall button on.");

      } else if (command == "overall_off") {
        overallButtonOn = false;
        Serial.println("Virtual overall button off.");

      } else if (command == "else" && overallButtonOn &&
                 !everythingElseButtonOn) {
        // If the button has been released && the button wasn't on before
        everythingElseButtonOn = true;
        recircButtonOn = false;
        recircFlushButtonOn = false;
        currentState = 2;
        Serial.println("Virtual else button pressed.");

      } else if (command == "recirc" && overallButtonOn && !recircButtonOn) {
        // If the button has been released && the button wasn't on before
        everythingElseButtonOn = false;
        recircButtonOn = true;
        recircFlushButtonOn = false;
        currentState = 3;
        Serial.println("Virtual recirc button pressed.");

      } else if (command == "flush" && overallButtonOn &&
                 !recircFlushButtonOn) {
        // If the button has been released && the button wasn't on before
        everythingElseButtonOn = false;
        recircButtonOn = false;
        recircFlushButtonOn = true;
        currentState = 4;
        Serial.println("Virtual flush button pressed.");
      }
      command = "";
    }
  }
  delay(20);

  // ----- Overall Button STATES -----
  overallButtonState = digitalRead(overallButtonPin);

  // Checking if the button has been pressed
  if ((millis() - lastOverallDebounceTime) > debounceDelay) {
    // Serial.println("Overall button has been pressed longer than debounce
    // time");
    if (overallButtonState != lastOverallButtonState) {
      lastOverallDebounceTime = millis();

      if (overallButtonState == LOW) { // If the button has been released
        overallButtonOn =
            !overallButtonOn; // Switch the state of the overall button
        Serial.println("Real overall button pressed.");
      }
    }
  }
  lastOverallButtonState = overallButtonState;

  // ----- Everything Else Button STATE -----
  everythingElseButtonState = digitalRead(everythingElseButtonPin);
  // everythingElseButtonOn = checkButtonOn(everythingElseButtonState,
  // lastEverythingElseButtonState, everythingElseButtonOn,
  // everythingElseButtonPin);

  // Checking if the button has been pressed
  if ((millis() - lastEverythingElseDebounceTime) > debounceDelay) {
    if (everythingElseButtonState != lastEverythingElseButtonState) {
      lastEverythingElseDebounceTime = millis();
      if (overallButtonOn && everythingElseButtonState == LOW &&
          !everythingElseButtonOn) { // If the button has been released && the
        // button wasn't on before
        everythingElseButtonOn = true;
        recircButtonOn = false;
        recircFlushButtonOn = false;
        currentState = 2;
        Serial.println("Real else button pressed.");
      }
    }
  }

  lastEverythingElseButtonState = everythingElseButtonState;

  // ----- Recirc Button STATE -----
  recircButtonState = digitalRead(recircButtonPin);
  // recircButtonOn = checkButtonOn(recircButtonState, lastRecircButtonState,
  // recircButtonOn, recircButtonPin);

  // Checking if the button has been pressed
  if ((millis() - lastRecircDebounceTime) > debounceDelay) {
    if (recircButtonState != lastRecircButtonState) {
      lastRecircDebounceTime = millis();
      if (overallButtonOn && recircButtonState == LOW &&
          !recircButtonOn) { // If the button has been released && the button
        // wasn't on before
        everythingElseButtonOn = false;
        recircButtonOn = true;
        recircFlushButtonOn = false;
        currentState = 3;
        Serial.println("Real recirc button pressed.");
      }
    }
  }
  lastRecircButtonState = recircButtonState;

  // ----- Recirc Flush Button STATE -----
  recircFlushButtonState = digitalRead(recircFlushButtonPin);

  // Checking if the button has been pressed
  if ((millis() - lastRecircFlushDebounceTime) > debounceDelay) {
    if (recircFlushButtonState != lastRecircFlushButtonState) {
      lastRecircFlushDebounceTime = millis();
      if (overallButtonOn && recircFlushButtonState == LOW &&
          !recircFlushButtonOn) { // If the button has been released && the
        // button wasn't on before
        everythingElseButtonOn = false;
        recircButtonOn = false;
        recircFlushButtonOn = true;
        currentState = 4;
        Serial.println("Real flush button pressed.");
      }
    }
  }
  lastRecircFlushButtonState = recircFlushButtonState;

  if (startup) {
    // Setting valves to everything else mode on startup once
    digitalWrite(valveOnePin, LOW);
    digitalWrite(valveTwoPin, HIGH);
    digitalWrite(valveThreePin, HIGH);
    digitalWrite(valveFourPin, LOW);
    digitalWrite(valveFivePin, HIGH);
    digitalWrite(valveSixPin, LOW);
    digitalWrite(valveSevenPin, HIGH);

    startup = false;
  }

  if (overallButtonOn) {
    if (everythingElseButtonOn) {
      // ----- Everything Else Button STATE -----
      // Pump 1 OFF (recirc)
      // Pump 2 ON
      // Overall and Everything else LED ON
      // Recirc and RecircFlush LED OFF
      // Valve 1, 4, 6 ON (water through)
      // Valve 2, 3, 5, 7 OFF (water blocked)

      // Checking if this is the first time it's going through this state
      if (currentState != lastState) {
        // Last valve change starting now
        lastValveChange = millis();
        Serial.println("Toggling everything else ARITHMETIC");
      }

      // Pumps
      digitalWrite(pumpOnePin, HIGH);
      digitalWrite(pumpTwoPin, HIGH);

      // LEDs
      digitalWrite(overallLEDPin, LOW);
      digitalWrite(everythingElseLEDPin, LOW);
      digitalWrite(recircLEDPin, HIGH);
      digitalWrite(recircFlushLEDPin, HIGH);

      // Valves
      digitalWrite(valveOnePin, LOW);
      digitalWrite(valveTwoPin, HIGH);
      digitalWrite(valveThreePin, HIGH);
      digitalWrite(valveFourPin, LOW);
      digitalWrite(valveFivePin, HIGH);
      digitalWrite(valveSixPin, LOW);
      digitalWrite(valveSevenPin, HIGH);

      // Wait 20 seconds for the valves to finish changing until the pump can be
      // turned back on
      if (millis() - lastValveChange > 20000) {
        digitalWrite(pumpTwoPin, LOW);
      }
    } else if (recircButtonOn) {
      // ----- Recirc Button STATE -----
      // Pump 1 ON (recirc)
      // Pump 2 OFF
      // Recirc LED ON
      // Recirc and RecircFlush LED OFF
      // Valve 2, 3, 6 ON (water through)
      // Valve 1, 4, 5, 7 OFF (water blocked)

      // Checking if this is the first time it's going through this state
      if (currentState != lastState) {
        // Last valve change starting now
        lastValveChange = millis();
        Serial.println("Toggling recirculating shower ARITHMETIC");
      }

      // Turning both pumps off when changing state
      digitalWrite(pumpOnePin, HIGH);
      digitalWrite(pumpTwoPin, HIGH);

      // LEDs
      digitalWrite(overallLEDPin, LOW);
      digitalWrite(everythingElseLEDPin, HIGH);
      digitalWrite(recircLEDPin, LOW);
      digitalWrite(recircFlushLEDPin, HIGH);

      // Valves
      digitalWrite(valveOnePin, HIGH);
      digitalWrite(valveTwoPin, LOW);
      digitalWrite(valveThreePin, LOW);
      digitalWrite(valveFourPin, HIGH);
      digitalWrite(valveFivePin, HIGH);
      digitalWrite(valveSixPin, LOW);
      digitalWrite(valveSevenPin, HIGH);

      // Wait 20 seconds for the valves to finish changing until the pump can be
      // turned back on
      if (millis() - lastValveChange > 20000) {
        digitalWrite(pumpOnePin, LOW);
        digitalWrite(pumpTwoPin, LOW);
      }
    } else if (recircFlushButtonOn) {

      // ----- Recirc Flush Button STATE -----
      // Pump 1 ON (recirc)
      // Pump 2 ON
      // Recirc LED OFF
      // RecircFlush LED ON (Blinking), when the flush is done, the LED will be
      // ON indefinitely Valve 1, 4, 6 OFF (water blocked) Valve 2, 3, 5, 7 ON
      // (water through)

      // Checking if this is the first time it's going through this state
      if (currentState != lastState) {
        // Last valve change starting now
        lastValveChange = millis();
        previousFlushMillis = millis();
        Serial.println(
            "Toggling recirculating shower flush in progress ARITHMETIC");
      }

      // Turning both pumps off when changing state
      digitalWrite(pumpOnePin, HIGH);
      digitalWrite(pumpTwoPin, HIGH);

      // LEDs
      digitalWrite(overallLEDPin, LOW);
      digitalWrite(everythingElseLEDPin, HIGH);
      digitalWrite(recircLEDPin, HIGH);

      // Check if its currently flushing and set a blinking LED if it is, and a
      // solid LED if its done.
      if (millis() - previousFlushMillis <= (flushDuration + valveDuration)) {
        currentBlinkMillis = millis();
        if (currentBlinkMillis - previousBlinkMillis >= blinkDuration) {
          // save the last time you blinked the LED
          previousBlinkMillis = currentBlinkMillis;

          // if the LED is off turn it on and vice-versa:
          if (ledState == LOW) {
            ledState = HIGH;
          } else {
            ledState = LOW;
          }

          // set the LED with the ledState of the variable:
          digitalWrite(recircFlushLEDPin, ledState);
        }
      } else {
        Serial.println("Recirculating shower done flushing");
        // Turning both pumps off when changing state
        digitalWrite(pumpOnePin, HIGH);
        digitalWrite(pumpTwoPin, HIGH);

        // LEDs
        digitalWrite(overallLEDPin, LOW);
        digitalWrite(everythingElseLEDPin, HIGH);
        digitalWrite(recircLEDPin, HIGH);
        digitalWrite(recircFlushButtonPin, LOW);
      }

      // Valves
      digitalWrite(valveOnePin, HIGH);
      digitalWrite(valveTwoPin, LOW);
      digitalWrite(valveThreePin, LOW);
      digitalWrite(valveFourPin, HIGH);
      digitalWrite(valveFivePin, LOW);
      digitalWrite(valveSixPin, LOW);
      digitalWrite(valveSevenPin, LOW);

      // Wait 20 seconds for the valves to finish changing until the pump can be
      // turned back on
      if (millis() - lastValveChange >= valveDuration) {
        digitalWrite(pumpOnePin, LOW);
        digitalWrite(pumpTwoPin, LOW);
      }
    }
  } else // (EVERYTHING OFF)
  {
    // ----- Overall Button STATES -----
    // Turns off all LEDs
    // Turns off both pumps
    // Leaves valve state alone

    // Turning both pumps off when changing state
    digitalWrite(pumpOnePin, HIGH);
    digitalWrite(pumpTwoPin, HIGH);

    // LEDs
    digitalWrite(overallLEDPin, HIGH);
    digitalWrite(everythingElseLEDPin, HIGH);
    digitalWrite(recircLEDPin, HIGH);
    digitalWrite(recircFlushLEDPin, HIGH);
  }

  lastState = currentState;
}