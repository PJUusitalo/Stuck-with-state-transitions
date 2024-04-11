#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define DHTPIN 2
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

const int numReadings = 60;
float humidityReadings[numReadings];
int index = 0;

const int buttonPin = 7;
int buttonState = 0;
int lastButtonState = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 20;

const int motorPin = 9;
bool motorActive = false;

LiquidCrystal_I2C lcd(0x27, 16, 2);

enum State {
  IDLE,
  MOTOR_START_10,
  MOTOR_START_50,
  SHUTDOWN,
};

State currentState = IDLE;
unsigned long motorStartTime = 0;
unsigned long lastMotorToggleTime = 0;
float meanHumidity = 0;
volatile unsigned int pulseCount = 0;
unsigned long lastPulseTime = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("DHT22 Humidity and Temperature Sensor");

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(motorPin, OUTPUT);

  dht.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();

  attachInterrupt(digitalPinToInterrupt(3), handlePulse, FALLING); // Assuming pulse input is connected to digital pin 3

  meanHumidity = 30;
}

void loop() {
  Serial.println("Entering loop...");

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  humidityReadings[index] = humidity;
  index = (index + 1) % numReadings;

  if (index == 0) {
    meanHumidity = calculateMeanHumidity(); 
    Serial.print("Mean Humidity (excluding last 3 hours): ");
    Serial.print(meanHumidity);
    Serial.println("%");
  }

  displayHumidityTemperature(humidity, temperature);

  Serial.println("Delaying for 1 second...");
  Serial.println(currentState);
  delay(1000);

  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

 if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == HIGH) {
        Serial.println("Button pressed!");

        // Handle button press based on current state
        switch (currentState) {
          case IDLE:
            currentState = MOTOR_START_10;
            motorStartTime = millis();
            Serial.println("Ajastus");
            break;

          case MOTOR_START_10:
            currentState = MOTOR_START_50; // Siirtyy suoraan tehostustilaan
            motorStartTime = millis();
            Serial.println("Tehostus");
            break;

          case MOTOR_START_50:
          case SHUTDOWN:
            currentState = SHUTDOWN; // Sammutetaan ohjelma
            Serial.println("Sammutus");
            break;
        }
      }
    }
  }

  // Handle state transitions
  switch (currentState) {
    case MOTOR_START_10:
      analogWrite(motorPin, 55); // 10% duty cycle, about 1V
      if (millis() - motorStartTime >= 6 * 3600 * 1000) { // 6 hours in milliseconds
      currentState = SHUTDOWN; // If the motor has been running for 6 hours, shut down
      }
      break;

    case MOTOR_START_50:
      analogWrite(motorPin, 140); // 25% duty cycle, about 2,5V 
      float currentHumidity = dht.readHumidity();
      if (currentHumidity <= (meanHumidity - 5)) { // If current humidity is at or below above mean humidity, shut down
      currentState = SHUTDOWN; 
      }
      break;

    case SHUTDOWN:
      // Perform any necessary shutdown tasks
      // For example, saving data to memory, closing connections, etc.
      // Then, turn off the motor and LCD
      analogWrite(motorPin, 0);
      Serial.println("idle...?");
      lcd.clear();
      currentState = IDLE;
      break;

    case IDLE:
      // Perform any tasks in the idle state if needed
      break;
  }

  lastButtonState = reading;

  Serial.println("Exiting loop...");
}

float calculateMeanHumidity() {
  float sum = 0;

  // Calculate the sum of all humidity readings
  for (int i = 0; i < numReadings; i++) {
    sum += humidityReadings[i];
  }

  // Calculate the mean humidity over the entire period
  float meanHumidity = -1; // Alustetaan keskiarvo -1:llä

  if (numReadings > 0) {
    meanHumidity = sum / numReadings;

    // Exclude the latest 3 hours of readings
    int startIndexToExclude = (index - 3 * 60 + numReadings) % numReadings; // Calculate the index of the first reading to exclude
    for (int i = index; i != startIndexToExclude; i = (i + 1) % numReadings) {
      meanHumidity -= humidityReadings[i] / numReadings;
    }
  }

  return meanHumidity;
}



void displayHumidityTemperature(float humidity, float temperature) {
  lcd.clear();
  lcd.setCursor(0, 0);

  // Display humidity on LCD
  char humidityStr[3];
  dtostrf(humidity, 3, 0, humidityStr);
  lcd.print("RH:");
  lcd.print(humidityStr);
  lcd.print("%");

  // Display temperature on LCD
  lcd.setCursor(8, 0);
  char temperatureStr[4];
  dtostrf(temperature, 4, 1, temperatureStr);
  lcd.print("T:");
  lcd.print(temperatureStr);
  lcd.print("C");

  // Display fan RPM on lower row
  lcd.setCursor(0, 1);
  lcd.print("RPM:");
  lcd.print(getFanRPM(), 0); // Display RPM as integer

  // Display mode on lower right corner
  lcd.setCursor(8, 1);
  switch (currentState) {
    case MOTOR_START_10:
      lcd.print("AJASTUS");
      break;
    case MOTOR_START_50:
      lcd.print("TEHOSTUS");
      break;
    default:
      break;
  }
}

void handlePulse() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastPulseTime;

  // Check if the pulse width is valid (to avoid erroneous pulses)
  if (pulseWidth > 1000 && pulseWidth < 50000) {
    pulseCount++;
  }

  lastPulseTime = currentTime;
}

float getFanRPM() {
  // Assuming pulseCount increments for every revolution
  // Calculate RPM based on the time between pulses
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastMotorToggleTime;

  if (elapsedTime >= 60000) { // Update RPM every minute
    float rpm = (float)pulseCount / (elapsedTime / 60000.0);
    pulseCount = 0;
    lastMotorToggleTime = currentTime;
    return rpm;
  }

  return 0; // Return 0 if RPM calculation interval hasn't passed yet
}
