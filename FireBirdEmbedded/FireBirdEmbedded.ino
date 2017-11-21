#include <MsTimer2.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <ArduinoJson.h>

// Json

StaticJsonBuffer<200> jsonBuffer;
JsonObject& measurementPacket = jsonBuffer.createObject();

// Sensores

const float maxVoltage = 5.0;

// -- Sensores analógicos

const float sensorAnalogMin = 0;
const float sensorAnalogMax = 1023;

const int sensorTemperature = A0;
const int sensorSmoke = A1;
const int sensorFlame = A2;
const int sensorFoodEat = A3;
const int sensorFoodDepositLow = A4;
const int sensorFoodDepositHigh = A5;

// -- Sensores digitales

const int sensorWaterDeposit = 7;
const int sensorWaterDrink = 10;

// -- Constantes

const int differenceTemperatureLimit = 3;
const int sensorSmokeLimit = 120;
const int sensorFlameLimit = 600;
const int sensorFoodLimit = 100;
const int sensorFoodDepositLowLimit = 100;
const int sensorFoodDepositHighLimit = 100;

// Actuadores

const int actuatorServo = 6;

const int actuatorBuzzer = 9;
const int actuatorWaterPump = 8;
const int actuatorLed = 9;

// -- Constantes

const int servoClosedPosition = 80;
const int servoOpenedPosition = 50;

// Variables

// -- Mediciones

int measurementFlame;
int measurementSmoke;
float measurementTemperature;
int measurementFoodEat;
int measurementFoodDepositLow;
int measurementFoodDepositHigh;

float lastTemperature = 0;

bool thereIsSmoke;
bool thereIsFlame;
bool temperatureChanged;

// -- Simular el Timer

int timeCounter = 0;

// Componentes

// -- Bluetooth

SoftwareSerial btSerial(3, 2);

const int requestMeasurements = 50;
const int requestLed = 11;
const int requestBuzzer = 22;
const int requestWaterPump = 33;
const int noAction = 99;

int btCode;
bool doingAction = false;
int currentAction;
int timeActionCounter;

const int durationLed = 5;
const int durationBuzzer = 5;
const int durationWaterPump = 1;

// -- Servo Motor

Servo servo;

// Setup

void setup() {

  // Setear los actuadores como pines de salida
  pinMode(actuatorServo, OUTPUT);
  pinMode(actuatorBuzzer, OUTPUT);
  pinMode(actuatorWaterPump, OUTPUT);
  pinMode(actuatorLed, OUTPUT);


  // Configurar el Timer 2 cada una décima de segundo
  MsTimer2::set(100, checkMeasurements);
  MsTimer2::start();

  // Atachar el componente servo al pin correspondiente
  servo.attach(actuatorServo);

  // Habilitar las interrupciones
  interrupts();

  // Configurar la velocidad de transmisión
  btSerial.begin(9600);
  Serial.begin(9600);

}

// Loop

void loop() {

  // Tomar las mediciones
  measurementFlame = analogRead(sensorFlame);
  measurementSmoke = analogRead(sensorSmoke);
  measurementTemperature = analogRead(sensorTemperature) * (maxVoltage / sensorAnalogMax) * 100;
  measurementFoodEat = analogRead(sensorFoodEat);
  measurementFoodDepositLow = analogRead(sensorFoodDepositLow);
  measurementFoodDepositHigh = analogRead(sensorFoodDepositHigh);

  // Chequear las mediciones
  thereIsSmoke = measurementSmoke >= sensorSmokeLimit;
  thereIsFlame = measurementFlame <= sensorFlameLimit;
  temperatureChanged = measurementTemperature - lastTemperature >= differenceTemperatureLimit;

  // Comprobar si hay fuego
  if (thereIsFlame && thereIsSmoke && temperatureChanged) {
    // Bombear agua, encender led y buzzer
    digitalWrite(actuatorWaterPump, HIGH);
    digitalWrite(actuatorLed, HIGH);
    //tone(actuatorBuzzer, 1000);
  }

}

// Funciones extras

void checkMeasurements() {

  // Aumento en 1 el contador de décimas de segundos
  timeCounter++;

  // Cada 5 segundos actualizar la última temperatura
  if (timeCounter % 50 == 0) {
    lastTemperature = measurementTemperature;
  }

  // Cada 1 segundo verificar el nivel de comida
  if (timeCounter % 10 == 0) {
    if (measurementFoodEat < sensorFoodLimit) {
      servo.write(servoClosedPosition);
    } else {
      servo.write(servoOpenedPosition);
    }
  }

  // Cada 2 décimas de segundo verificar si hay solicitudes por Bluetooth
  if (timeCounter % 2 == 0 && btSerial.available()) {

    // Leer el codigo y realizar la acción necesaria
    btCode = Serial.println(btSerial.read());

    switch (btCode) {
      case requestMeasurements:
        measurementPacket["temperature"] = measurementTemperature;
        measurementPacket["smoke"] = thereIsSmoke ? "Detectado" : "No Detectado";
        measurementPacket["flame"] = thereIsFlame ? "Detectado" : "No Detectado";
        measurementPacket["food"] = analogRead(sensorFoodDepositHigh);
        measurementPacket["water"] = digitalRead(sensorWaterDrink);
        measurementPacket["tankWater"] = digitalRead(sensorWaterDeposit);
        measurementPacket.printTo(btSerial);
        break;
      case requestLed:
        if (!doingAction) {
          digitalWrite(actuatorLed, HIGH);
          doingAction = true;
          currentAction = requestLed;
        }
        break;
      case requestBuzzer:
        if (!doingAction) {
          tone(actuatorBuzzer, 1000);
          doingAction = true;
          currentAction = requestBuzzer;
        }
        break;
      case requestWaterPump:
        if (!doingAction) {
          digitalWrite(actuatorWaterPump, HIGH);
          doingAction = true;
          currentAction = requestWaterPump;
        }
        break;
    }

  }

  // Comprobar si se está realizando una acción
  if (doingAction) {
    timeActionCounter++;
    // Cancelar la acción después del tiempo predeterminado
    switch (currentAction) {
requestLed:
        if (timeActionCounter >= durationLed) {
          digitalWrite(actuatorLed, LOW);
          doingAction = false;
        }
        break;
requestBuzzer:
        if (timeActionCounter >= durationBuzzer) {
          noTone(actuatorBuzzer);
          doingAction = false;
        }
        break;
requestWaterPump:
        if (timeActionCounter >= durationWaterPump) {
          digitalWrite(actuatorWaterPump, LOW);
          doingAction = false;
        }
        break;
    }
    // Resetear la acción actual
    if (!doingAction) {
      timeActionCounter = 0;
      currentAction = noAction;
    }
  }


  // Resetear cada 5 segundos
  if (timeCounter == 50) {
    timeCounter = 0;
  }

}
