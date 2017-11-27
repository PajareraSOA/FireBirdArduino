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

const int actuatorBuzzer = 11;
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
int measurementWaterDrink;
int measurementWaterDeposit;

float lastTemperature = 0;

bool thereIsSmoke;
bool thereIsFlame;
bool temperatureChanged;
bool isFoodDepositLow;
bool isFoodDepositHigh;
bool thereIsWaterDrink;
bool thereIsWaterDeposit;

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
int timeActionCounter = 0;

const int durationLed = 3;
const int durationBuzzer = 1;
const int durationWaterPump = 1;

// -- Servo Motor

Servo servo;

// Detección de incendio

int timeFireCounter = 0;

bool fireDetected = false;

bool isBuzzerAndLedEnabled = false;
bool isWaterPumpEnabled = false;

const int durationBuzzerAndLed = 5;
const int durationWaterPumpOnFire = 5;

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
  measurementWaterDrink = digitalRead(sensorWaterDrink);
  measurementWaterDeposit = digitalRead(sensorWaterDeposit);

  // Chequear las mediciones
  thereIsSmoke = measurementSmoke >= sensorSmokeLimit;
  thereIsFlame = measurementFlame <= sensorFlameLimit;
  temperatureChanged = measurementTemperature - lastTemperature >= differenceTemperatureLimit;

  isFoodDepositHigh = measurementFoodDepositHigh >= sensorFoodDepositHighLimit;
  isFoodDepositLow = measurementFoodDepositLow >= sensorFoodDepositLowLimit;
  thereIsWaterDrink = measurementWaterDrink == 0;
  thereIsWaterDeposit = measurementWaterDeposit == 0;

  // Comprobar si hay fuego
  if (thereIsFlame && thereIsSmoke && temperatureChanged && !doingAction) {

    if (!fireDetected) {

      fireDetected = true;

      timeFireCounter = 0;
      digitalWrite(actuatorWaterPump, LOW);
      digitalWrite(actuatorLed, HIGH);
      digitalWrite(actuatorBuzzer, HIGH);
      isBuzzerAndLedEnabled = true;



    }

      

    //delay(5000);


  }

  if(fireDetected && !doingAction){
    if (timeFireCounter >= durationBuzzerAndLed * 10 && isBuzzerAndLedEnabled) {
        digitalWrite(actuatorLed, LOW);
        digitalWrite(actuatorBuzzer, LOW);
        digitalWrite(actuatorWaterPump, HIGH);
        isBuzzerAndLedEnabled = false;
        isWaterPumpEnabled = true;
        timeFireCounter = 0;
      } else if (timeFireCounter >= durationWaterPumpOnFire * 10 && isWaterPumpEnabled) {
//        digitalWrite(actuatorLed, LOW);
//        digitalWrite(actuatorBuzzer, LOW);
        digitalWrite(actuatorWaterPump, LOW);
        timeFireCounter = 0;
        fireDetected = false;
      }

  }
  //else {
  //
  //    // Apagar todo
  //    if (fireDetected) {
  //      digitalWrite(actuatorLed, LOW);
  //      digitalWrite(actuatorBuzzer, LOW);
  //      digitalWrite(actuatorWaterPump, LOW);
  //    }
  //
  //    fireDetected = false;
  //    timeFireCounter = 0;
  //  }

}

// Funciones extras

void checkMeasurements() {

  // Aumento en 1 el contador de décimas de segundos
  timeCounter++;
  timeFireCounter++;

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
  if (timeCounter % 2 == 0 && btSerial.available() > 0) {

    // Leer el codigo y realizar la acción necesaria
    btCode = btSerial.read();

    switch (btCode) {
      case requestMeasurements:
        measurementPacket["temperature"] = measurementTemperature - 10;
        measurementPacket["smoke"] = measurementSmoke;//thereIsSmoke ? "Detectado" : "No Detectado";
        measurementPacket["flame"] = measurementFlame;//thereIsFlame ? "Detectado" : "No Detectado";
        measurementPacket["food"] = isFoodDepositHigh ? "Alto" : isFoodDepositLow ? "Medio" : "Bajo";
        measurementPacket["water"] = thereIsWaterDrink ? "Alto" : "Bajo";
        measurementPacket["tankWater"] = thereIsWaterDeposit ? "Alto" : "Bajo";
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
          digitalWrite(actuatorBuzzer, HIGH);
          //tone(actuatorBuzzer, 1000);
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
      case requestLed:
        if (timeActionCounter >= durationLed * 10) {
          digitalWrite(actuatorLed, LOW);
          doingAction = false;
        }
        break;
      case requestBuzzer:
        if (timeActionCounter >= durationBuzzer * 10) {
          digitalWrite(actuatorBuzzer, LOW);
          doingAction = false;
        }
        break;
      case requestWaterPump:
        if (timeActionCounter >= durationWaterPump * 10) {
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

  Serial.println("sigo lupeando");
  // Resetear cada 5 segundos
  if (timeCounter == 50) {
    timeCounter = 0;
  }

}
