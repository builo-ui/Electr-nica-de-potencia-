#define BLYNK_TEMPLATE_ID   "TMPL24Zjy4aUe"
#define BLYNK_TEMPLATE_NAME "Motor Estrella Delta"
#define BLYNK_AUTH_TOKEN    "nyr113AaeQAbPmUSqnG2JBMQrLVetwVp"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// WIFI
char ssid[] = "SM-A356E/DS";
char pass[] = "YeimerYMS";

// PINES
#define PIN_K1 25
#define PIN_K2 26
#define PIN_K3 27

#define PIN_ACS 34

// VIRTUAL PINS

#define VPIN_START       V0
#define VPIN_STOP        V1
#define VPIN_EMERGENCY   V2
#define VPIN_CURRENT     V3
#define VPIN_K1          V4
#define VPIN_K2          V5
#define VPIN_K3          V6
#define VPIN_RAMP        V7
#define VPIN_MOTOR       V8
#define VPIN_ESP32       V9

// TIEMPO MUERTO

const unsigned long DEAD_TIME_MS = 100;
// ACS712 20 A

const float ACS_SENSITIVITY = 0.100;


// DIVISOR

const float DIVIDER_FACTOR = 10.0 / 30.0;

// MUESTREO

const int CURRENT_SAMPLES = 1000;

const unsigned long CURRENT_SAMPLE_INTERVAL_US = 200;
// VARIABLES ACS712

float acsZeroVoltage = 1.65;

float currentSumSquares = 0.0;

int currentSampleCount = 0;

unsigned long lastCurrentSampleMicros = 0;

float currentRMS = 0.0;


// ESTADOS

enum MotorState
{
  MOTOR_PARADO,
  MOTOR_ESTRELLA,
  MOTOR_TIEMPO_MUERTO,
  MOTOR_DELTA,
  MOTOR_EMERGENCIA
};

MotorState estadoMotor = MOTOR_PARADO;


// VARIABLES

unsigned long tiempoInicioEstado = 0;

float tiempoRampa = 2.0;

bool emergencia = false;


// TIMER BLYNK

BlynkTimer timer;


// NOMBRE DEL ESTADO

String nombreEstado()
{
  switch (estadoMotor)
  {
    case MOTOR_PARADO:
      return "PARADO";

    case MOTOR_ESTRELLA:
      return "ESTRELLA";

    case MOTOR_TIEMPO_MUERTO:
      return "TIEMPO MUERTO";

    case MOTOR_DELTA:
      return "DELTA";

    case MOTOR_EMERGENCIA:
      return "EMERGENCIA";

    default:
      return "DESCONOCIDO";
  }
}


// ACTUALIZAR INDICADORES

void actualizarIndicadores()
{
  if (!Blynk.connected())
    return;

  Blynk.virtualWrite(
    VPIN_K1,
    digitalRead(PIN_K1)
  );

  Blynk.virtualWrite(
    VPIN_K2,
    digitalRead(PIN_K2)
  );

  Blynk.virtualWrite(
    VPIN_K3,
    digitalRead(PIN_K3)
  );

  Blynk.virtualWrite(
    VPIN_MOTOR,
    nombreEstado()
  );
}


// APAGAR MOTOR

void apagarMotor()
{
  digitalWrite(PIN_K1, LOW);
  digitalWrite(PIN_K2, LOW);
  digitalWrite(PIN_K3, LOW);
}


// ESTRELLA

void activarEstrella()
{
  // Primero K2 OFF
  digitalWrite(PIN_K2, LOW);

  // K1 ON
  digitalWrite(PIN_K1, HIGH);

  // K3 ON
  digitalWrite(PIN_K3, HIGH);
}


// DELTA

void activarDelta()
{
  // K3 OFF
  digitalWrite(PIN_K3, LOW);

  // K1 ON
  digitalWrite(PIN_K1, HIGH);

  // K2 ON
  digitalWrite(PIN_K2, HIGH);
}


// START

BLYNK_WRITE(VPIN_START)
{
  int valor = param.asInt();

  if (valor != 1)
    return;


  // EMERGENCIA ACTIVA

  if (emergencia)
  {
    Blynk.virtualWrite(
      VPIN_START,
      0
    );

    return;
  }

  // SOLO ARRANCA DESDE PARADO

  if (estadoMotor != MOTOR_PARADO)
  {
    Blynk.virtualWrite(
      VPIN_START,
      0
    );

    return;
  }

  // ARRANQUE EN ESTRELLA
  activarEstrella();
  estadoMotor = MOTOR_ESTRELLA;
  tiempoInicioEstado = millis();
  actualizarIndicadores();
}


// STOP
BLYNK_WRITE(VPIN_STOP)
{
  int valor = param.asInt();
  if (valor != 1)
    return;
  apagarMotor();
  estadoMotor = MOTOR_PARADO;
  tiempoInicioEstado = millis();
  Blynk.virtualWrite(
    VPIN_START,
    0
  );

  Blynk.virtualWrite(
    VPIN_STOP,
    0
  );

  actualizarIndicadores();
}


// EMERGENCIA
BLYNK_WRITE(VPIN_EMERGENCY)
{
  int valor = param.asInt();
  // EMERGENCIA ACTIVADA
  if (valor == 1)
  {
    emergencia = true;
    apagarMotor();
    estadoMotor = MOTOR_EMERGENCIA;
    tiempoInicioEstado = millis();
    Blynk.virtualWrite(
      VPIN_START,
      0
    );

    actualizarIndicadores();

    return;
  }


  // EMERGENCIA LIBERADA

  emergencia = false;
  apagarMotor();
  estadoMotor = MOTOR_PARADO;
  tiempoInicioEstado = millis();
  Blynk.virtualWrite(
    VPIN_START,
    0
  );
  Blynk.virtualWrite(
    VPIN_EMERGENCY,
    0
  );

  actualizarIndicadores();
}

// TIEMPO DE RAMPA
BLYNK_WRITE(VPIN_RAMP)
{
  tiempoRampa = param.asFloat();
  if (tiempoRampa < 2.0)
    tiempoRampa = 2.0;

  if (tiempoRampa > 10.0)
    tiempoRampa = 10.0;
}


// MAQUINA DE ESTADOS
void ejecutarFSM()
{
  unsigned long ahora = millis();

  switch (estadoMotor)
  {

    // PARADO
    case MOTOR_PARADO:
      digitalWrite(PIN_K1, LOW);
      digitalWrite(PIN_K2, LOW);
      digitalWrite(PIN_K3, LOW);
      break;

    // ESTRELLA
    case MOTOR_ESTRELLA:
      digitalWrite(PIN_K1, HIGH);
      digitalWrite(PIN_K2, LOW);
      digitalWrite(PIN_K3, HIGH);

      if (
        ahora - tiempoInicioEstado >=
        (unsigned long)(tiempoRampa * 1000.0)
      )
      {

        // K1 permanece ON
        digitalWrite(PIN_K1, HIGH);
        // K3 OFF
        digitalWrite(PIN_K3, LOW);
        // K2 OFF
        digitalWrite(PIN_K2, LOW);
        estadoMotor = MOTOR_TIEMPO_MUERTO;
        tiempoInicioEstado = ahora;

        actualizarIndicadores();
      }

      break;


    // TIEMPO MUERTO
    case MOTOR_TIEMPO_MUERTO:
      digitalWrite(PIN_K1, HIGH);
      digitalWrite(PIN_K2, LOW);
      digitalWrite(PIN_K3, LOW);
      if (
        ahora - tiempoInicioEstado >=
        DEAD_TIME_MS
      )
      {

        activarDelta();

        estadoMotor = MOTOR_DELTA;
        tiempoInicioEstado = ahora;

        actualizarIndicadores();
      }

      break;
    // DELTA
    case MOTOR_DELTA:
      digitalWrite(PIN_K1, HIGH);
      digitalWrite(PIN_K2, HIGH);
      digitalWrite(PIN_K3, LOW);
      break;
    // EMERGENCIA
    case MOTOR_EMERGENCIA:


      digitalWrite(PIN_K1, LOW);
      digitalWrite(PIN_K2, LOW);
      digitalWrite(PIN_K3, LOW);
      break;
  }


  // PROTECCION K2 / K3

  if (
    digitalRead(PIN_K2) == HIGH &&
    digitalRead(PIN_K3) == HIGH
  )
  {

    digitalWrite(PIN_K2, LOW);
    digitalWrite(PIN_K3, LOW);
    digitalWrite(PIN_K1, HIGH);
    estadoMotor = MOTOR_ESTRELLA;
    tiempoInicioEstado = millis();
    actualizarIndicadores();
  }
}


// ACS712

void procesarACS712()
{
  unsigned long ahoraMicros = micros();

  if (
    ahoraMicros - lastCurrentSampleMicros <
    CURRENT_SAMPLE_INTERVAL_US
  )
  {
    return;
  }

  lastCurrentSampleMicros = ahoraMicros;

  int adc = analogRead(PIN_ACS);

  float voltageADC =
    ((float)adc / 4095.0) * 3.3;

  float voltageACS =
    voltageADC / DIVIDER_FACTOR;

  float acVoltage =
    voltageACS - acsZeroVoltage;

  currentSumSquares +=
    acVoltage * acVoltage;
  currentSampleCount++;

  if (
    currentSampleCount >= CURRENT_SAMPLES
  )
  {

    float voltageRMS =
      sqrt(
        currentSumSquares /
        CURRENT_SAMPLES
      );

    currentRMS =
      voltageRMS /
      ACS_SENSITIVITY;

    if (currentRMS < 0.05)
      currentRMS = 0.0;
    currentSumSquares = 0.0;
    currentSampleCount = 0;
  }
}

// ENVIAR CORRIENTE
void enviarCorriente()
{
  if (Blynk.connected())
  {
    Blynk.virtualWrite(
      VPIN_CURRENT,
      currentRMS
    );
  }
}


// ESTADO ESP32

void enviarEstadoESP32()
{
  if (
    WiFi.status() == WL_CONNECTED &&
    Blynk.connected()
  )
  {
    Blynk.virtualWrite(
      VPIN_ESP32,
      "ONLINE"
    );
  }
}


// VERIFICAR CONEXION
void verificarConexion()
{
  
  if (
    WiFi.status() == WL_CONNECTED &&
    Blynk.connected()
  )
  {
  }
  else
  {
  }
}


// CALIBRACION ACS712

void calibrarACS712()
{
  delay(2000);

  const int muestras = 1000;

  long sumaADC = 0;
  for (
    int i = 0;
    i < muestras;
    i++
  )
  {
    sumaADC += analogRead(PIN_ACS);

    delayMicroseconds(500);
  }

  float promedioADC =
    (float)sumaADC / muestras;


  float voltageADC =
    (promedioADC / 4095.0) * 3.3;
  acsZeroVoltage =
    voltageADC / DIVIDER_FACTOR;
}


// BLYNK CONECTADO

BLYNK_CONNECTED()
{
  
  Blynk.virtualWrite(
    VPIN_K1,
    digitalRead(PIN_K1)
  );

  Blynk.virtualWrite(
    VPIN_K2,
    digitalRead(PIN_K2)
  );

  Blynk.virtualWrite(
    VPIN_K3,
    digitalRead(PIN_K3)
  );

  Blynk.virtualWrite(
    VPIN_MOTOR,
    nombreEstado()
  );

  Blynk.virtualWrite(
    VPIN_ESP32,
    "ONLINE"
  );
}

// SETUP

void setup()
{
  pinMode(PIN_K1, OUTPUT);
  pinMode(PIN_K2, OUTPUT);
  pinMode(PIN_K3, OUTPUT);

  // ARRANQUE INICIAL SEGURO
  digitalWrite(PIN_K1, LOW);
  digitalWrite(PIN_K2, LOW);
  digitalWrite(PIN_K3, LOW);
  // ACS712
  pinMode(PIN_ACS, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(
    PIN_ACS,
    ADC_11db
  );
  calibrarACS712();
  // BLYNK

  Blynk.begin(
    BLYNK_AUTH_TOKEN,
    ssid,
    pass
  );


  // TIMERS
  timer.setInterval(
    1000L,
    enviarCorriente
  );

  timer.setInterval(
    1000L,
    enviarEstadoESP32
  );

  timer.setInterval(
    1000L,
    verificarConexion
  );

  // ESTADO INICIAL
  estadoMotor = MOTOR_PARADO;
  tiempoInicioEstado = millis();
  emergencia = false;
  currentRMS = 0.0;
}


// LOOP

void loop()
{
  Blynk.run();

  timer.run();

  ejecutarFSM();

  procesarACS712();
}