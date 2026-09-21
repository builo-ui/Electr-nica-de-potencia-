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

// DIVISOR DE VOLTAJE
// Usa 10.0 / 30.0 si la de 1k va a GND y la de 2k va al sensor (Factor 1/3)
// Usa 2.0 / 3.0 si la de 2k va a GND y la de 1k va al sensor (Factor 2/3)
const float DIVIDER_FACTOR = 2 / 3.0; 

// MUESTREO
const int CURRENT_SAMPLES = 1000;
const unsigned long CURRENT_SAMPLE_INTERVAL_US = 200;

// ==========================================
// CONFIGURACIÓN PROTECCIÓN TÉRMICA
// ==========================================
const float MAX_CURRENT_NOMINAL = 3.5;               // Corriente máxima de trabajo continuo (Ajustable)
const unsigned long OVERCURRENT_TRIP_TIME_MS = 1500; // Tiempo de tolerancia al atasco (1.5 seg)
const unsigned long INRUSH_GRACE_TIME_MS = 2000;     // Tiempo ciego para ignorar picos de arranque (2 seg)

bool overcurrentActive = false;
unsigned long overcurrentStartTime = 0;

// VARIABLES ACS712
float acsZeroVoltage = 1.65;
float currentSumSquares = 0.0;
int currentSampleCount = 0;
unsigned long lastCurrentSampleMicros = 0;
float currentRMS = 0.0;
float ultimaCorrienteEnviada = -1.0; // Variable para optimización de Blynk

// ESTADOS
enum MotorState
{
  MOTOR_PARADO,
  MOTOR_ESTRELLA,
  MOTOR_TIEMPO_MUERTO,
  MOTOR_DELTA,
  MOTOR_EMERGENCIA,
  MOTOR_FALLA_TERMICA 
};

MotorState estadoMotor = MOTOR_PARADO;

// VARIABLES GENERALES
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
    case MOTOR_PARADO: return "PARADO";
    case MOTOR_ESTRELLA: return "ESTRELLA";
    case MOTOR_TIEMPO_MUERTO: return "TIEMPO MUERTO";
    case MOTOR_DELTA: return "DELTA";
    case MOTOR_EMERGENCIA: return "EMERGENCIA";
    case MOTOR_FALLA_TERMICA: return "FALLA TERMICA";
    default: return "DESCONOCIDO";
  }
}

// ACTUALIZAR INDICADORES
void actualizarIndicadores()
{
  if (!Blynk.connected()) return;

  Blynk.virtualWrite(VPIN_K1, digitalRead(PIN_K1));
  Blynk.virtualWrite(VPIN_K2, digitalRead(PIN_K2));
  Blynk.virtualWrite(VPIN_K3, digitalRead(PIN_K3));
  Blynk.virtualWrite(VPIN_MOTOR, nombreEstado());
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
  digitalWrite(PIN_K2, LOW);
  digitalWrite(PIN_K1, HIGH);
  digitalWrite(PIN_K3, HIGH);
}

// DELTA
void activarDelta()
{
  digitalWrite(PIN_K3, LOW);
  digitalWrite(PIN_K1, HIGH);
  digitalWrite(PIN_K2, HIGH);
}

// START
BLYNK_WRITE(VPIN_START)
{
  int valor = param.asInt();
  if (valor != 1) return;

  if (emergencia || estadoMotor != MOTOR_PARADO)
  {
    Blynk.virtualWrite(VPIN_START, 0);
    return;
  }

  activarEstrella();
  estadoMotor = MOTOR_ESTRELLA;
  tiempoInicioEstado = millis();
  overcurrentActive = false;
  actualizarIndicadores();
}

// STOP
BLYNK_WRITE(VPIN_STOP)
{
  int valor = param.asInt();
  if (valor != 1) return;
  
  apagarMotor();
  estadoMotor = MOTOR_PARADO;
  tiempoInicioEstado = millis();
  overcurrentActive = false; 
  
  Blynk.virtualWrite(VPIN_START, 0);
  Blynk.virtualWrite(VPIN_STOP, 0);
  actualizarIndicadores();
}

// EMERGENCIA
BLYNK_WRITE(VPIN_EMERGENCY)
{
  int valor = param.asInt();
  if (valor == 1)
  {
    emergencia = true;
    apagarMotor();
    estadoMotor = MOTOR_EMERGENCIA;
    tiempoInicioEstado = millis();
    Blynk.virtualWrite(VPIN_START, 0);
    actualizarIndicadores();
    return;
  }

  emergencia = false;
  apagarMotor();
  estadoMotor = MOTOR_PARADO;
  tiempoInicioEstado = millis();
  Blynk.virtualWrite(VPIN_START, 0);
  Blynk.virtualWrite(VPIN_EMERGENCY, 0);
  actualizarIndicadores();
}

// TIEMPO DE RAMPA
BLYNK_WRITE(VPIN_RAMP)
{
  tiempoRampa = param.asFloat();
  if (tiempoRampa < 2.0) tiempoRampa = 2.0;
  if (tiempoRampa > 10.0) tiempoRampa = 10.0;
}

// MÁQUINA DE ESTADOS
void ejecutarFSM()
{
  unsigned long ahora = millis();

  switch (estadoMotor)
  {
    case MOTOR_PARADO:
    case MOTOR_EMERGENCIA:
    case MOTOR_FALLA_TERMICA:
      digitalWrite(PIN_K1, LOW);
      digitalWrite(PIN_K2, LOW);
      digitalWrite(PIN_K3, LOW);
      break;

    case MOTOR_ESTRELLA:
      digitalWrite(PIN_K1, HIGH);
      digitalWrite(PIN_K2, LOW);
      digitalWrite(PIN_K3, HIGH);

      if (ahora - tiempoInicioEstado >= (unsigned long)(tiempoRampa * 1000.0))
      {
        digitalWrite(PIN_K1, HIGH);
        digitalWrite(PIN_K3, LOW);
        digitalWrite(PIN_K2, LOW);
        estadoMotor = MOTOR_TIEMPO_MUERTO;
        tiempoInicioEstado = ahora;
        actualizarIndicadores();
      }
      break;

    case MOTOR_TIEMPO_MUERTO:
      digitalWrite(PIN_K1, HIGH);
      digitalWrite(PIN_K2, LOW);
      digitalWrite(PIN_K3, LOW);
      
      if (ahora - tiempoInicioEstado >= DEAD_TIME_MS)
      {
        activarDelta();
        estadoMotor = MOTOR_DELTA;
        tiempoInicioEstado = ahora;
        actualizarIndicadores();
      }
      break;

    case MOTOR_DELTA:
      digitalWrite(PIN_K1, HIGH);
      digitalWrite(PIN_K2, HIGH);
      digitalWrite(PIN_K3, LOW);
      break;
  }

  // PROTECCION CRUZADA K2 / K3
  if (digitalRead(PIN_K2) == HIGH && digitalRead(PIN_K3) == HIGH)
  {
    digitalWrite(PIN_K2, LOW);
    digitalWrite(PIN_K3, LOW);
    digitalWrite(PIN_K1, HIGH);
    estadoMotor = MOTOR_ESTRELLA;
    tiempoInicioEstado = millis();
    actualizarIndicadores();
  }
}

// ==========================================
// FUNCIÓN RELÉ TÉRMICO VIRTUAL
// ==========================================
void verificarProteccionTermica()
{
  if (estadoMotor == MOTOR_PARADO || estadoMotor == MOTOR_EMERGENCIA || estadoMotor == MOTOR_FALLA_TERMICA) {
    overcurrentActive = false;
    return;
  }

  if (estadoMotor == MOTOR_TIEMPO_MUERTO) {
    overcurrentActive = false;
    return;
  }

  unsigned long tiempoEnEstado = millis() - tiempoInicioEstado;

  if ((estadoMotor == MOTOR_ESTRELLA || estadoMotor == MOTOR_DELTA) && tiempoEnEstado < INRUSH_GRACE_TIME_MS) {
    overcurrentActive = false;
    return;
  }

  if (currentRMS > MAX_CURRENT_NOMINAL) {
    if (!overcurrentActive) {
      overcurrentActive = true;
      overcurrentStartTime = millis();
    } else {
      if (millis() - overcurrentStartTime >= OVERCURRENT_TRIP_TIME_MS) {
        apagarMotor();
        estadoMotor = MOTOR_FALLA_TERMICA;
        tiempoInicioEstado = millis();
        
        if (Blynk.connected()) {
          Blynk.virtualWrite(VPIN_START, 0);
          Blynk.logEvent("falla_termica", "Sobrecarga detectada. Motor detenido por seguridad.");
        }
        actualizarIndicadores();
      }
    }
  } else {
    overcurrentActive = false;
  }
}

// PROCESAR ACS712
void procesarACS712()
{
  unsigned long ahoraMicros = micros();

  if (ahoraMicros - lastCurrentSampleMicros < CURRENT_SAMPLE_INTERVAL_US) return;
  lastCurrentSampleMicros = ahoraMicros;

  int adc = analogRead(PIN_ACS);
  float voltageADC = ((float)adc / 4095.0) * 3.3;
  float voltageACS = voltageADC / DIVIDER_FACTOR;
  float acVoltage = voltageACS - acsZeroVoltage;

  currentSumSquares += acVoltage * acVoltage;
  currentSampleCount++;

  if (currentSampleCount >= CURRENT_SAMPLES)
  {
    float voltageRMS = sqrt(currentSumSquares / CURRENT_SAMPLES);
    currentRMS = voltageRMS / ACS_SENSITIVITY;

    if (currentRMS < 0.05) currentRMS = 0.0;
    
    currentSumSquares = 0.0;
    currentSampleCount = 0;
  }
}

// ENVIAR CORRIENTE (OPTIMIZADO)
void enviarCorriente()
{
  if (!Blynk.connected()) return;

  float diferencia = abs(currentRMS - ultimaCorrienteEnviada);

  if (diferencia >= 0.15 || (currentRMS == 0.0 && ultimaCorrienteEnviada != 0.0))
  {
    Blynk.virtualWrite(VPIN_CURRENT, currentRMS);
    ultimaCorrienteEnviada = currentRMS;
  }
}

// CALIBRACIÓN
void calibrarACS712()
{
  delay(2000);
  const int muestras = 1000;
  long sumaADC = 0;
  
  for (int i = 0; i < muestras; i++)
  {
    sumaADC += analogRead(PIN_ACS);
    delayMicroseconds(500);
  }

  float promedioADC = (float)sumaADC / muestras;
  float voltageADC = (promedioADC / 4095.0) * 3.3;
  acsZeroVoltage = voltageADC / DIVIDER_FACTOR;
}

// ==========================================
// NUEVA FUNCIÓN: VERIFICACIÓN DE CONEXIÓN
// ==========================================
void verificarConexion() 
{
  static bool conexionPrevia = true; // Blynk asume conexión tras iniciar
  bool conexionActual = Blynk.connected();

  // Si se detecta que acaba de perder la conexión
  if (!conexionActual && conexionPrevia) {
    emergencia = true;
    apagarMotor();
    estadoMotor = MOTOR_EMERGENCIA;
    tiempoInicioEstado = millis();
  }
  
  conexionPrevia = conexionActual;
}

BLYNK_CONNECTED()
{
  Blynk.virtualWrite(VPIN_K1, digitalRead(PIN_K1));
  Blynk.virtualWrite(VPIN_K2, digitalRead(PIN_K2));
  Blynk.virtualWrite(VPIN_K3, digitalRead(PIN_K3));
  Blynk.virtualWrite(VPIN_MOTOR, nombreEstado());
  Blynk.virtualWrite(VPIN_ESP32, "ONLINE");

  // Al reconectarse, actualizar la app si se disparó la emergencia por desconexión
  if (emergencia) {
    Blynk.virtualWrite(VPIN_EMERGENCY, 1);
    Blynk.virtualWrite(VPIN_START, 0);
  } else {
    Blynk.virtualWrite(VPIN_EMERGENCY, 0);
  }
}

void setup()
{
  pinMode(PIN_K1, OUTPUT);
  pinMode(PIN_K2, OUTPUT);
  pinMode(PIN_K3, OUTPUT);

  digitalWrite(PIN_K1, LOW);
  digitalWrite(PIN_K2, LOW);
  digitalWrite(PIN_K3, LOW);
  
  pinMode(PIN_ACS, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ACS, ADC_11db);
  calibrarACS712();
  
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Único timer restante para ahorrar mensajes
  timer.setInterval(2000L, enviarCorriente);

  estadoMotor = MOTOR_PARADO;
  tiempoInicioEstado = millis();
  emergencia = false;
  currentRMS = 0.0;
}

void loop()
{
  Blynk.run();
  timer.run();
  
  verificarConexion(); // <--- Llama a la nueva función
  ejecutarFSM();
  procesarACS712();
  verificarProteccionTermica();
}