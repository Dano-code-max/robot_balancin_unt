#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- UUIDs ESTÁNDAR NORDIC UART SERVICE (NUS) PARA WEB BLUETOOTH ---
#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool deviceConnected = false;
String bleRxBuffer = "";
portMUX_TYPE bleMux = portMUX_INITIALIZER_UNLOCKED;

// --- PINES HARDWARE VERIFICADOS ---
#define LED_PIN 2
#define SDA_PIN 21
#define SCL_PIN 22

#define ENC_IZQ_PIN 34
#define ENC_DER_PIN 19

#define ENA_PIN 25
#define IN1_PIN 27
#define IN2_PIN 26

#define ENB_PIN 17
#define IN3_PIN 16
#define IN4_PIN 4

// --- CONSTANTES FÍSICAS Y DE MUESTREO ---
const uint8_t MPU_ADDR = 0x68;
const unsigned long TS_US = 10000; // Lazo interno: Ts = 10 ms (100 Hz)
const float DT = 0.01f;
const float DT_POS = 0.05f;        // Lazo externo: Ts_pos = 50 ms (20 Hz)
const float V_BAT = 7.4f;
const float CM_POR_PULSO = 1.021f; // Rueda 65 mm / 20 pulsos

// --- PARÁMETROS LAZO INTERNO: ÁNGULO ---
float setpointBaseAngulo = 5.75f; // Alineado con centro de gravedad real [grados]
float Kp_ang = 52.0f;             // [V / rad]
float Ki_ang = 180.0f;            // [V / (rad*s)]
float Kd_ang = 1.45f;             // [V*s / rad]

// --- CALIBRACIÓN INDIVIDUAL DE MOTORES ---
int pwmMinIzq = 80;               // Zona muerta avance motor izq (comando ZI)
int pwmMinDer = 80;               // Zona muerta avance motor der (comando ZD)
int pwmMinRev = 84;               // Zona muerta reversa ambos motores (comando ZR)
float balanceDer = 1.00f;         // Factor simetría motor derecho (comando B)

// --- PARÁMETROS LAZO EXTERNO: POSICIÓN LINEAL ---
float targetPosCm = 0.0f;
float Kp_pos = 0.035f;            // [grados / cm]
float Ki_pos = 0.005f;            // [grados / (cm*s)]
float Kd_pos = 0.085f;            // [grados*s / cm]
const float MAX_TILT_FWD = 1.80f; // Máxima inclinación hacia adelante [grados]
const float MAX_TILT_BACK = 1.25f;// Máxima inclinación hacia atrás [grados]

// --- PARÁMETROS CONTROL DE RUMBO / YAW ---
float targetYawDeg = 0.0f;        // Orientación objetivo [grados]
float Kp_yaw = 0.045f;            // [V / grado]
float Kd_yaw = 0.018f;            // [V*s / grado]

bool controlActivo = true;
bool modoCSV = true;

// --- PARADA DE EMERGENCIA DIRECCIONAL (CAÍDA FRONTAL SEGURA) ---
bool paradaEmergenciaActiva = false;
unsigned long inicioParadaEmergencia = 0;
const unsigned long TIEMPO_IMPULSO_STOP_MS = 160; // Calibrado a 160 ms
const float ANGULO_CORTE_FRONTAL = 12.0f;          // Grados hacia adelante para cortar antes

// --- VARIABLES PARA IDENTIFICACIÓN PRBS (CAPÍTULO 4) ---
bool prbsActivo = false;
unsigned long inicioPRBS = 0;
uint8_t lfsr = 0x5A;
int contPRBS = 0;
float uPRBS = 0.0f;

// --- VARIABLES DE ESTADO ---
unsigned long tiempoAnterior = 0;
float anguloY = 0.0f;
float yawDeg = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

float integralAng = 0.0f;
float voltajeBase = 0.0f;

float deltaAnguloPos = 0.0f;
float integralPos = 0.0f;
float posAnteriorCm = 0.0f;
float velPosFiltrada = 0.0f;
int contadorLazoPos = 0;

// Encoders
volatile long pulsosIzq = 0;
volatile long pulsosDer = 0;
volatile int dirIzq = 0;
volatile int dirDer = 0;
volatile unsigned long ultimoPulsoIzq = 0;
volatile unsigned long ultimoPulsoDer = 0;

// --- CALLBACKS BLE ---
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pSrv) override {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer *pSrv) override {
    deviceConnected = false;
    BLEDevice::startAdvertising(); // Permite reconectar sin reiniciar el ESP32
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String rxValue = String(pCharacteristic->getValue().c_str());
    if (rxValue.length() > 0) {
      portENTER_CRITICAL(&bleMux);
      bleRxBuffer += rxValue;
      portEXIT_CRITICAL(&bleMux);
    }
  }
};

void enviarBT(const String &mensaje) {
  if (!deviceConnected || pTxCharacteristic == nullptr) return;
  pTxCharacteristic->setValue((uint8_t *)mensaje.c_str(), mensaje.length());
  pTxCharacteristic->notify();
}

void IRAM_ATTR isrEncoderIzq() {
  unsigned long ahora = micros();
  if (ahora - ultimoPulsoIzq > 2500) {
    if (digitalRead(ENC_IZQ_PIN) == HIGH) {
      pulsosIzq += dirIzq;
      ultimoPulsoIzq = ahora;
    }
  }
}

void IRAM_ATTR isrEncoderDer() {
  unsigned long ahora = micros();
  if (ahora - ultimoPulsoDer > 2500) {
    if (digitalRead(ENC_DER_PIN) == HIGH) {
      pulsosDer += dirDer;
      ultimoPulsoDer = ahora;
    }
  }
}

bool esNumeroValido(const String &s) {
  if (s.length() == 0) return false;
  char c = s.charAt(0);
  return isDigit(c) || c == '-' || c == '+' || c == '.';
}

void aplicarVoltajeMotores(float vIzq, float vDer) {
  vDer *= balanceDer;

  vIzq = constrain(vIzq, -V_BAT, V_BAT);
  vDer = constrain(vDer, -V_BAT, V_BAT);

  // Motor Izquierdo
  if (fabs(vIzq) < 0.10f) {
    dirIzq = 0;
    digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, 0);
  } else if (vIzq > 0.0f) {
    int pwmI = pwmMinIzq + (int)((vIzq / V_BAT) * (255 - pwmMinIzq));
    dirIzq = 1;
    digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, constrain(pwmI, 0, 255));
  } else {
    int pwmI = pwmMinRev + (int)((fabs(vIzq) / V_BAT) * (255 - pwmMinRev));
    dirIzq = -1;
    digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, constrain(pwmI, 0, 255));
  }

  // Motor Derecho
  if (fabs(vDer) < 0.10f) {
    dirDer = 0;
    digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);
    analogWrite(ENB_PIN, 0);
  } else if (vDer > 0.0f) {
    int pwmD = pwmMinDer + (int)((vDer / V_BAT) * (255 - pwmMinDer));
    dirDer = 1;
    digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);
    analogWrite(ENB_PIN, constrain(pwmD, 0, 255));
  } else {
    int pwmD = pwmMinRev + (int)((fabs(vDer) / V_BAT) * (255 - pwmMinRev));
    dirDer = -1;
    digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, HIGH);
    analogWrite(ENB_PIN, constrain(pwmD, 0, 255));
  }
}

void configurarMPU6050() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true);
}

void leerMPU6050(float &accAngY, float &gyroRateY, float &gyroRateZ) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  int16_t acX = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t acZ = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  Wire.read(); Wire.read();
  int16_t gyY = (Wire.read() << 8) | Wire.read();
  int16_t gyZ = (Wire.read() << 8) | Wire.read();

  accAngY = atan2(-(float)acX, (float)acZ) * 180.0f / PI;
  gyroRateY = ((float)gyY / 131.0f) - gyroOffsetY;
  gyroRateZ = ((float)gyZ / 131.0f) - gyroOffsetZ;
}

void calibrarGiroscopio() {
  digitalWrite(LED_PIN, HIGH);
  Serial.println("1/2 Calibrando giroscopio Y/Z (2 seg)... NO MOVER.");
  float sumaY = 0, sumaZ = 0, ay, gy, gz;
  for (int i = 0; i < 200; i++) {
    leerMPU6050(ay, gy, gz);
    sumaY += gy;
    sumaZ += gz;
    delay(10);
  }
  gyroOffsetY = sumaY / 200.0f;
  gyroOffsetZ = sumaZ / 200.0f;
}

void cuentaRegresivaUbicacion() {
  Serial.println("2/2 Ubica el robot en el suelo (3 seg)...");
  for (int i = 0; i < 30; i++) {
    digitalWrite(LED_PIN, i % 2);
    delay(100);
  }
  float ay, gy, gz;
  leerMPU6050(ay, gy, gz);
  anguloY = ay;
  yawDeg = 0.0f;
  targetYawDeg = 0.0f;
  integralAng = 0.0f;

  noInterrupts();
  pulsosIzq = 0; pulsosDer = 0;
  interrupts();

  targetPosCm = 0.0f;
  integralPos = 0.0f;
  posAnteriorCm = 0.0f;
  velPosFiltrada = 0.0f;
  deltaAnguloPos = 0.0f;

  digitalWrite(LED_PIN, HIGH);
  Serial.println("ACTIVO: BLE UART + Control Completo.");
}

void ejecutarComando(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  char buf[64];

  // 1. COMANDOS DE ACCIÓN Y NAVEGACIÓN
  if (cmd == "FWD")   { targetPosCm += 15.0f; snprintf(buf, sizeof(buf), "#POS_OBJ:%.1f\n", targetPosCm); enviarBT(buf); return; }
  if (cmd == "BACK")  { targetPosCm -= 15.0f; snprintf(buf, sizeof(buf), "#POS_OBJ:%.1f\n", targetPosCm); enviarBT(buf); return; }
  if (cmd == "LEFT")  { targetYawDeg += 30.0f; snprintf(buf, sizeof(buf), "#YAW_OBJ:%.1f\n", targetYawDeg); enviarBT(buf); return; }
  if (cmd == "RIGHT") { targetYawDeg -= 30.0f; snprintf(buf, sizeof(buf), "#YAW_OBJ:%.1f\n", targetYawDeg); enviarBT(buf); return; }

  if (cmd == "STOP") {
    prbsActivo = false;
    if (paradaEmergenciaActiva) {
      paradaEmergenciaActiva = false;
      controlActivo = false;
      aplicarVoltajeMotores(0.0f, 0.0f);
      enviarBT("#STOP_INMEDIATO\n");
    } else if (controlActivo && fabs(setpointBaseAngulo - anguloY) <= 25.0f) {
      controlActivo = false;
      paradaEmergenciaActiva = true;
      inicioParadaEmergencia = millis();
      aplicarVoltajeMotores(-V_BAT, -V_BAT);
      enviarBT("#STOP_FRONTAL\n");
    } else {
      controlActivo = false;
      paradaEmergenciaActiva = false;
      aplicarVoltajeMotores(0.0f, 0.0f);
      enviarBT("#STOP\n");
    }
    return;
  }

  if (cmd == "RUN") {
    paradaEmergenciaActiva = false;
    integralAng = 0.0f; integralPos = 0.0f;
    controlActivo = true;
    enviarBT("#RUN\n");
    return;
  }

  if (cmd == "ZERO") {
    noInterrupts();
    pulsosIzq = 0; pulsosDer = 0;
    interrupts();
    targetPosCm = 0.0f; yawDeg = 0.0f; targetYawDeg = 0.0f;
    integralPos = 0.0f; posAnteriorCm = 0.0f; velPosFiltrada = 0.0f;
    enviarBT("#ZERO_OK\n");
    return;
  }
  if (cmd == "CSV") { modoCSV = !modoCSV; return; }

  if (cmd == "PRBS") {
    prbsActivo = true;
    inicioPRBS = millis();
    contPRBS = 0;
    lfsr = 0x5A;
    uPRBS = 0.65f;
    enviarBT("#PRBS_START\n");
    return;
  }

  // 2. RECÁLCULO PID DESDE MATLAB (PID,Kp,Ki,Kd)
  if (cmd.startsWith("PID,")) {
    int c1 = cmd.indexOf(',');
    int c2 = cmd.indexOf(',', c1 + 1);
    int c3 = cmd.indexOf(',', c2 + 1);
    if (c1 > 0 && c2 > 0 && c3 > 0) {
      Kp_ang = cmd.substring(c1 + 1, c2).toFloat();
      Ki_ang = cmd.substring(c2 + 1, c3).toFloat();
      Kd_ang = cmd.substring(c3 + 1).toFloat();
      integralAng = 0.0f;
      snprintf(buf, sizeof(buf), "#OK_PID:%.2f,%.2f,%.2f\n", Kp_ang, Ki_ang, Kd_ang);
      enviarBT(buf);
    }
    return;
  }

  // 3. CALIBRACIÓN DE MOTORES
  if (cmd.startsWith("ZR") && esNumeroValido(cmd.substring(2))) {
    pwmMinRev = cmd.substring(2).toInt(); snprintf(buf, sizeof(buf), "#ZR=%d\n", pwmMinRev); enviarBT(buf); return;
  }
  if (cmd.startsWith("ZI") && esNumeroValido(cmd.substring(2))) {
    pwmMinIzq = cmd.substring(2).toInt(); snprintf(buf, sizeof(buf), "#ZI=%d\n", pwmMinIzq); enviarBT(buf); return;
  }
  if (cmd.startsWith("ZD") && esNumeroValido(cmd.substring(2))) {
    pwmMinDer = cmd.substring(2).toInt(); snprintf(buf, sizeof(buf), "#ZD=%d\n", pwmMinDer); enviarBT(buf); return;
  }
  if (cmd.startsWith("B") && esNumeroValido(cmd.substring(1))) {
    balanceDer = cmd.substring(1).toFloat(); snprintf(buf, sizeof(buf), "#BalDer=%.3f\n", balanceDer); enviarBT(buf); return;
  }

  // 4. AJUSTES RÁPIDOS DE GANANCIAS
  String arg = cmd.substring(1);
  if (!esNumeroValido(arg)) return;

  char tipo = cmd.charAt(0);
  float valor = arg.toFloat();

  if      (tipo == 'P') { Kp_ang = valor; snprintf(buf, sizeof(buf), "#Kp_ang=%.2f\n", Kp_ang); enviarBT(buf); }
  else if (tipo == 'I') { Ki_ang = valor; integralAng = 0.0f; snprintf(buf, sizeof(buf), "#Ki_ang=%.2f\n", Ki_ang); enviarBT(buf); }
  else if (tipo == 'D') { Kd_ang = valor; snprintf(buf, sizeof(buf), "#Kd_ang=%.3f\n", Kd_ang); enviarBT(buf); }
  else if (tipo == 'S') { setpointBaseAngulo = valor; integralAng = 0.0f; snprintf(buf, sizeof(buf), "#Set=%.2f\n", setpointBaseAngulo); enviarBT(buf); }
  else if (tipo == 'A') { Kp_pos = valor; snprintf(buf, sizeof(buf), "#Kp_pos=%.3f\n", Kp_pos); enviarBT(buf); }
  else if (tipo == 'M') { Ki_pos = valor; integralPos = 0.0f; snprintf(buf, sizeof(buf), "#Ki_pos=%.3f\n", Ki_pos); enviarBT(buf); }
  else if (tipo == 'V') { Kd_pos = valor; snprintf(buf, sizeof(buf), "#Kd_pos=%.3f\n", Kd_pos); enviarBT(buf); }
  else if (tipo == 'Y') { Kp_yaw = valor; snprintf(buf, sizeof(buf), "#Kp_yaw=%.3f\n", Kp_yaw); enviarBT(buf); }
  else if (tipo == 'R') { Kd_yaw = valor; snprintf(buf, sizeof(buf), "#Kd_yaw=%.3f\n", Kd_yaw); enviarBT(buf); }
}

void procesarComandosBT() {
  String linea = "";
  portENTER_CRITICAL(&bleMux);
  int idx = bleRxBuffer.indexOf('\n');
  if (idx >= 0) {
    linea = bleRxBuffer.substring(0, idx);
    bleRxBuffer.remove(0, idx + 1);
  }
  portEXIT_CRITICAL(&bleMux);

  if (linea.length() > 0) {
    ejecutarComando(linea);
  }
}

void iniciarBLE() {
  BLEDevice::init("RobotBalancin_UNT");
  BLEDevice::setMTU(128);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_WRITE_NR
                                         );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void setup() {
  Serial.begin(115200);
  iniciarBLE();

  pinMode(LED_PIN, OUTPUT);
  pinMode(ENC_IZQ_PIN, INPUT);
  pinMode(ENC_DER_PIN, INPUT);
  pinMode(ENA_PIN, OUTPUT); pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT); pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);

  aplicarVoltajeMotores(0.0f, 0.0f);

  attachInterrupt(digitalPinToInterrupt(ENC_IZQ_PIN), isrEncoderIzq, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DER_PIN), isrEncoderDer, RISING);

  configurarMPU6050();
  delay(200);

  calibrarGiroscopio();
  cuentaRegresivaUbicacion();

  tiempoAnterior = micros();
}

void loop() {
  procesarComandosBT();

  unsigned long ahora = micros();
  if (ahora - tiempoAnterior >= TS_US) {
    tiempoAnterior += TS_US;

    float accY, gyroY, gyroZ;
    leerMPU6050(accY, gyroY, gyroZ);
    anguloY = 0.98f * (anguloY + gyroY * DT) + 0.02f * accY;

    if (fabs(gyroZ) < 0.25f) gyroZ = 0.0f;
    yawDeg += gyroZ * DT;

    noInterrupts();
    long pIzq = pulsosIzq;
    long pDer = pulsosDer;
    interrupts();

    float posIzqCm = pIzq * CM_POR_PULSO;
    float posDerCm = pDer * CM_POR_PULSO;
    float posActualCm = 0.5f * (posIzqCm + posDerCm);

    float uTotalIdent = 0.0f;

    // SECUENCIA DE PARADA DE EMERGENCIA CON CAÍDA FRONTAL
    if (paradaEmergenciaActiva) {
      bool vencioAdelante = (anguloY <= (setpointBaseAngulo - ANGULO_CORTE_FRONTAL));
      bool tiempoCumplido = (millis() - inicioParadaEmergencia >= TIEMPO_IMPULSO_STOP_MS);

      if (vencioAdelante || tiempoCumplido) {
        paradaEmergenciaActiva = false;
        aplicarVoltajeMotores(0.0f, 0.0f);
        voltajeBase = 0.0f;
        enviarBT("#MOTORES_OFF\n");
      } else {
        aplicarVoltajeMotores(-V_BAT, -V_BAT);
        voltajeBase = -V_BAT;
      }
    } else if (!controlActivo || fabs(setpointBaseAngulo - anguloY) > 35.0f) {
      aplicarVoltajeMotores(0.0f, 0.0f);
      voltajeBase = 0.0f;
      integralAng = 0.0f;
      integralPos = 0.0f;
      deltaAnguloPos = 0.0f;
      posAnteriorCm = posActualCm;
      velPosFiltrada = 0.0f;
      prbsActivo = false;
    } else {
      // 1. LAZO EXTERNO: POSICIÓN LINEAL (Cada 50 ms)
      if (++contadorLazoPos >= 5) {
        contadorLazoPos = 0;

        float errorPos = targetPosCm - posActualCm;
        float velPosCruda = (posActualCm - posAnteriorCm) / DT_POS;
        velPosFiltrada = 0.75f * velPosFiltrada + 0.25f * velPosCruda;
        posAnteriorCm = posActualCm;

        integralPos += Ki_pos * DT_POS * errorPos;
        integralPos = constrain(integralPos, -0.8f, 0.8f);

        float salidaPos = -Kp_pos * errorPos - integralPos + Kd_pos * velPosFiltrada;
        deltaAnguloPos = constrain(salidaPos, -MAX_TILT_FWD, MAX_TILT_BACK);
      }

      // 2. LAZO INTERNO: ÁNGULO (Cada 10 ms)
      float setpointTotal = setpointBaseAngulo + deltaAnguloPos;
      float errorGrados = setpointTotal - anguloY;
      float errorRad = errorGrados * (PI / 180.0f);

      float P_ang = Kp_ang * errorRad;
      integralAng += Ki_ang * DT * errorRad;
      integralAng = constrain(integralAng, -V_BAT * 0.6f, V_BAT * 0.6f);
      float D_ang = -Kd_ang * (gyroY * (PI / 180.0f));

      voltajeBase = P_ang + integralAng + D_ang;

      // 3. LAZO PD DE ORIENTACIÓN (YAW)
      float errorYaw = targetYawDeg - yawDeg;
      float uYaw = 0.0f;
      if (fabs(errorYaw) > 1.2f || fabs(gyroZ) > 3.5f) {
        uYaw = constrain(Kp_yaw * errorYaw - Kd_yaw * gyroZ, -1.1f, 1.1f);
      }

      // 4. EXCITACIÓN PRBS (Solo cuando MATLAB envía "PRBS")
      float uExcitacion = 0.0f;
      if (prbsActivo) {
        if (millis() - inicioPRBS >= 10000) {
          prbsActivo = false;
          enviarBT("#PRBS_END\n");
        } else {
          if (++contPRBS >= 5) {
            contPRBS = 0;
            uint8_t bit = ((lfsr >> 6) ^ (lfsr >> 5)) & 1;
            lfsr = ((lfsr << 1) | bit) & 0x7F;
            uPRBS = (lfsr & 1) ? 0.65f : -0.65f;
          }
          uExcitacion = uPRBS;
        }
      }

      uTotalIdent = voltajeBase + uExcitacion;

      if (uTotalIdent < -0.25f) {
        float maxYawRev = 0.45f * fabs(uTotalIdent);
        uYaw = constrain(uYaw, -maxYawRev, maxYawRev);
      }

      float vIzq = uTotalIdent - uYaw;
      float vDer = uTotalIdent + uYaw;

      if (fabs(uTotalIdent) < 0.25f && fabs(errorYaw) < 2.0f) {
        if (uTotalIdent >= 0.0f) {
          vIzq = max(0.0f, vIzq);
          vDer = max(0.0f, vDer);
        } else {
          vIzq = min(0.0f, vIzq);
          vDer = min(0.0f, vDer);
        }
      }

      aplicarVoltajeMotores(vIzq, vDer);
    }

    // 5. TELEMETRÍA BLE (A 25 Hz para fluidez máxima sin saturar paquetes)
// 5. TELEMETRÍA BLE (Se pausa al recibir STOP y se reanuda con RUN)
    static int divPrint = 0;
    if (prbsActivo && deviceConnected) {
      char buf[64];
      snprintf(buf, sizeof(buf), "ID,%lu,%.4f,%.4f\n", millis(), uTotalIdent, anguloY);
      enviarBT(buf);
    } else if (controlActivo && ++divPrint >= 4 && deviceConnected) {
      divPrint = 0;
      char buf[80];
      if (modoCSV) {
        snprintf(buf, sizeof(buf), "%lu,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                 millis(), voltajeBase, anguloY, posActualCm, posIzqCm, posDerCm);
      } else {
        snprintf(buf, sizeof(buf), "Pos:%.1fcm | Ang:%.2f | Yaw:%.1fdeg | V:%.2f\n",
                 posActualCm, anguloY, yawDeg, voltajeBase);
      }
      enviarBT(buf);
    }
  }
}
