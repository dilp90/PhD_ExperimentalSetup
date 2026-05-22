#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <EMGFilters.h>
#include <math.h>

// ================== INSTANCIAS GLOBALES ==================
Adafruit_BNO08x bno = Adafruit_BNO08x();
EMGFilters myFilter;

// Variables de estado
float roll = 0, pitch = 0, yaw = 0;
unsigned long lastToggleTime = 0;
int currentClass = 0; 
int toggleCount = 0;
int repetitionCount = 0;
const int maxRepetitions = 10;
bool stopAcquisition = false;
bool imuPresent = false; 

// Configuración EMG
#define SensorInputPin 1 
const uint16_t DS = 10;   
uint16_t ds_count = 0;
uint32_t emg_accum = 0;

// Función de conversión
void quaternionToEuler(float qr, float qi, float qj, float qk, float &roll, float &pitch, float &yaw) {
  float sqr = qr * qr;
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;
  roll = atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  pitch = asinf(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  yaw = atan2f(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  roll *= 180.0f / PI;
  pitch *= 180.0f / PI;
  yaw *= 180.0f / PI;
}

void setup() {
  Serial.begin(115200);
  while(!Serial && millis() < 5000); 
  delay(1000);
  
  Serial.println("\n--- SISTEMA DE ADQUISICION EMG/IMU ---");

  // Inicializar I2C en pines 8 y 9
  Wire.begin(8, 9);
  Wire.setClock(100000); 
  
  myFilter.init(SAMPLE_FREQ_1000HZ, NOTCH_FREQ_50HZ, true, true, true);

  Serial.println("# Buscando BNO08x...");
  
  // Intento en 0x4A
  Wire.beginTransmission(0x4A);
  if (Wire.endTransmission() == 0) {
    if (bno.begin_I2C(0x4A, &Wire)) imuPresent = true;
  }
  
  // Si no se halló, intento en 0x4B
  if (!imuPresent) {
    Wire.beginTransmission(0x4B);
    if (Wire.endTransmission() == 0) {
      if (bno.begin_I2C(0x4B, &Wire)) imuPresent = true;
    }
  }

  if (imuPresent) {
    Serial.println("# BNO08x encontrado.");
    bno.enableReport(SH2_ROTATION_VECTOR);
  } else {
    Serial.println("# ADVERTENCIA: No se detectó el IMU físicamente.");
  }

  lastToggleTime = millis();
  Serial.println("H,EMG,Clase,Roll,Pitch,Yaw");
}

void loop() {
  if (stopAcquisition) return;

  // 1. IMU (Solo si está presente)
  if (imuPresent) {
    sh2_SensorValue_t sensorValue;
    if (bno.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        quaternionToEuler(sensorValue.un.rotationVector.real,
                          sensorValue.un.rotationVector.i,
                          sensorValue.un.rotationVector.j,
                          sensorValue.un.rotationVector.k,
                          roll, pitch, yaw);
      }
    }
  }

  // 2. EMG
  int v = analogRead(SensorInputPin);      
  v = map(v, 0, 4095, 0, 1023);           
  int y = myFilter.update(v);
  unsigned long envelope = (unsigned long)(y * y); // sq(y)
  
  emg_accum += envelope;
  ds_count++;

  // 3. Temporización de clases
  if (millis() - lastToggleTime >= 5000) {
    currentClass = 1 - currentClass;
    lastToggleTime = millis();
    toggleCount++;
    if (toggleCount % 2 == 0) repetitionCount++;
    if (repetitionCount >= maxRepetitions) {
      stopAcquisition = true;
      Serial.println("# Fin de adquisicion.");
      return;
    }
  }

  // 4. Salida Serial (Downsampling)
  if (ds_count >= DS) {
    Serial.print("D,");
    Serial.print(emg_accum / ds_count);
    Serial.print(",");
    Serial.print(currentClass);
    Serial.print(",");
    Serial.print(roll, 2); Serial.print(",");
    Serial.print(pitch, 2); Serial.print(",");
    Serial.println(yaw, 2);

    emg_accum = 0;
    ds_count = 0;
  }
  
  delayMicroseconds(1000); 
}
