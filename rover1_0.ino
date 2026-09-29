//===================== LIBRERÍAS =====================
#include <Wire.h>
#include <Preferences.h>
#include "INA226.h"
#include <Adafruit_QMC5883P.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//===================== MOTORES (A, B = delanteros | C, D = traseros) =====================
//                            A   B   C   D
const uint8_t PIN_PWM[4] = { 13, 25, 19,  4 };
const uint8_t PIN_FWD[4] = { 12, 26,  5, 17 };
const uint8_t PIN_BWD[4] = { 14, 27, 18, 16 };

#define PWM_FREQ 5000
#define PWM_RES  8          // 0-255
#define VEL      150        // velocidad de la secuencia y del test

//===================== INA226 =====================
#define SDA_PIN 22
#define SCL_PIN 21
INA226 INA(0x44);
const float R_SHUNT = 0.0005f;   // 0.5 mOhm

volatile float g_busVoltage_V   = 0;   // corregido con ganancia
volatile float g_current_A      = 0;   // corregido con ganancia
volatile float g_power_W        = 0;
volatile float g_shuntVoltage_mV = 0;
volatile float g_busVoltage_raw = 0;   // sin ganancia (para calibrar con multímetro)
volatile float g_current_raw    = 0;

//===================== MAGNETÓMETRO QMC5883P =====================
Adafruit_QMC5883P qmc;
bool g_magOK = false;

const unsigned long CAL_TIME_MS = 15000;            // tiempo de calibración hard-iron

// Índices 0,1,2 = X,Y,Z. Valores por defecto hasta que haya calibración guardada en NVS
float magOffset[3] = { -0.8060f, -0.0595f, -0.1153f };   // hard-iron (Gauss)
float magComp[3]   = { 0, 0, 0 };                        // compensación por corriente (Gauss/A)

const float MOUNTING_OFFSET_DEG = -174.0f;  // rotación de montaje del sensor
const float DECLINATION_DEG     = -5.2f;    // declinación magnética local (verifícala en NOAA)

volatile float g_magX = 0, g_magY = 0, g_magZ = 0;  // Gauss, compensados
volatile float g_heading = 0;                       // grados 0-360
volatile bool  g_magOverflow = false;               // true si el sensor se saturó

//===================== MPU6050 =====================
Adafruit_MPU6050 mpu;
bool g_mpuOK = false;
volatile float g_accX = 0, g_accY = 0, g_accZ = 0;  // m/s^2
volatile float g_pitch = 0, g_roll = 0;             // grados (filtro complementario)
const float COMP_FILTER_ALPHA = 0.98f;

//===================== TELEMETRÍA / PLOTTER =====================
#define TELEMETRY_HZ        20
#define TELEMETRY_PERIOD_MS (1000 / TELEMETRY_HZ)
// 1 = agrega las series constantes Min:0 y Max:360 para fijar la escala del Serial Plotter
#define PLOT_LIMITS 1

// El Serial Plotter de Arduino IDE 2.x solo dibuja 8 series por línea.
// Por eso la telemetría se reparte en grupos (cambia en caliente con el comando PLOT).
enum PlotMode { PLOT_POWER, PLOT_MAG, PLOT_IMU };
#define PLOT_DEFAULT PLOT_MAG
volatile PlotMode g_plotMode = PLOT_DEFAULT;

//===================== SINCRONIZACIÓN =====================
Preferences prefs;
SemaphoreHandle_t prefsMutex;   // protege el acceso a NVS
SemaphoreHandle_t i2cMutex;     // protege el bus I2C compartido

// Trabajo pedido por el comando serial; lo ejecuta loop() con los motores parados
enum Request { REQ_NONE, REQ_CAL_MAG, REQ_CAL_COMP, REQ_TEST };
volatile Request g_request = REQ_NONE;
volatile bool g_calibrationInProgress = false;   // silencia la telemetría durante calibraciones

//===================== CONFIGURACIÓN PERSISTENTE =====================
struct Config {
  bool  invert[4];
  float voltageCal;
  float currentCal;
} cfg;

void setDefaultConfig() {
  for (int i = 0; i < 4; i++) cfg.invert[i] = false;
  cfg.voltageCal = 1.0f;
  cfg.currentCal = 1.0f;
}

void loadConfig() {
  setDefaultConfig();
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY) == pdTRUE) {
    prefs.begin("robotcfg", true);
    for (int i = 0; i < 4; i++) {
      char key[5] = { 'i', 'n', 'v', (char)('A' + i), 0 };
      cfg.invert[i] = prefs.getBool(key, cfg.invert[i]);
    }
    cfg.voltageCal = prefs.getFloat("voltCal", cfg.voltageCal);
    cfg.currentCal = prefs.getFloat("currCal", cfg.currentCal);
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }
}

void saveConfig() {
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY) == pdTRUE) {
    prefs.begin("robotcfg", false);
    for (int i = 0; i < 4; i++) {
      char key[5] = { 'i', 'n', 'v', (char)('A' + i), 0 };
      prefs.putBool(key, cfg.invert[i]);
    }
    prefs.putFloat("voltCal", cfg.voltageCal);
    prefs.putFloat("currCal", cfg.currentCal);
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }
  Serial.println("Configuracion guardada.");
}

void printConfig() {
  Serial.println("----- CONFIGURACION -----");
  for (int i = 0; i < 4; i++)
    Serial.printf("Invertir %c: %s\n", 'A' + i, cfg.invert[i] ? "SI" : "NO");
  Serial.printf("Ganancia voltaje: %.5f\n", cfg.voltageCal);
  Serial.printf("Ganancia corriente: %.5f\n", cfg.currentCal);
  Serial.printf("Mag offset  X/Y/Z: %.4f / %.4f / %.4f\n", magOffset[0], magOffset[1], magOffset[2]);
  Serial.printf("Mag comp    X/Y/Z: %.5f / %.5f / %.5f\n", magComp[0], magComp[1], magComp[2]);
  Serial.println("-------------------------");
}

//===================== CALIBRACIÓN MAG PERSISTENTE =====================
void loadMagCalibration() {
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY) == pdTRUE) {
    prefs.begin("magcal", true);
    if (prefs.isKey("offX")) {
      magOffset[0] = prefs.getFloat("offX", magOffset[0]);
      magOffset[1] = prefs.getFloat("offY", magOffset[1]);
      magOffset[2] = prefs.getFloat("offZ", magOffset[2]);
    }
    magComp[0] = prefs.getFloat("compX", magComp[0]);
    magComp[1] = prefs.getFloat("compY", magComp[1]);
    magComp[2] = prefs.getFloat("compZ", magComp[2]);
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }
}

void saveMagCalibration() {
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY) == pdTRUE) {
    prefs.begin("magcal", false);
    prefs.putFloat("offX", magOffset[0]);
    prefs.putFloat("offY", magOffset[1]);
    prefs.putFloat("offZ", magOffset[2]);
    prefs.putFloat("compX", magComp[0]);
    prefs.putFloat("compY", magComp[1]);
    prefs.putFloat("compZ", magComp[2]);
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }
  Serial.println("Calibracion del magnetometro guardada.");
}

//===================== UTILIDADES =====================
// Lee el magnetómetro con el mutex I2C. Devuelve true si hay dato nuevo.
bool readMag(float g[3], bool *overflow = nullptr) {
  bool ok = false;
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    if (qmc.isDataReady()) {
      ok = qmc.getGaussField(&g[0], &g[1], &g[2]);
      if (ok && overflow) *overflow = qmc.isOverflow();
    }
    xSemaphoreGive(i2cMutex);
  }
  return ok;
}

// Promedio móvil O(1)
struct MovAvg {
  static const int N = 8;
  float buf[N] = {0};
  float sum = 0;
  int idx = 0, count = 0;
  float update(float v) {
    sum += v - buf[idx];
    buf[idx] = v;
    idx = (idx + 1) % N;
    if (count < N) count++;
    return sum / count;
  }
};

float linearRegressionSlope(const float *x, const float *y, int n) {
  float sx = 0, sy = 0, sxy = 0, sx2 = 0;
  for (int i = 0; i < n; i++) {
    sx += x[i]; sy += y[i]; sxy += x[i] * y[i]; sx2 += x[i] * x[i];
  }
  float denom = n * sx2 - sx * sx;
  if (fabsf(denom) < 1e-9f) return 0.0f;
  return (n * sxy - sx * sy) / denom;
}

//===================== MOTORES =====================
// API LEDC: el core 3.x eliminó ledcSetup/ledcAttachPin y ledcWrite usa el pin.
// Este código compila en ambos (si la macro no existe, usa la API 2.x).
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_BEGIN(i)      ledcAttach(PIN_PWM[i], PWM_FREQ, PWM_RES)
  #define PWM_WRITE(i, d)   ledcWrite(PIN_PWM[i], d)
#else
  #define PWM_BEGIN(i)      do { ledcSetup(i, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_PWM[i], i); } while (0)
  #define PWM_WRITE(i, d)   ledcWrite(i, d)
#endif

void setMotor(int i, int speed) {
  bool forward = (speed >= 0) != cfg.invert[i];   // XOR con la inversión guardada
  digitalWrite(PIN_FWD[i], forward ? HIGH : LOW);
  digitalWrite(PIN_BWD[i], forward ? LOW : HIGH);
  PWM_WRITE(i, constrain(abs(speed), 0, 255));
}

void setAllMotors(int speed) {
  for (int i = 0; i < 4; i++) setMotor(i, speed);
}

void runMotorTest() {
  for (int i = 0; i < 4; i++) {
    Serial.printf("Probando motor %c ADELANTE\n", 'A' + i);
    setMotor(i, VEL);  delay(5000);
    Serial.printf("Probando motor %c ATRAS\n", 'A' + i);
    setMotor(i, -VEL); delay(5000);
    setMotor(i, 0);    delay(2000);
  }
  Serial.println("Test terminado.");
}

//===================== CALIBRACIONES (bloqueantes, se llaman desde loop) =====================
void calibrateMagnetometer() {
  g_calibrationInProgress = true;
  Serial.println("\n=== CALIBRACION MAGNETOMETRO ===");
  Serial.println("Gira el sensor en los 3 ejes (como un '8') durante 15 s...");
  delay(2000);

  float mn[3] = { 1e6f, 1e6f, 1e6f }, mx[3] = { -1e6f, -1e6f, -1e6f };
  unsigned long start = millis();
  while (millis() - start < CAL_TIME_MS) {
    float g[3];
    if (readMag(g)) {
      for (int k = 0; k < 3; k++) {
        if (g[k] < mn[k]) mn[k] = g[k];
        if (g[k] > mx[k]) mx[k] = g[k];
      }
    }
    delay(20);
  }
  for (int k = 0; k < 3; k++) magOffset[k] = (mx[k] + mn[k]) / 2.0f;

  Serial.printf("Offsets X/Y/Z: %.4f / %.4f / %.4f\n", magOffset[0], magOffset[1], magOffset[2]);
  saveMagCalibration();
  g_calibrationInProgress = false;
}

// Los motores GIRAN: levanta el robot antes de ejecutarlo.
void calibrateMotorCompensation() {
  g_calibrationInProgress = true;
  Serial.println("\n=== COMPENSACION AUTOMATICA POR CORRIENTE ===");
  Serial.println("ADVERTENCIA: los motores van a girar. Levanta el robot.");
  Serial.println("Inicia en 5 s...");
  delay(5000);

  const int NUM_STEPS = 4;
  const int speeds[NUM_STEPS] = { 0, 90, 170, 255 };   // 0 = referencia
  const unsigned long SETTLE_MS = 1500, SAMPLE_MS = 1500;
  float avgG[3][NUM_STEPS], avgI[NUM_STEPS];

  for (int s = 0; s < NUM_STEPS; s++) {
    Serial.printf("Paso %d/%d: velocidad = %d\n", s + 1, NUM_STEPS, speeds[s]);
    setAllMotors(speeds[s]);
    delay(SETTLE_MS);

    float sum[3] = { 0, 0, 0 }, sumI = 0;
    int count = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < SAMPLE_MS) {
      float g[3];
      if (readMag(g)) {
        for (int k = 0; k < 3; k++) sum[k] += g[k];
        sumI += g_current_A;
        count++;
      }
      delay(20);
    }
    for (int k = 0; k < 3; k++) avgG[k][s] = count ? sum[k] / count : 0;
    avgI[s] = count ? sumI / count : 0;
    Serial.printf("  I=%.3fA  X=%.4f  Y=%.4f  Z=%.4f\n", avgI[s], avgG[0][s], avgG[1][s], avgG[2][s]);
  }
  setAllMotors(0);

  for (int k = 0; k < 3; k++) magComp[k] = linearRegressionSlope(avgI, avgG[k], NUM_STEPS);
  Serial.printf("MAG_COMP X/Y/Z: %.5f / %.5f / %.5f\n", magComp[0], magComp[1], magComp[2]);
  saveMagCalibration();
  g_calibrationInProgress = false;
}

//===================== TAREA: POTENCIA (INA226) =====================
// El INA226 ya promedia 64 muestras por hardware, no hace falta promediar por software.
void powerMonitorTask(void *pv) {
  for (;;) {
    float v = 0, mv = 0;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      v  = INA.getBusVoltage();
      mv = INA.getShuntVoltage_mV();
      xSemaphoreGive(i2cMutex);
    }
    float iRaw = (mv / 1000.0f) / R_SHUNT;

    g_busVoltage_raw  = v;
    g_shuntVoltage_mV = mv;
    g_current_raw     = iRaw;
    g_busVoltage_V   = v * cfg.voltageCal;
    g_current_A      = iRaw * cfg.currentCal;
    g_power_W        = g_busVoltage_V * g_current_A;

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

//===================== TAREA: TELEMETRÍA (20 Hz, formato Serial Plotter) =====================
void telemetryTask(void *pv) {
  TickType_t lastWake = xTaskGetTickCount();
  const float dt = TELEMETRY_PERIOD_MS / 1000.0f;
  MovAvg fx, fy;
  bool imuSeeded = false;
  char line[288];
  const char *limits = PLOT_LIMITS ? "Min:0,Max:360," : "";

  for (;;) {
    if (!g_calibrationInProgress) {

      // ---- Magnetómetro ----
      if (g_magOK) {
        float g[3];
        bool overflow = false;
        bool newData = readMag(g, &overflow);
        if (newData) g_magOverflow = overflow;
        if (newData && !overflow) {
          float current = g_current_A;
          float cx = g[0] - magOffset[0] - magComp[0] * current;
          float cy = g[1] - magOffset[1] - magComp[1] * current;
          float cz = g[2] - magOffset[2] - magComp[2] * current;

          float ax = fx.update(cx);
          float ay = fy.update(cy);
          g_magX = cx; g_magY = cy; g_magZ = cz;

          // atan2 gira antihorario -> se niega; luego montaje + declinación
          float h = -atan2f(ay, ax) * RAD_TO_DEG + MOUNTING_OFFSET_DEG + DECLINATION_DEG;
          h = fmodf(h, 360.0f);
          if (h < 0) h += 360.0f;
          g_heading = h;
        }
      }

      // ---- IMU: pitch/roll con filtro complementario ----
      if (g_mpuOK) {
        sensors_event_t a, g, t;
        bool ok = false;
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
          ok = mpu.getEvent(&a, &g, &t);
          xSemaphoreGive(i2cMutex);
        }
        if (ok) {
          g_accX = a.acceleration.x;
          g_accY = a.acceleration.y;
          g_accZ = a.acceleration.z;

          float pitchAcc = atan2f(-g_accX, sqrtf(g_accY * g_accY + g_accZ * g_accZ)) * RAD_TO_DEG;
          float rollAcc  = atan2f(g_accY, g_accZ) * RAD_TO_DEG;

          if (!imuSeeded) {            // arranca desde el ángulo del acelerómetro
            g_pitch = pitchAcc;
            g_roll  = rollAcc;
            imuSeeded = true;
          } else {
            g_pitch = COMP_FILTER_ALPHA * (g_pitch + g.gyro.y * RAD_TO_DEG * dt) + (1.0f - COMP_FILTER_ALPHA) * pitchAcc;
            g_roll  = COMP_FILTER_ALPHA * (g_roll  + g.gyro.x * RAD_TO_DEG * dt) + (1.0f - COMP_FILTER_ALPHA) * rollAcc;
          }
        }
      }

      // ---- Una línea por ciclo con el grupo elegido (máx. 8 series con Min/Max) ----
      switch (g_plotMode) {
        case PLOT_POWER:
          snprintf(line, sizeof(line), "%sBusV:%.3f,ShuntmV:%.4f,CurrentA:%.3f,PowerW:%.3f",
                   limits, g_busVoltage_V, g_shuntVoltage_mV, g_current_A, g_power_W);
          break;
        case PLOT_MAG:
          snprintf(line, sizeof(line), "%sHeading:%.1f,MagX:%.3f,MagY:%.3f,MagZ:%.3f,Overflow:%d",
                   limits, g_heading, g_magX, g_magY, g_magZ, g_magOverflow ? 1 : 0);
          break;
        case PLOT_IMU:
          snprintf(line, sizeof(line), "%sPitch:%.1f,Roll:%.1f,AccX:%.3f,AccY:%.3f,AccZ:%.3f",
                   limits, g_pitch, g_roll, g_accX, g_accY, g_accZ);
          break;
      }
      Serial.println(line);
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
  }
}

//===================== COMANDOS SERIAL =====================
bool requestJob(Request r, const char *msg) {
  if (g_request != REQ_NONE) {
    Serial.println("Ya hay un trabajo pendiente, espera...");
    return false;
  }
  g_request = r;
  Serial.println(msg);
  return true;
}

void processCommand(String cmd) {
  cmd.replace('\t', ' ');
  cmd.trim();
  while (cmd.indexOf("  ") >= 0) cmd.replace("  ", " ");
  cmd.toUpperCase();
  Serial.print("> "); Serial.println(cmd);

  if (cmd.startsWith("INVERT ")) {
    int m = cmd.charAt(7) - 'A';
    if (m < 0 || m > 3) { Serial.println("Motor invalido. Usa A, B, C o D."); return; }
    cfg.invert[m] = !cfg.invert[m];
    saveConfig();
    printConfig();
  }
  else if (cmd.startsWith("CAL VOLT ")) {
    if (g_busVoltage_raw <= 0) { Serial.println("Sin lectura de voltaje valida."); return; }
    cfg.voltageCal = cmd.substring(9).toFloat() / g_busVoltage_raw;
    Serial.printf("Nueva ganancia de voltaje: %.5f\n", cfg.voltageCal);
    saveConfig();
  }
  else if (cmd.startsWith("CAL CURR ")) {
    if (g_current_raw <= 0) { Serial.println("Sin lectura de corriente valida (aplica carga)."); return; }
    cfg.currentCal = cmd.substring(9).toFloat() / g_current_raw;
    Serial.printf("Nueva ganancia de corriente: %.5f\n", cfg.currentCal);
    saveConfig();
  }
  else if (cmd.startsWith("CAL MAGCOMP")) {
    String rest = cmd.substring(11);
    rest.trim();

    if (rest == "AUTO") {
      if (!g_magOK) Serial.println("Magnetometro no disponible.");
      else requestJob(REQ_CAL_COMP, "Compensacion automatica solicitada. Levanta el robot: los motores van a girar.");
      return;
    }

    int sp = rest.indexOf(' ');
    if (sp < 0) { Serial.println("Uso: CAL MAGCOMP AUTO  o  CAL MAGCOMP <X|Y|Z> <valor>"); return; }

    int axis = rest.charAt(0) - 'X';
    if (rest.substring(0, sp).length() != 1 || axis < 0 || axis > 2) {
      Serial.println("Eje invalido. Usa X, Y o Z.");
      return;
    }
    magComp[axis] = rest.substring(sp + 1).toFloat();
    Serial.printf("Nuevo MAG_COMP_%c = %.5f\n", 'X' + axis, magComp[axis]);
    saveMagCalibration();
  }
  else if (cmd == "CAL MAG") {
    if (!g_magOK) Serial.println("Magnetometro no disponible.");
    else requestJob(REQ_CAL_MAG, "Calibracion del magnetometro solicitada.");
  }
  else if (cmd.startsWith("PLOT ")) {
    String m = cmd.substring(5);
    m.trim();
    if      (m == "POWER") g_plotMode = PLOT_POWER;
    else if (m == "MAG")   g_plotMode = PLOT_MAG;
    else if (m == "IMU")   g_plotMode = PLOT_IMU;
    else { Serial.println("Uso: PLOT POWER|MAG|IMU"); return; }
    Serial.println("Grafica cambiada.");
  }
  else if (cmd == "SHOW")  printConfig();
  else if (cmd == "RESET") { setDefaultConfig(); saveConfig(); printConfig(); }
  else if (cmd == "TEST")  requestJob(REQ_TEST, "Test de motores solicitado.");
  else {
    Serial.println("Comando no reconocido. Usa: INVERT A|B|C|D, CAL VOLT <v>, CAL CURR <a>, CAL MAG, CAL MAGCOMP AUTO, CAL MAGCOMP X|Y|Z <v>, PLOT POWER|MAG|IMU, SHOW, RESET, TEST");
  }
}

void serialCommandTask(void *pv) {
  String line;
  for (;;) {
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n') {
        line.trim();
        if (line.length() > 0) processCommand(line);
        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

//===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  for (int i = 0; i < 4; i++) {
    pinMode(PIN_FWD[i], OUTPUT);
    pinMode(PIN_BWD[i], OUTPUT);
    PWM_BEGIN(i);
  }

  prefsMutex = xSemaphoreCreateMutex();
  i2cMutex   = xSemaphoreCreateMutex();
  loadConfig();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);   // los 3 sensores soportan 400 kHz

  // ---- INA226 ----
  if (!INA.begin()) {
    Serial.println("INA226: no conecta, revisa el cableado.");
  } else {
    INA.setAverage(3);                     // 64 muestras por hardware
    INA.setBusVoltageConversionTime(4);
    INA.setShuntVoltageConversionTime(4);
  }

  // ---- QMC5883P ----
  if (!qmc.begin()) {
    Serial.println("QMC5883P: no conecta, revisa el cableado.");
  } else {
    qmc.setMode(QMC5883P_MODE_CONTINUOUS);   // necesario para que isDataReady() funcione
    qmc.setODR(QMC5883P_ODR_50HZ);
    qmc.setOSR(QMC5883P_OSR_8);
    qmc.setDSR(QMC5883P_DSR_1);
    qmc.setRange(QMC5883P_RANGE_12G);         // rango fijo: mejor resolución para el campo terrestre
    qmc.setSetResetMode(QMC5883P_SETRESET_ON);
    g_magOK = true;
    loadMagCalibration();
  }

  // ---- MPU6050 ----
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050: no conecta, revisa el cableado.");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    g_mpuOK = true;
  }

  xTaskCreatePinnedToCore(powerMonitorTask, "Power",     3072, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(telemetryTask,    "Telemetry", 6144, NULL, 1, NULL, 1);  // snprintf con floats necesita stack
  xTaskCreatePinnedToCore(serialCommandTask,"SerialCmd", 4096, NULL, 1, NULL, 1);

  Serial.println("Listo. Comandos: INVERT A|B|C|D, CAL VOLT <v>, CAL CURR <a>, CAL MAG, CAL MAGCOMP AUTO, CAL MAGCOMP X|Y|Z <v>, PLOT POWER|MAG|IMU, SHOW, RESET, TEST");
}

//===================== LOOP: SECUENCIA ADELANTE / PARO / ATRÁS =====================
// Ejecuta el trabajo pendiente (calibración o test) con los motores parados.
bool runPendingRequest() {
  Request r = g_request;
  if (r == REQ_NONE) return false;

  setAllMotors(0);
  if      (r == REQ_TEST)      runMotorTest();
  else if (r == REQ_CAL_MAG)   calibrateMagnetometer();
  else if (r == REQ_CAL_COMP)  calibrateMotorCompensation();

  g_request = REQ_NONE;
  Serial.println("Trabajo terminado, reanudando secuencia.");
  return true;
}

// Espera revisando cada 100 ms si llegó una solicitud
void interruptibleDelay(unsigned long ms) {
  for (unsigned long waited = 0; waited < ms && g_request == REQ_NONE; waited += 100) {
    delay(min(100UL, ms - waited));
  }
}

void loop() {
  // Sin Serial.println aquí para no ensuciar el Serial Plotter
  static const struct { int speed; unsigned long ms; } steps[] = {
    {  VEL, 10000 },   // adelante 10 s
    {    0,  3000 },   // paro 3 s
    { -VEL, 10000 },   // atrás 10 s
    {    0,  3000 },   // paro 3 s
  };

  for (const auto &s : steps) {
    if (runPendingRequest()) return;
    setAllMotors(s.speed);
    interruptibleDelay(s.ms);
  }
  runPendingRequest();
}
