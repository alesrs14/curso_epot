#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---------------------- CONFIGURACIÓN DE HARDWARE ----------------------
#define PIN_V     A0     // Voltaje (Vpp=2V, offset=1.0V)
#define PIN_I     A1     // "Corriente" en forma de voltaje (Vpp=3V, offset=1.5V)
#define LCD_ADDR  0x27   // Cambia a 0x3F si tu LCD usa esa dirección

// ---------------------- AJUSTES DE SEÑAL / CALIBRACIÓN -----------------
// Offsets EN VOLTIOS de cada canal (los que entrega tu generador):
const float V_OFFSET_V = 1.0f;   // Offset del canal de tensión
const float I_OFFSET_V = 1.5f;   // Offset del canal de corriente (voltaje shunt)

// Escalas para pasar de voltios de entrada ADC a unidades "reales":
//   KV = factor para el canal de tensión (por ej., relación de trafo o divisor).
//   KI = factor para el canal de corriente (por ej., 1/(Rshunt * G)).
const float KV = 1.0f;           // V_reales = KV * V_medidos
const float KI = 1.0f;           // I_reales = KI * V_shunt_medidos

// Número de ciclos a integrar (más ciclos = más estable, pero más lento)
const int   N_CYCLES = 10;

// Límites de tiempo para evitar bloqueos si no hay cruces por cero
const unsigned long ZC_TIMEOUT_MS   = 1000;   // esperar cruce inicial
const unsigned long MEAS_TIMEOUT_MS = 4000;   // medición completa

// ---------------------- LCD y variables de pantalla --------------------
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
unsigned long lastScreenMs = 0;
uint8_t screenIdx = 0;
const unsigned long SCREEN_PERIOD_MS = 2000;

// ---------------------- Utilidades ADC ----------------------
inline float adcToVolts(int adc) {
  // ADC 10-bit, referencia 5.0V (Arduino UNO por defecto)
  return (adc * (5.0f / 1023.0f));
}

inline float readChannelVolts(uint8_t pin) {
  return adcToVolts(analogRead(pin));
}

// En lugar de lambdas, definimos funciones "callback" tradicionales
float readVFunc() { return readChannelVolts(PIN_V); }
float readIFunc() { return readChannelVolts(PIN_I); }

// ---------------------- Espera cruce por cero ascendente ----------------
bool waitForZeroCrossRising(float (*readV)(), float vOffset, unsigned long timeoutMs) {
  unsigned long t0 = millis();
  float prev = readV() - vOffset;

  // Espera a que esté en negativo
  while (prev >= 0.0f) {
    if (millis() - t0 > timeoutMs) return false;
    prev = readV() - vOffset;
  }
  // Espera a que pase a positivo (cruce ascendente)
  while (true) {
    if (millis() - t0 > timeoutMs) return false;
    float now = readV() - vOffset;
    if (now >= 0.0f && prev < 0.0f) return true;
    prev = now;
  }
}

// ---------------------- Medición por ciclos ----------------------------
bool measureCycles(
  int nCycles,
  float (*readV)(), float vOffset,
  float (*readI)(), float iOffset,
  float &Vrms, float &Irms, float &P, float &S, float &Q, float &PF,
  float &phiDeg, int &phiSign  // phiSign: -1 = LEAD, +1 = LAG, 0 = desconocido
) {
  unsigned long tStart = millis();

  // Sincroniza con el primer cruce ascendente de V
  if (!waitForZeroCrossRising(readV, vOffset, ZC_TIMEOUT_MS)) return false;

  // Variables de acumulación
  double sumV2 = 0.0, sumI2 = 0.0, sumP = 0.0;
  unsigned long nSamples = 0;

  // Para estimar signo de fase
  bool vFirstCrossSeen = false, iFirstCrossSeen = false;
  long  vFirstIndex = -1, iFirstIndex = -1;
  long  sampleIndex = 0;

  int cyclesDone = 0;
  float prevV = readV() - vOffset;
  float prevI = readI() - iOffset;

  while (cyclesDone < nCycles) {
    if (millis() - tStart > MEAS_TIMEOUT_MS) return false;

    float v = readV() - vOffset;  // centrado
    float i = readI() - iOffset;  // centrado

    // Escalas físicas
    float vScaled = KV * v;       // V reales (o unidades elegidas)
    float iScaled = KI * i;       // A reales (o unidades elegidas)

  // >>>>>>>>>> AÑADE ESTO PARA PLOTEAR LAS SEÑALES EN TIEMPO REAL <<<<<<<<<<
  // Enviar dos series: V e I (con etiquetas para leyenda)
  Serial.print("V:"); Serial.print(vScaled, 4);
  Serial.print('\t'); // separador (tab)
  Serial.print("I:"); Serial.print(iScaled, 4);
  Serial.print('\n'); // fin de muestra (nueva línea)

    // Acumulación
    sumV2 += (double)vScaled * vScaled;
    sumI2 += (double)iScaled * iScaled;
    sumP  += (double)vScaled * iScaled;
    nSamples++;
    sampleIndex++;

    // Cruce ascendente de V cuenta ciclos
    if (!vFirstCrossSeen && prevV < 0.0f && v >= 0.0f) {
      vFirstCrossSeen = true;
      vFirstIndex = sampleIndex;
      cyclesDone++;
    } else if (prevV < 0.0f && v >= 0.0f) {
      cyclesDone++;
    }

    // Primer cruce ascendente de I para el signo
    if (!iFirstCrossSeen && prevI < 0.0f && i >= 0.0f) {
      iFirstCrossSeen = true;
      iFirstIndex = sampleIndex;
    }

    prevV = v;
    prevI = i;
  }

  if (nSamples == 0) return false;

  Vrms = sqrt(sumV2 / (double)nSamples);
  Irms = sqrt(sumI2 / (double)nSamples);
  P    = (float)(sumP / (double)nSamples);
  S    = Vrms * Irms;

  // PF puede ser negativo si I está 180° invertida; acotamos numéricamente
  PF   = (S > 1e-9f) ? (P / S) : 0.0f;
  if (PF >  1.0f) PF =  1.0f;
  if (PF < -1.0f) PF = -1.0f;

  // Q a partir de S^2 = P^2 + Q^2 (para senoides)
  float rad = S*S - P*P;
  Q = (rad > 0.0f) ? sqrtf(rad) : 0.0f;

  // Ángulo de fase (módulo) y signo
  float PFabs = fabsf(PF);
  if (PFabs > 1.0f) PFabs = 1.0f;
  phiDeg = acosf(PFabs) * 180.0f / PI;

  // Signo: si I cruza antes → LEAD (capacitivo, negativo). Si después → LAG (inductivo, positivo).
  phiSign = 0;
  if (vFirstCrossSeen && iFirstCrossSeen) {
    if (iFirstIndex < vFirstIndex) phiSign = -1;  // LEAD
    else if (iFirstIndex > vFirstIndex) phiSign = +1; // LAG
    else phiSign = 0;
  }

  return true;
}

// ---------------------- Ciclo principal ----------------------
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Wattimetro");
  lcd.setCursor(0, 1); lcd.print("Simulacion I2C");
  delay(1200);
  lcd.clear();
}

void loop() {
  float Vrms=0, Irms=0, P=0, S=0, Q=0, PF=0, phiDeg=0;
  int phiSign = 0;

  bool ok = measureCycles(
    N_CYCLES,
    readVFunc, V_OFFSET_V,
    readIFunc, I_OFFSET_V,
    Vrms, Irms, P, S, Q, PF, phiDeg, phiSign
  );

  unsigned long now = millis();
  if (now - lastScreenMs > SCREEN_PERIOD_MS) {
    screenIdx = (screenIdx + 1) % 3;
    lastScreenMs = now;
  }

  lcd.clear();

  if (!ok) {
    lcd.setCursor(0, 0); lcd.print("No hay cruce");
    lcd.setCursor(0, 1); lcd.print("Ver senal/frec");
    delay(300);
    return;
  }

  // Texto LEAD/LAG
  char signTxt[6] = "";
  if (phiSign < 0)      strcpy(signTxt, "LEAD");
  else if (phiSign > 0) strcpy(signTxt, "LAG ");
  else                  strcpy(signTxt, "N/A ");

  switch (screenIdx) {
    case 0: {
      // P y PF / phi
      lcd.setCursor(0, 0);
      lcd.print("P:");
      lcd.print(P, 3); // más decimales en simulación
      lcd.print("W");
      lcd.setCursor(0, 1);
      lcd.print("PF:");
      lcd.print(PF, 3);
      lcd.print(" ");
      lcd.print((char)223); // simbolo de grado (HD44780)
      lcd.print(":");
      lcd.print(phiDeg, 1);
      break;
    }
    case 1: {
      // S y Q
      lcd.setCursor(0, 0);
      lcd.print("S:");
      lcd.print(S, 3);
      lcd.print("VA");
      lcd.setCursor(0, 1);
      lcd.print("Q:");
      lcd.print(Q, 3);
      lcd.print("var");
      break;
    }
    case 2: {
      // Vrms e Irms + signo fase
      lcd.setCursor(0, 0);
      lcd.print("V:");
      lcd.print(Vrms, 3);
      lcd.print("Vr");
      lcd.setCursor(0, 1);
      lcd.print("I:");
      lcd.print(Irms, 3);
      lcd.print("Ar ");
      lcd.print(signTxt);
      break;
    }
  }

  delay(5000);
}