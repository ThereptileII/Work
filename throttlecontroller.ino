#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <math.h>

#define TCAADDR 0x70

Adafruit_MCP4725 dac1; // on channel 3
Adafruit_MCP4725 dac2; // on channel 4

void tcaselect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

const int potPin      = A0;
const int fallbackPin = A1;

// DAC1 → 0.76–3.8 V, DAC2 → 0.38–1.9 V
const int dac1_min = 622, dac1_max = 3110;
const int dac2_min = 311, dac2_max = 1556;

const float alpha = 0.1f;  // EMA smoothing
float filtPot1 = 0, filtPot2 = 0;
float filtFb1  = 0, filtFb2  = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();

  tcaselect(3);
  dac1.begin(0x60);
  tcaselect(4);
  dac2.begin(0x60);
  tcaselect(0);

  Serial.println("Setup complete: A0=primary, A1=fallback.");
}

void loop() {
  int rawA0 = analogRead(potPin);
  int rawA1 = analogRead(fallbackPin);

  // use A0 if turned at all, else A1
  bool usePot = rawA0 > 10;
  float rawVal = usePot ? rawA0 : rawA1;
  String src = usePot ? "A0" : "A1";

  // EMA smoothing
  if (usePot) {
    filtPot1 = alpha * rawVal + (1 - alpha) * filtPot1;
    filtPot2 = alpha * rawVal + (1 - alpha) * filtPot2;
  } else {
    filtFb1 = alpha * rawVal + (1 - alpha) * filtFb1;
    filtFb2 = alpha * rawVal + (1 - alpha) * filtFb2;
  }
  float f1 = usePot ? filtPot1 : filtFb1;
  float f2 = usePot ? filtPot2 : filtFb2;

  // parabolic mapping
  int dacValue1 = parabolicScaleToDAC(f1, 0, 1023, dac1_min, dac1_max);
  int dacValue2 = parabolicScaleToDAC(f2, 0, 1023, dac2_min, dac2_max);

  // expected voltages
  float v1 = (5.0 * dacValue1) / 4095.0;
  float v2 = (5.0 * dacValue2) / 4095.0;

  // write to DACs
  tcaselect(3);
  dac1.setVoltage(dacValue1, false);
  tcaselect(4);
  dac2.setVoltage(dacValue2, false);

  // single-line serial output
  Serial.print(src);
  Serial.print(" raw:");
  Serial.print((int)rawVal);
  Serial.print(" filt1:");
  Serial.print(f1, 2);
  Serial.print(" dac1:");
  Serial.print(dacValue1);
  Serial.print("(");
  Serial.print(v1, 3);
  Serial.print("V) filt2:");
  Serial.print(f2, 2);
  Serial.print(" dac2:");
  Serial.print(dacValue2);
  Serial.print("(");
  Serial.print(v2, 3);
  Serial.println("V)");

  delay(100);
}

int parabolicScaleToDAC(float input, int in_min, int in_max, int out_min, int out_max) {
  input = constrain(input, in_min, in_max);
  float norm = (input - in_min) / float(in_max - in_min);
  float curve = norm * norm;
  float val = curve * (out_max - out_min) + out_min;
  return constrain(int(val), 0, 4095);
}
