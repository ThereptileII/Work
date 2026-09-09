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
const int inputThreshold = 10;
// The ESP32 DAC is 0-3.3 V, while the Nano ADC uses its nominal 5 V supply as
// reference: 1023 * 3.3 / 5.0 = 675 ADC counts at full throttle.
const int inputFullScale = 675;

// DAC2 → approximately 0.38–1.9 V; DAC1 is always exactly twice DAC2.
// The released code is calibrated for this installed pair. Codes 295/590
// measured 0.40/0.78 V, so reduce them to 282/564 for about 0.38/0.75 V.
const int dac2_min = 282, dac2_max = 1556;
const int dac1_released = dac2_min * 2;
const float dacSupplyVolts = 5.0f;

const float alpha = 0.1f;  // EMA smoothing
float filtPot1 = 0, filtPot2 = 0;
float filtFb1  = 0, filtFb2  = 0;

void writeThrottleDacs(int dacValue1, int dacValue2) {
  tcaselect(3);
  dac1.setVoltage(dacValue1, false);
  tcaselect(4);
  dac2.setVoltage(dacValue2, false);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();

  tcaselect(3);
  dac1.begin(0x60);
  tcaselect(4);
  dac2.begin(0x60);

  // Present a valid released-pedal signal as soon as the DACs are available.
  // A 0 V / 0 V pair is an implausible pedal signal to the Resolve controller.
  writeThrottleDacs(dac1_released, dac2_min);
  tcaselect(0);

  Serial.println("Setup complete: A0=primary, A1=fallback, idle=0.76V/0.38V.");
}

void loop() {
  int rawA0 = analogRead(potPin);
  int rawA1 = analogRead(fallbackPin);

  // Use A0 if active, otherwise A1. No input means a released pedal, not 0 V.
  bool usePot = rawA0 > inputThreshold;
  bool useFallback = !usePot && rawA1 > inputThreshold;
  bool hasInput = usePot || useFallback;
  float rawVal = usePot ? rawA0 : (useFallback ? rawA1 : 0);
  const char* src = usePot ? "A0" : (useFallback ? "A1" : "NONE");

  // EMA smoothing
  if (!hasInput) {
    // Prevent a stale throttle value from returning when an input reconnects.
    filtPot1 = filtPot2 = 0;
    filtFb1 = filtFb2 = 0;
  } else if (usePot) {
    filtPot1 = alpha * rawVal + (1 - alpha) * filtPot1;
    filtPot2 = alpha * rawVal + (1 - alpha) * filtPot2;
  } else {
    filtFb1 = alpha * rawVal + (1 - alpha) * filtFb1;
    filtFb2 = alpha * rawVal + (1 - alpha) * filtFb2;
  }
  float f1 = usePot ? filtPot1 : filtFb1;
  float f2 = usePot ? filtPot2 : filtFb2;

  // Parabolic mapping. Derive DAC1 from DAC2 for an exact 2:1 code ratio.
  int dacValue2 = hasInput
                    ? parabolicScaleToDAC(f2, 0, inputFullScale,
                                          dac2_min, dac2_max)
                    : dac2_min;
  int dacValue1 = constrain(dacValue2 * 2, 0, 4095);

  // expected voltages
  float v1 = (dacSupplyVolts * dacValue1) / 4095.0;
  float v2 = (dacSupplyVolts * dacValue2) / 4095.0;

  // write to DACs
  writeThrottleDacs(dacValue1, dacValue2);

  // single-line serial output
  Serial.print(src);
  Serial.print(" rawA0:");
  Serial.print(rawA0);
  Serial.print(" rawA1:");
  Serial.print(rawA1);
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
