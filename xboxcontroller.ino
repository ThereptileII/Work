#include <Bluepad32.h>
#include <math.h>  // for pow()

// --- Bluepad32 configuration ---
#define BP32_MAX_GAMEPADS 4

// --- DAC configuration for throttle output ---
#define THROTTLE_DAC_PIN      25   // ESP32 DAC1 on GPIO25

// --- Relay pin definitions ---
// Control relay (0x0040)
#define RELAY_CONTINUOUS_PIN  33
// Gear selection
#define DRIVE_RELAY_PIN       26
#define NEUTRAL_RELAY_PIN     27
#define REVERSE_RELAY_PIN     14
// Joystick horizontal relays
#define RELAY_JOY_RIGHT_PIN   32
#define RELAY_JOY_LEFT_PIN    35

// --- Global variables for Bluepad32 controllers ---
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// --- Throttle & gear control ---
int throttlePercent = 0;
bool dpadOverride   = false;
enum Gear { DRIVE, NEUTRAL, REVERSE };
Gear currentGear = DRIVE;

// Relay pulse timers
unsigned long driveRelayStart   = 0;
unsigned long neutralRelayStart = 0;
unsigned long reverseRelayStart = 0;

// Debounce for gear buttons
bool lastBtnDrive   = false;
bool lastBtnNeutral = false;
bool lastBtnReverse = false;

// Joystick-horizontal relay state
bool joyRightActive = false;
bool joyLeftActive  = false;

// Status print timing
unsigned long lastPrintTime = 0;

// Exponential mapping exponent
const float throttleExponent = 2.0;

// --- Exponential map ---
long mapExp(long x, long in_min, long in_max, long out_min, long out_max, float exponent) {
  float norm = (float)(x - in_min) / (in_max - in_min);
  norm = constrain(norm, 0.0f, 1.0f);
  norm = pow(norm, exponent);
  return out_min + norm * (out_max - out_min);
}

// --- Bluepad32 callbacks ---
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    if (!myControllers[i]) { myControllers[i] = ctl; break; }
}
void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    if (myControllers[i] == ctl) { myControllers[i] = nullptr; break; }
}

// --- Status print (includes DAC val & axisRY & axisX) ---
void printStatus(const char* trigger, int axisRY, int axisX) {
  int dacVal = map(throttlePercent, 0, 100, 0, 255);
  const char* gearName = (currentGear==DRIVE)?"DRIVE":(currentGear==NEUTRAL)?"NEUTRAL":"REVERSE";
  Serial.printf("[%lu] %s | Gear:%s | Th:%d%% | DAC:%d | Y:%d | X:%d\n",
                millis(), trigger, gearName, throttlePercent, dacVal, axisRY, axisX);
}

// --- Main processing ---
void processGamepad(ControllerPtr ctl) {
  uint16_t buttons = ctl->buttons();
  int axisRY = ctl->axisRY();
  int axisX  = ctl->axisX();

  // Continuous control button
  bool ctrl = buttons & 0x0040;
  digitalWrite(RELAY_CONTINUOUS_PIN, ctrl);
  if (ctrl && millis()-lastPrintTime>=500) {
    printStatus("0x0040 pressed", axisRY, axisX);
    lastPrintTime = millis();
  }

  // Gear selection (requires ctrl)
  if (ctrl) {
    bool btn = buttons & 0x0008;
    if (btn && !lastBtnDrive) {
      currentGear = DRIVE;
      printStatus("DRIVE+(ctrl)", axisRY, axisX);
      driveRelayStart = millis();
    }
    lastBtnDrive = btn;

    btn = buttons & 0x0004;
    if (btn && !lastBtnNeutral) {
      currentGear = NEUTRAL;
      printStatus("NEUTRAL", axisRY, axisX);
      neutralRelayStart = millis();
    }
    lastBtnNeutral = btn;

    btn = buttons & 0x0001;
    if (btn && !lastBtnReverse) {
      currentGear = REVERSE;
      printStatus("REVERSE", axisRY, axisX);
      reverseRelayStart = millis();
    }
    lastBtnReverse = btn;
  }

  // DRIVE mode: drive button always pulses
  if (currentGear==DRIVE && (buttons & 0x0008))
    driveRelayStart = millis();

  // Relay outputs (500ms pulse)
  digitalWrite(DRIVE_RELAY_PIN,   millis()-driveRelayStart   < 500);
  digitalWrite(NEUTRAL_RELAY_PIN, millis()-neutralRelayStart < 500);
  digitalWrite(REVERSE_RELAY_PIN, millis()-reverseRelayStart < 500);

  // Throttle logic (unchanged)
  const int dz = 10;
  if (currentGear==DRIVE) {
    if (axisRY<=-dz) {
      int v = -axisRY;
      throttlePercent = (v<20?1:mapExp(v-20,0,492,1,100,throttleExponent));
      dpadOverride=false;
    }
    else if (axisRY>=dz) {
      throttlePercent=0; dpadOverride=false;
    }
    else {
      uint8_t dp = ctl->dpad();
      if (dp==0x01) { throttlePercent=min(throttlePercent+10,100); dpadOverride=true; printStatus("D-PAD +",axisRY,axisX); }
      else if (dp==0x02){ throttlePercent=max(throttlePercent-10,0); dpadOverride=true; printStatus("D-PAD -",axisRY,axisX); }
      else if (!dpadOverride) throttlePercent=0;
    }
  }
  else if (currentGear==NEUTRAL) throttlePercent=0;
  else { // REVERSE
    if (axisRY>=dz) {
      throttlePercent = (axisRY<20?1:mapExp(axisRY-20,0,492,1,100,throttleExponent));
    } else throttlePercent=0;
  }

  // --- New: joystick X-axis relays with hysteresis ---
  const int onTh  = 500;
  const int offTh = 460;
  // Right relay
  if (!joyRightActive && axisX >= onTh) {
    joyRightActive = true;
    digitalWrite(RELAY_JOY_RIGHT_PIN, HIGH);
    printStatus("JOY RIGHT ON", axisRY, axisX);
  } else if (joyRightActive && axisX < offTh) {
    joyRightActive = false;
    digitalWrite(RELAY_JOY_RIGHT_PIN, LOW);
    printStatus("JOY RIGHT OFF", axisRY, axisX);
  }
  // Left relay
  if (!joyLeftActive && axisX <= -onTh) {
    joyLeftActive = true;
    digitalWrite(RELAY_JOY_LEFT_PIN, HIGH);
    printStatus("JOY LEFT ON", axisRY, axisX);
  } else if (joyLeftActive && axisX > -offTh) {
    joyLeftActive = false;
    digitalWrite(RELAY_JOY_LEFT_PIN, LOW);
    printStatus("JOY LEFT OFF", axisRY, axisX);
  }

  // Output throttle via DAC
  int dacVal = map(throttlePercent, 0, 100, 0, 255);
  dacWrite(THROTTLE_DAC_PIN, dacVal);

  // Periodic status
  if (millis()-lastPrintTime>=500) {
    printStatus("UPDATE", axisRY, axisX);
    lastPrintTime = millis();
  }
}

void processControllers() {
  for (auto ctl: myControllers)
    if (ctl && ctl->isConnected() && ctl->hasData() && ctl->isGamepad())
      processGamepad(ctl);
}

void setup() {
  Serial.begin(115200);
  throttlePercent = 0;

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  // Relay pins
  pinMode(RELAY_CONTINUOUS_PIN, OUTPUT); digitalWrite(RELAY_CONTINUOUS_PIN, LOW);
  pinMode(DRIVE_RELAY_PIN, OUTPUT);      digitalWrite(DRIVE_RELAY_PIN, LOW);
  pinMode(NEUTRAL_RELAY_PIN, OUTPUT);    digitalWrite(NEUTRAL_RELAY_PIN, LOW);
  pinMode(REVERSE_RELAY_PIN, OUTPUT);    digitalWrite(REVERSE_RELAY_PIN, LOW);
  pinMode(RELAY_JOY_RIGHT_PIN, OUTPUT);  digitalWrite(RELAY_JOY_RIGHT_PIN, LOW);
  pinMode(RELAY_JOY_LEFT_PIN, OUTPUT);   digitalWrite(RELAY_JOY_LEFT_PIN, LOW);
}

void loop() {
  if (BP32.update()) processControllers();
  delay(150);
}
