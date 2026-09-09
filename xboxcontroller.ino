#include <Bluepad32.h>

// --- Output pins ---
constexpr uint8_t THROTTLE_DAC_PIN = 25;  // ESP32 DAC1
constexpr uint8_t BRAKE_RELAY_PIN = 33;
constexpr uint8_t DRIVE_RELAY_PIN = 26;
constexpr uint8_t NEUTRAL_RELAY_PIN = 27;
constexpr uint8_t REVERSE_RELAY_PIN = 14;
constexpr uint8_t RELAY_JOY_RIGHT_PIN = 32;
constexpr uint8_t RELAY_JOY_LEFT_PIN = 19;

// --- Control timing and thresholds ---
constexpr int AXIS_DEADZONE = 30;
constexpr int AXIS_CURVE_START = 30;
constexpr int AXIS_MAX = 512;
constexpr int DIRECTION_CONFLICT_THRESHOLD = 96;
constexpr int DPAD_STEP_PERCENT = 10;
constexpr int JOY_ON_THRESHOLD = 500;
constexpr int JOY_OFF_THRESHOLD = 460;
// Give the downstream Nano at least two of its 100 ms update cycles to expose
// a released-pedal output before Resolve sees a gear-selection pulse.
constexpr uint32_t BRAKE_SETTLE_MS = 300;
constexpr uint32_t GEAR_PULSE_MS = 500;
constexpr uint32_t CONTROLLER_TIMEOUT_MS = 1500;
constexpr uint32_t STATUS_INTERVAL_MS = 500;
constexpr uint32_t LED_TEST_START_DELAY_MS = 750;
constexpr uint32_t LED_TEST_STEP_MS = 1500;

enum Gear : uint8_t { DRIVE, NEUTRAL, REVERSE };
enum ThrottleSource : uint8_t {
  THROTTLE_SOURCE_NONE,
  THROTTLE_SOURCE_JOYSTICK,
  THROTTLE_SOURCE_DPAD
};

struct ControllerState {
  bool connected = false;
  bool valid = false;
  uint32_t lastDataMs = 0;
  int axisRY = 0;
  int axisX = 0;
  uint8_t dpad = 0;
  bool brake = false;
  bool neutral = false;
};

struct ControllerLedTestState {
  bool active = false;
  uint8_t phase = 0;
  uint32_t nextStepMs = 0;
};

struct CombinedInput {
  bool anyFresh = false;
  bool brake = false;
  bool neutral = false;
  bool directionConflict = false;
  uint8_t activeControllers = 0;
  int forwardMagnitude = 0;
  int reverseMagnitude = 0;
  int maxRightX = 0;
  int minLeftX = 0;
  int representativeRY = 0;
  int representativeX = 0;
};

ControllerPtr myControllers[BP32_MAX_GAMEPADS] = {};
ControllerState controllerStates[BP32_MAX_GAMEPADS] = {};
ControllerLedTestState controllerLedTests[BP32_MAX_GAMEPADS] = {};

Gear currentGear = NEUTRAL;
Gear pendingGear = NEUTRAL;
bool gearKnown = false;
bool gearPulseActive = false;
bool gearChangeAttemptedDuringBrake = false;
uint32_t gearPulseStartMs = 0;

bool brakeActive = false;
uint32_t brakeActiveSinceMs = 0;

int throttlePercent = 0;
bool throttleDacEnabled = false;
bool throttleOutputIsHardLow = false;
int dpadThrottlePercent = 0;  // -100 (reverse) to +100 (drive)
bool dpadOverride = false;
ThrottleSource throttleSource = THROTTLE_SOURCE_NONE;
bool throttleSourceChanged = false;

bool joyRightActive = false;
bool joyLeftActive = false;
bool controllerDataLost = false;
uint32_t lastPrintTime = 0;

const char* gearName(Gear gear) {
  switch (gear) {
    case DRIVE:
      return "DRIVE";
    case NEUTRAL:
      return "NEUTRAL";
    case REVERSE:
      return "REVERSE";
  }
  return "UNKNOWN";
}

void setAllGearRelays(bool level) {
  digitalWrite(DRIVE_RELAY_PIN, level);
  digitalWrite(NEUTRAL_RELAY_PIN, level);
  digitalWrite(REVERSE_RELAY_PIN, level);
}

void setGearRelay(Gear gear, bool level) {
  switch (gear) {
    case DRIVE:
      digitalWrite(DRIVE_RELAY_PIN, level);
      break;
    case NEUTRAL:
      digitalWrite(NEUTRAL_RELAY_PIN, level);
      break;
    case REVERSE:
      digitalWrite(REVERSE_RELAY_PIN, level);
      break;
  }
}

void setThrottleOutput(int percent) {
  throttlePercent = constrain(percent, 0, 100);

  if (throttlePercent == 0) {
    // The ESP32 DAC buffer can sit above ground even at code 0. Disable the
    // analog peripheral and actively drive GPIO25 LOW for a true zero output.
    if (throttleDacEnabled) {
      dacDisable(THROTTLE_DAC_PIN);
      throttleDacEnabled = false;
    }
    if (!throttleOutputIsHardLow) {
      pinMode(THROTTLE_DAC_PIN, OUTPUT);
      digitalWrite(THROTTLE_DAC_PIN, LOW);
      throttleOutputIsHardLow = true;
    }
    return;
  }

  const int dacValue = map(throttlePercent, 0, 100, 0, 255);
  dacWrite(THROTTLE_DAC_PIN, dacValue);
  throttleDacEnabled = true;
  throttleOutputIsHardLow = false;
}

int mapAxisMagnitudeToPercent(int magnitude) {
  magnitude = constrain(magnitude, AXIS_DEADZONE, AXIS_MAX);
  if (magnitude < AXIS_CURVE_START) {
    return 1;
  }

  const float norm =
      float(magnitude - AXIS_CURVE_START) / float(AXIS_MAX - AXIS_CURVE_START);
  return 1 + int(norm * norm * 99.0f);
}

void clearDpadThrottle() {
  dpadThrottlePercent = 0;
  dpadOverride = false;
}

void resetThrottleCommand() {
  clearDpadThrottle();
  throttleSource = THROTTLE_SOURCE_NONE;
  throttleSourceChanged = true;
  setThrottleOutput(0);
}

void selectThrottleSource(ThrottleSource nextSource) {
  if (throttleSource == nextSource) {
    return;
  }

  // Never carry a latched value from one input method into the other.
  clearDpadThrottle();
  throttleSource = nextSource;
  throttleSourceChanged = true;
  setThrottleOutput(0);
}

const char* throttleSourceName() {
  switch (throttleSource) {
    case THROTTLE_SOURCE_JOYSTICK:
      return "JOYSTICK";
    case THROTTLE_SOURCE_DPAD:
      return "DPAD";
    case THROTTLE_SOURCE_NONE:
      return "NONE";
  }
  return "NONE";
}

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (!myControllers[i]) {
      myControllers[i] = ctl;
      controllerStates[i] = ControllerState{};
      controllerStates[i].connected = true;
      controllerLedTests[i] = ControllerLedTestState{};
      controllerLedTests[i].active = true;
      controllerLedTests[i].nextStepMs = millis() + LED_TEST_START_DELAY_MS;

      const ControllerProperties properties = ctl->getProperties();
      Serial.printf(
          "Controller connected in slot %d: model=%s VID=0x%04x "
          "PID=0x%04x flags=0x%04x [rumble:%s player-leds:%s rgb:%s]\n",
          i, ctl->getModelName().c_str(), properties.vendor_id,
          properties.product_id, properties.flags,
          (properties.flags & (1u << 0)) ? "yes" : "no",
          (properties.flags & (1u << 1)) ? "yes" : "no",
          (properties.flags & (1u << 2)) ? "yes" : "no");
      return;
    }
  }
  Serial.println("Controller connected, but no free slot is available");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      controllerStates[i] = ControllerState{};
      controllerLedTests[i] = ControllerLedTestState{};
      resetThrottleCommand();
      controllerDataLost = true;
      Serial.printf("Controller disconnected from slot %d\n", i);
      return;
    }
  }
}

void serviceControllerLedTests(uint32_t now) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    ControllerLedTestState& test = controllerLedTests[i];
    ControllerPtr ctl = myControllers[i];
    if (!test.active) {
      continue;
    }
    if (!ctl || !ctl->isConnected()) {
      test = ControllerLedTestState{};
      continue;
    }
    if ((int32_t)(now - test.nextStepMs) < 0) {
      continue;
    }

    switch (test.phase) {
      case 0:
        ctl->setColorLED(255, 0, 0);
        Serial.printf("LED test slot %d: requested RED\n", i);
        break;
      case 1:
        ctl->setColorLED(0, 255, 0);
        Serial.printf("LED test slot %d: requested GREEN\n", i);
        break;
      case 2:
        ctl->setColorLED(0, 0, 255);
        Serial.printf("LED test slot %d: requested BLUE\n", i);
        break;
      case 3:
        ctl->setPlayerLEDs(1u << (i % 4));
        Serial.printf("LED test slot %d: requested player LED %d\n", i,
                      i + 1);
        break;
      default:
        test.active = false;
        Serial.printf("LED test slot %d complete\n", i);
        continue;
    }

    test.phase++;
    test.nextStepMs = now + LED_TEST_STEP_MS;
  }
}

void updateControllerStates(uint32_t now, bool& dpadUpPressed,
                            bool& dpadDownPressed) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    ControllerPtr ctl = myControllers[i];
    if (!ctl || !ctl->isConnected() || !ctl->isGamepad() || !ctl->hasData()) {
      continue;
    }

    ControllerState& state = controllerStates[i];
    const uint8_t newDpad = ctl->dpad();
    dpadUpPressed |=
        (newDpad & DPAD_UP) != 0 && (state.dpad & DPAD_UP) == 0;
    dpadDownPressed |=
        (newDpad & DPAD_DOWN) != 0 && (state.dpad & DPAD_DOWN) == 0;

    state.connected = true;
    state.valid = true;
    state.lastDataMs = now;
    state.axisRY = ctl->axisRY();
    state.axisX = ctl->axisX();
    state.dpad = newDpad;
    state.brake = ctl->l2();
    state.neutral = ctl->thumbR();
  }
}

bool expireStaleControllerData(uint32_t now) {
  bool expired = false;
  for (ControllerState& state : controllerStates) {
    if (state.valid && now - state.lastDataMs > CONTROLLER_TIMEOUT_MS) {
      const bool connected = state.connected;
      state = ControllerState{};
      state.connected = connected;
      expired = true;
    }
  }
  return expired;
}

bool anyControllerConnected() {
  for (ControllerPtr ctl : myControllers) {
    if (ctl && ctl->isConnected()) {
      return true;
    }
  }
  return false;
}

CombinedInput combineControllerStates() {
  CombinedInput input;

  for (const ControllerState& state : controllerStates) {
    if (!state.valid) {
      continue;
    }

    input.anyFresh = true;
    input.activeControllers++;
    input.brake |= state.brake;
    input.neutral |= state.neutral;

    if (state.axisRY <= -AXIS_DEADZONE) {
      input.forwardMagnitude = max(input.forwardMagnitude, -state.axisRY);
    } else if (state.axisRY >= AXIS_DEADZONE) {
      input.reverseMagnitude = max(input.reverseMagnitude, state.axisRY);
    }

    input.maxRightX = max(input.maxRightX, state.axisX);
    input.minLeftX = min(input.minLeftX, state.axisX);
  }

  // Ignore normal center drift from another controller. Two deliberate,
  // opposing inputs still stop throttle and block a gear change.
  input.directionConflict =
      input.forwardMagnitude >= DIRECTION_CONFLICT_THRESHOLD &&
      input.reverseMagnitude >= DIRECTION_CONFLICT_THRESHOLD;

  if (!input.directionConflict) {
    if (input.forwardMagnitude > input.reverseMagnitude) {
      input.reverseMagnitude = 0;
      input.representativeRY = -input.forwardMagnitude;
    } else if (input.reverseMagnitude > input.forwardMagnitude) {
      input.forwardMagnitude = 0;
      input.representativeRY = input.reverseMagnitude;
    } else {
      input.forwardMagnitude = 0;
      input.reverseMagnitude = 0;
    }
  }

  input.representativeX =
      input.maxRightX >= -input.minLeftX ? input.maxRightX : input.minLeftX;
  return input;
}

void updateDpadThrottle(bool upPressed, bool downPressed, bool locked) {
  if (locked) {
    return;
  }
  if (upPressed == downPressed) {
    return;
  }

  selectThrottleSource(THROTTLE_SOURCE_DPAD);
  dpadOverride = true;

  // Do not jump directly across zero after a throttle reset. The D-pad must
  // select physical Neutral between Drive and Reverse before it can continue
  // into the opposite direction.
  const bool holdAtNeutral =
      dpadThrottlePercent == 0 && gearKnown &&
      ((currentGear == DRIVE && downPressed) ||
       (currentGear == REVERSE && upPressed));
  if (holdAtNeutral) {
    Serial.println("D-pad throttle: +0% (NEUTRAL gate)");
    return;
  }

  if (upPressed) {
    dpadThrottlePercent =
        min(dpadThrottlePercent + DPAD_STEP_PERCENT, 100);
    Serial.printf("D-pad throttle: %+d%%\n", dpadThrottlePercent);
  } else {
    dpadThrottlePercent =
        max(dpadThrottlePercent - DPAD_STEP_PERCENT, -100);
    Serial.printf("D-pad throttle: %+d%%\n", dpadThrottlePercent);
  }
}

int calculateSignedThrottle(const CombinedInput& input, bool& hasGearRequest,
                            Gear& requestedGear) {
  hasGearRequest = false;

  if (!input.anyFresh) {
    resetThrottleCommand();
    return 0;
  }

  if (input.neutral) {
    resetThrottleCommand();
    hasGearRequest = true;
    requestedGear = NEUTRAL;
    return 0;
  }

  const bool hasAnalogDirection =
      input.forwardMagnitude > 0 || input.reverseMagnitude > 0;
  if (hasAnalogDirection) {
    selectThrottleSource(THROTTLE_SOURCE_JOYSTICK);
    if (input.directionConflict) {
      return 0;
    }

    hasGearRequest = true;
    if (input.forwardMagnitude > 0) {
      requestedGear = DRIVE;
      return mapAxisMagnitudeToPercent(input.forwardMagnitude);
    }

    requestedGear = REVERSE;
    return -mapAxisMagnitudeToPercent(input.reverseMagnitude);
  }

  if (!dpadOverride) {
    return 0;
  }

  hasGearRequest = true;
  if (dpadThrottlePercent > 0) {
    requestedGear = DRIVE;
  } else if (dpadThrottlePercent < 0) {
    requestedGear = REVERSE;
  } else {
    requestedGear = NEUTRAL;
  }
  return dpadThrottlePercent;
}

void updateBrakeRelay(bool requested, uint32_t now) {
  if (requested == brakeActive) {
    return;
  }

  // Remove throttle before asserting the brake input. Resolve requires the
  // pedal to be fully released as well as the brake to be active when shifting.
  if (requested) {
    setThrottleOutput(0);
  }

  brakeActive = requested;
  // Permit at most one gear-change pulse for each deliberate brake press.
  gearChangeAttemptedDuringBrake = false;
  digitalWrite(BRAKE_RELAY_PIN, brakeActive ? HIGH : LOW);
  if (brakeActive) {
    brakeActiveSinceMs = now;
    Serial.println("BRAKE ON");
  } else {
    Serial.println("BRAKE OFF");
  }
}

void cancelGearPulse(const char* reason) {
  if (!gearPulseActive) {
    return;
  }

  setAllGearRelays(LOW);
  gearPulseActive = false;
  gearKnown = false;
  resetThrottleCommand();
  Serial.printf("Gear pulse aborted: %s\n", reason);
}

void startGearPulse(Gear gear, uint32_t now) {
  resetThrottleCommand();
  setAllGearRelays(LOW);
  pendingGear = gear;
  gearPulseStartMs = now;
  gearPulseActive = true;
  gearChangeAttemptedDuringBrake = true;
  setGearRelay(pendingGear, HIGH);
  Serial.printf("%s gear pulse started\n", gearName(pendingGear));
}

void serviceGearChange(uint32_t now, bool hasGearRequest,
                       Gear requestedGear, bool directionConflict) {
  if (gearPulseActive) {
    if (!brakeActive) {
      cancelGearPulse("brake released");
    } else if (directionConflict) {
      cancelGearPulse("conflicting direction commands");
    } else if (now - gearPulseStartMs >= GEAR_PULSE_MS) {
      setAllGearRelays(LOW);
      gearPulseActive = false;
      currentGear = pendingGear;
      gearKnown = true;
      resetThrottleCommand();
      // This is the commanded state. Resolve can still reject the pulse if one
      // of its interlocks is not satisfied; this ESP32 has no gear feedback.
      Serial.printf("%s gear pulse completed (not feedback-confirmed)\n",
                    gearName(currentGear));
    }
    return;
  }

  if (!brakeActive || !hasGearRequest || gearChangeAttemptedDuringBrake) {
    return;
  }
  // Re-sending Drive would toggle Resolve's regen setting, so suppress
  // duplicate Drive/Reverse requests. Allow one fresh Neutral pulse per brake
  // press because the previous Neutral pulse may have been rejected while the
  // ESP32 still recorded it as completed.
  if (gearKnown && requestedGear == currentGear && requestedGear != NEUTRAL) {
    return;
  }
  if (now - brakeActiveSinceMs < BRAKE_SETTLE_MS) {
    return;
  }

  startGearPulse(requestedGear, now);
}

void updateJoystickRelays(const CombinedInput& input) {
  bool nextRight = joyRightActive
                       ? input.maxRightX >= JOY_OFF_THRESHOLD
                       : input.maxRightX >= JOY_ON_THRESHOLD;
  bool nextLeft = joyLeftActive
                      ? input.minLeftX <= -JOY_OFF_THRESHOLD
                      : input.minLeftX <= -JOY_ON_THRESHOLD;

  // Opposing commands from different gamepads cancel each other.
  if (nextRight && nextLeft) {
    nextRight = false;
    nextLeft = false;
  }

  if (nextRight != joyRightActive) {
    joyRightActive = nextRight;
    digitalWrite(RELAY_JOY_RIGHT_PIN, joyRightActive ? HIGH : LOW);
    Serial.printf("JOY RIGHT %s\n", joyRightActive ? "ON" : "OFF");
  }
  if (nextLeft != joyLeftActive) {
    joyLeftActive = nextLeft;
    digitalWrite(RELAY_JOY_LEFT_PIN, joyLeftActive ? HIGH : LOW);
    Serial.printf("JOY LEFT %s\n", joyLeftActive ? "ON" : "OFF");
  }
}

void printStatus(const CombinedInput& input, int signedThrottle,
                 const char* throttleBlockReason) {
  const int dacValue = map(throttlePercent, 0, 100, 0, 255);
  const char* selectedGear = gearKnown ? gearName(currentGear) : "UNKNOWN";
  const char* brakeStatus =
      brakeActive ? (input.brake ? "ON" : "HELD") : "OFF";
  Serial.printf(
      "[%lu] GearCmd:%s | Brake:%s | Cmd:%+d%% | Out:%d%% | DAC:%d | "
      "Pads:%u | Y:%d | X:%d | Source:%s | Block:%s%s\n",
      millis(), selectedGear, brakeStatus, signedThrottle, throttlePercent,
      dacValue, input.activeControllers,
      input.representativeRY, input.representativeX, throttleSourceName(),
      throttleBlockReason,
      input.directionConflict ? " | DIRECTION CONFLICT" : "");
}

void setup() {
  Serial.begin(115200);

  pinMode(BRAKE_RELAY_PIN, OUTPUT);
  pinMode(DRIVE_RELAY_PIN, OUTPUT);
  pinMode(NEUTRAL_RELAY_PIN, OUTPUT);
  pinMode(REVERSE_RELAY_PIN, OUTPUT);
  pinMode(RELAY_JOY_RIGHT_PIN, OUTPUT);
  pinMode(RELAY_JOY_LEFT_PIN, OUTPUT);

  digitalWrite(BRAKE_RELAY_PIN, LOW);
  setAllGearRelays(LOW);
  digitalWrite(RELAY_JOY_RIGHT_PIN, LOW);
  digitalWrite(RELAY_JOY_LEFT_PIN, LOW);
  setThrottleOutput(0);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  // Pairing keys are retained. Erase them only during an intentional reset.
  BP32.enableVirtualDevice(false);

  Serial.println(
      "Ready: press brake and request Drive/Reverse, or click R3 for Neutral.");
}

void loop() {
  const uint32_t now = millis();
  throttleSourceChanged = false;
  bool dpadUpPressed = false;
  bool dpadDownPressed = false;

  if (BP32.update()) {
    updateControllerStates(now, dpadUpPressed, dpadDownPressed);
  }
  serviceControllerLedTests(now);

  if (expireStaleControllerData(now)) {
    resetThrottleCommand();
    controllerDataLost = true;
    Serial.println(
        "Controller data timeout: throttle stopped until a fresh report");
  }

  const CombinedInput input = combineControllerStates();
  if (!input.anyFresh) {
    resetThrottleCommand();
  }
  if (!anyControllerConnected()) {
    // A real disconnect requires a new brake-gated gear selection. A short
    // report timeout stops throttle but does not forget the physical gear.
    gearKnown = false;
  }

  updateDpadThrottle(dpadUpPressed, dpadDownPressed,
                     gearPulseActive || gearChangeAttemptedDuringBrake);
  // Once a valid gear pulse starts, keep the physical brake relay active until
  // that pulse finishes. The user still has to press brake to initiate it.
  const bool holdBrakeForGearPulse = gearPulseActive && !controllerDataLost;
  updateBrakeRelay(input.brake || holdBrakeForGearPulse, now);
  updateJoystickRelays(input);

  bool hasGearRequest = false;
  Gear requestedGear = currentGear;
  int signedThrottle =
      calculateSignedThrottle(input, hasGearRequest, requestedGear);

  if (controllerDataLost) {
    cancelGearPulse("controller disconnected or timed out");
    controllerDataLost = false;
  } else {
    serviceGearChange(now, hasGearRequest, requestedGear,
                      input.directionConflict);
  }

  // Brake and gear activity always expose a zero throttle command as well as
  // a zero DAC output. Gear selection has already latched its direction.
  if (brakeActive || gearPulseActive) {
    signedThrottle = 0;
  }

  int outputThrottle = 0;
  const char* throttleBlockReason = "NONE";
  if (!input.anyFresh) {
    throttleBlockReason = "NO_INPUT";
  } else if (gearPulseActive) {
    throttleBlockReason = "GEAR_CHANGE";
  } else if (brakeActive) {
    throttleBlockReason = "BRAKE";
  } else if (input.directionConflict) {
    throttleBlockReason = "CONFLICT";
  } else if (!gearKnown) {
    throttleBlockReason = "GEAR_UNKNOWN";
  } else if (throttleSourceChanged) {
    throttleBlockReason = "SOURCE_CHANGE";
  } else if (signedThrottle == 0) {
    throttleBlockReason = "ZERO_COMMAND";
  } else {
    if (signedThrottle > 0 && currentGear == DRIVE) {
      outputThrottle = signedThrottle;
    } else if (signedThrottle < 0 && currentGear == REVERSE) {
      outputThrottle = -signedThrottle;
    } else {
      throttleBlockReason = "GEAR_MISMATCH";
    }
  }
  setThrottleOutput(outputThrottle);

  if (now - lastPrintTime >= STATUS_INTERVAL_MS) {
    printStatus(input, signedThrottle, throttleBlockReason);
    lastPrintTime = now;
  }

  delay(5);
}
