#include <MQTT.h>
#include <WiFiClientSecure.h>
#include <vector>
#include "creds.h"

const String WIFI_SSID = WIFI_SSID_STR;
const String WIFI_PASSWORD = WIFI_PASSWORD_STR;
const String MQTT_SERVER = MQTT_SERVER_STR;
const String MQTT_USERNAME = MQTT_USERNAME_STR;
const String MQTT_PASSWORD = MQTT_PASSWORD_STR;

class Button {
private:
  int pin;
  bool isPressed;
  bool continuous;
  std::function<void()> onPressed;
public:
  Button(int pin, std::function<void()> onPressed, bool continuous = true) {
    this->pin = pin;
    this->isPressed = false;
    this->onPressed = onPressed;
    this->continuous = true;
  }
  void setup() {
    pinMode(pin, INPUT_PULLUP);
  }
  void checkButton() {
    if (!digitalRead(pin)) {
      if (!isPressed || continuous) {
        onPressed();
      }
      isPressed = true;
    } else {
      isPressed = false;
    }
  }
};

class Servo {
private:
  int pin;
  int pwmMin;
  int pwmMax;
  int angleMin;
  int angleMax;
  int allowedAngleMin;
  int allowedAngleMax;
public:
  Servo(int pin, int allowedAngleMin = 0, int allowedAngleMax = 180, int pwmMin = 500, int pwmMax = 2500, int angleMin = 0, int angleMax = 180) {
    if (angleMin < 0) {
      angleMin = 0;
    }
    if (angleMax > 360) {
      angleMax = 360;
    }
    if (allowedAngleMin < angleMin) {
      allowedAngleMin = angleMin;
    }
    if (allowedAngleMax > angleMax) {
      allowedAngleMax = angleMax;
    }
    this->pin = pin;
    this->pwmMin = pwmMin;
    this->pwmMax = pwmMax;
    this->angleMin = angleMin;
    this->angleMax = angleMax;
    this->allowedAngleMin = allowedAngleMin;
    this->allowedAngleMax = allowedAngleMax;
  }
  int getAngleMin() const {
    return allowedAngleMin;
  }
  int getAngleMax() const {
    return allowedAngleMax;
  }
  void setup() {
    pinMode(this->pin, OUTPUT);
  }
  int moveToAngle(int angle) {
    if (angle < this->allowedAngleMin) {
      angle = this->allowedAngleMin;
    }
    if (angle > this->allowedAngleMax) {
      angle = this->allowedAngleMax;
    }
    int pulseWidth = map(angle, this->angleMin, this->angleMax, this->pwmMin, this->pwmMax);
    digitalWrite(this->pin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(this->pin, LOW);
    delayMicroseconds(pulseWidth);
    return angle;
  }
};

int getMiddleInt(int smaller, int bigger) {
  return smaller + ((bigger - smaller) / 2);
};

class RobotArm {
private:
public:
  String deviceId;
  String deviceName;
  String MQTTTriggerTopic;
  Servo* lowerArm;
  Servo* upperArm;
  int lowerArmAngle = -1;
  int upperArmAngle = -1;
  RobotArm(int lowerArmPin, int upperArmPin, String deviceNameWord = "megas", int allowedLowerArmAngleMin = 20, int allowedLowerArmAngleMax = 140, int allowedUpperArmAngleMin = 0, int allowedUpperArmAngleMax = 180) {
    deviceId = deviceNameWord;
    deviceId.toLowerCase();
    deviceName = deviceNameWord;
    deviceName.toLowerCase();
    deviceName.setCharAt(0, toupper(deviceName.charAt(0)));
    deviceName = "Robot Arm " + deviceName;
    MQTTTriggerTopic = "robotarm/" + deviceId + "/action";
    lowerArm = new Servo(lowerArmPin, allowedLowerArmAngleMin, allowedLowerArmAngleMax);
    upperArm = new Servo(upperArmPin, allowedUpperArmAngleMin, allowedUpperArmAngleMax);
  }
  void setup() {
    lowerArm->setup();
    upperArm->setup();
  }
  void move(int lowerArmAngle, int upperArmAngle) {
    // Serial.println("(" + String(lowerArmAngle) + ", " + String(upperArmAngle) + ")");
    this->lowerArmAngle = lowerArm->moveToAngle(lowerArmAngle);
    this->upperArmAngle = upperArm->moveToAngle(upperArmAngle);
  }
  void hold(int lowerArmAngle, int upperArmAngle, int milliseconds) {
    Serial.println("Holding for ~" + String(milliseconds) + "ms at angles (" + String(lowerArmAngle) + ", " + String(upperArmAngle) + ")...");
    unsigned long now = millis();
    unsigned long breakAt = now + milliseconds;
    while (true) {
      move(lowerArmAngle, upperArmAngle);
      if (millis() >= breakAt) {
        break;
      }
    }
    delay(30);
  }
  void neutralHold(int milliseconds) {
    hold(
      getMiddleInt(lowerArm->getAngleMin(), lowerArm->getAngleMax()) - 10,
      getMiddleInt(upperArm->getAngleMin(), upperArm->getAngleMax()) + 90,
      milliseconds);
  }
  void smoothMove(int lowerArmAngleStart, int lowerArmAngleEnd, int upperArmAngleStart, int upperArmAngleEnd, int angleRepetitions = 3, int angleSkips = 0) {
    // if (enable()) {
    //   int lowerArmAngleDiff = lowerArmAngleEnd - lowerArmAngleStart;
    //   int upperArmAngleDiff = upperArmAngleEnd - upperArmAngleStart;
    //   lowerArmAngleStart = lowerArmAngle;
    //   lowerArmAngleEnd = lowerArmAngleStart + lowerArmAngleDiff;
    //   upperArmAngleStart = upperArmAngle;
    //   upperArmAngleEnd = upperArmAngleStart + upperArmAngleDiff;
    // }
    if (angleRepetitions < 1) {
      angleRepetitions = 1;
    }
    if (angleSkips < 0) {
      angleSkips = 0;
    }
    Serial.println("Moving from angles (" + String(lowerArmAngleStart) + ", " + String(upperArmAngleStart) + ") to ("
                   + String(lowerArmAngleEnd) + ", " + String(upperArmAngleEnd) + ") with " + angleRepetitions + " angle repetitions and " + angleSkips + " skips...");
    int lA = lowerArmAngleStart;
    int uA = upperArmAngleStart;
    while (true) {
      for (int i = 0; i < angleRepetitions; i++) {
        move(lA, uA);
      }
      if (lA == lowerArmAngleEnd && uA == upperArmAngleEnd) {
        break;
      }
      bool lForward = lowerArmAngleStart < lowerArmAngleEnd;
      lA = lA + ((1 + angleSkips) * (lForward ? 1 : -1));
      lA = lForward ? (lA > lowerArmAngleEnd ? lowerArmAngleEnd : lA) : (lA < lowerArmAngleEnd ? lowerArmAngleEnd : lA);
      bool uForward = upperArmAngleStart < upperArmAngleEnd;
      uA = uA + ((1 + angleSkips) * (uForward ? 1 : -1));
      uA = uForward ? (uA > upperArmAngleEnd ? upperArmAngleEnd : uA) : (uA < upperArmAngleEnd ? upperArmAngleEnd : uA);
    }
    delay(30);
  }
  void fullSweep() {
    smoothMove(lowerArm->getAngleMin(), lowerArm->getAngleMax(), upperArm->getAngleMin(), upperArm->getAngleMax());
    hold(lowerArm->getAngleMax(), upperArm->getAngleMax(), 1000);
    smoothMove(lowerArm->getAngleMax(), lowerArm->getAngleMin(), upperArm->getAngleMax(), upperArm->getAngleMin());
    hold(lowerArm->getAngleMin(), upperArm->getAngleMin(), 1000);
  }
};

// class RobotArmHomeAssistantMQTTTrigger {
// private:
// public:
//   String action;
//   String discoveryTopic;
//   String discoveryPayload;
//   std::function<void()> actionFn;
//   RobotArmHomeAssistantMQTTTrigger(RobotArm* arm, String action, std::function<void()> actionFn) {
//     this->action = action;
//     this->actionFn = actionFn;
//     discoveryTopic = "homeassistant/device_automation/" + arm->deviceId + "/action_" + action + "/config";
//     discoveryPayload = "{"
//                        "\"automation_type\": \"trigger\","
//                        "\"type\": \"action\","
//                        "\"subtype\": \""
//                        + action + "\","
//                                   "\"payload\": \""
//                        + action + "\","
//                                   "\"topic\": \""
//                        + discoveryTopic + "\","
//                                           "\"device\": {"
//                                           "\"identifiers\": ["
//                                           "\""
//                        + arm->deviceId + "\""
//                                          "],"
//                                          "\"name\": \""
//                        + arm->deviceName + "\""
//                                            "}"
//                                            "}";
//   }
// };

RobotArm arm(D12, D11);
Button lowerArmForwardButton(D7, []() {
  arm.move(arm.lowerArmAngle - 1, arm.upperArmAngle);
});
Button lowerArmReverseButton(D5, []() {
  arm.move(arm.lowerArmAngle + 1, arm.upperArmAngle);
});
Button upperArmForwardButton(D6, []() {
  arm.move(arm.lowerArmAngle, arm.upperArmAngle + 1);
});
Button upperArmReverseButton(D4, []() {
  arm.move(arm.lowerArmAngle, arm.upperArmAngle - 1);
});
std::vector<Button> armButtons = { lowerArmForwardButton, lowerArmReverseButton, upperArmForwardButton, upperArmReverseButton };

// RobotArmHomeAssistantMQTTTrigger lowerArmForwardHATrigger(&arm, "lower_arm_forward", []() {
//   arm.smoothMove(arm.lowerArmAngle, arm.lowerArmAngle - 10, arm.upperArmAngle, arm.upperArmAngle);
// });
// RobotArmHomeAssistantMQTTTrigger lowerArmReverseHATrigger(&arm, "lower_arm_reverse", []() {
//   arm.smoothMove(arm.lowerArmAngle, arm.lowerArmAngle + 10, arm.upperArmAngle, arm.upperArmAngle);
// });
// RobotArmHomeAssistantMQTTTrigger upperArmForwardHATrigger(&arm, "upper_arm_forward", []() {
//   arm.smoothMove(arm.lowerArmAngle, arm.lowerArmAngle, arm.upperArmAngle, arm.upperArmAngle + 10);
// });
// RobotArmHomeAssistantMQTTTrigger upperArmReverseHATrigger(&arm, "upper_arm_reverse", []() {
//   arm.smoothMove(arm.lowerArmAngle, arm.lowerArmAngle, arm.upperArmAngle, arm.upperArmAngle - 10);
// });
// std::vector<RobotArmHomeAssistantMQTTTrigger> armHATriggers = { lowerArmForwardHATrigger, lowerArmReverseHATrigger, upperArmForwardHATrigger, upperArmReverseHATrigger };

WiFiClientSecure NET;
MQTTClient MQTT(1024);

String moveMQTTTopic = "robotarm/" + arm.deviceId + "/move";
String moveLowerMQTTTopic = "robotarm/" + arm.deviceId + "/move/lower";
String moveUpperMQTTTopic = "robotarm/" + arm.deviceId + "/move/upper";

bool MQTTConnect() {
  if (WiFi.status() == WL_CONNECTED) {
    // Prevent TLS certificate verification
    // TODO: Figure this out. See:
    // https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFiClientSecure/examples/WiFiClientSecure/WiFiClientSecure.ino
    NET.setInsecure();
    if (MQTT.connect(arm.deviceName.c_str(), MQTT_USERNAME.c_str(),
                     MQTT_PASSWORD.c_str(), false)) {
      // MQTT.subscribe(arm.MQTTTriggerTopic);
      // for (const RobotArmHATrigger& action : armHATriggers) {
      //   MQTT.publish(action.discoveryTopic, action.discoveryPayload);
      // }
      MQTT.subscribe(moveMQTTTopic);
      MQTT.subscribe(moveLowerMQTTTopic);
      MQTT.subscribe(moveUpperMQTTTopic);
      Serial.println("Connected.");
      return true;
    }
    return false;
  }
  return false;
}

int addOrReplaceStrNumtoInt(int num, String str) {
  char sign = str.charAt(0);
  if (sign == '+') {
    return num + str.substring(1).toInt();
  }
  if (sign == '-') {
    return num - str.substring(1).toInt();
  }
  return str.toInt();
}

void MQTTMessageReceived(String& topic, String& payload) {
  // if (topic == arm.MQTTTriggerTopic) {
  //   for (const RobotArmHATrigger& action : armHATriggers) {
  //     if (payload.equals(action.action)) {
  //       action.actionFn();
  //     }
  //   }
  // }

  if (topic == moveMQTTTopic) {
    int delimiterPosition = payload.indexOf(',');
    String lower = payload.substring(0, delimiterPosition);
    String upper = payload.substring(delimiterPosition + 1);
    arm.smoothMove(arm.lowerArmAngle, addOrReplaceStrNumtoInt(arm.lowerArmAngle, lower), arm.upperArmAngle, addOrReplaceStrNumtoInt(arm.upperArmAngle, upper));
  }
  if (topic == moveLowerMQTTTopic) {
    arm.smoothMove(arm.lowerArmAngle, addOrReplaceStrNumtoInt(arm.lowerArmAngle, payload), arm.upperArmAngle, arm.upperArmAngle);
  }
  if (topic == moveUpperMQTTTopic) {
    arm.smoothMove(arm.lowerArmAngle, arm.lowerArmAngle, arm.upperArmAngle, addOrReplaceStrNumtoInt(arm.upperArmAngle, payload));
  }
}

void setup() {
  Serial.begin(115200);
  delay(10);
  for (Button& button : armButtons) {
    button.setup();
  }
  arm.setup();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  MQTT.begin(MQTT_SERVER.c_str(), 8883, NET);
  MQTT.onMessage(MQTTMessageReceived);
  MQTTConnect();
  arm.neutralHold(1000);
}

long int disconnectedAt = -1;
void loop() {
  for (Button& button : armButtons) {
    button.checkButton();
  }
  MQTT.loop();
  if (!MQTT.connected()) {
    if (disconnectedAt < 0) {
      disconnectedAt = millis();
    }
    if (disconnectedAt + 30000 <= millis()) {
      MQTTConnect();  // After 30s of no connection, give up on trying until next reset
    }
  } else {
    disconnectedAt = -1;
  }
}
