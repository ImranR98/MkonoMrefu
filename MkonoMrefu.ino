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
    int getAngleMin() const { return allowedAngleMin; }
    int getAngleMax() const { return allowedAngleMax; }
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

class Arm {
  private:

  public:
    Servo* lowerArm;
    Servo* upperArm;
    int lowerArmAngle = -1;
    int upperArmAngle = -1;
    Arm(int lowerArmPin, int upperArmPin, int allowedLowerArmAngleMin = 20, int allowedLowerArmAngleMax = 140, int allowedUpperArmAngleMin = 0, int allowedUpperArmAngleMax = 180) {
      lowerArm = new Servo(lowerArmPin, allowedLowerArmAngleMin, allowedLowerArmAngleMax);
      upperArm = new Servo(upperArmPin, allowedUpperArmAngleMin, allowedUpperArmAngleMax);
    }
    void setup() {
      lowerArm->setup();
      upperArm->setup();
    }
    void move(int lowerArmAngle, int upperArmAngle) {
      Serial.println("(" + String(lowerArmAngle) + ", " + String(upperArmAngle) + ")");
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
        getMiddleInt(lowerArm->getAngleMin(), lowerArm->getAngleMax()), 
        getMiddleInt(upperArm->getAngleMin(), upperArm->getAngleMax()),
        milliseconds
      );
    }
    void smoothMove(int lowerArmAngleStart, int lowerArmAngleEnd, int upperArmAngleStart, int upperArmAngleEnd, int angleRepetitions = 3, int angleSkips = 0) {
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

Arm arm(D12, D11, D10);
Button lowerArmUpButton(D7, []() {
  arm.move(arm.lowerArmAngle + 1, arm.upperArmAngle);
});
Button lowerArmDownButton(D5, []() {
  arm.move(arm.lowerArmAngle - 1, arm.upperArmAngle);
});
Button upperArmUpButton(D6, []() {
  arm.move(arm.lowerArmAngle, arm.upperArmAngle + 1);
});
Button upperArmDownButton(D4, []() {
  arm.move(arm.lowerArmAngle, arm.upperArmAngle - 1);
});

void setup() {
  Serial.begin(115200);
  delay(10);
  lowerArmDownButton.setup();
  lowerArmUpButton.setup();
  upperArmDownButton.setup();
  upperArmUpButton.setup();
  arm.setup();
  arm.neutralHold(1000);
}

void loop() {
  lowerArmDownButton.checkButton();
  lowerArmUpButton.checkButton();
  upperArmDownButton.checkButton();
  upperArmUpButton.checkButton();
  arm.move(arm.lowerArmAngle, arm.upperArmAngle);
}
