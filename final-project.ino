// Author: justin0u0<mail@justin0u0.com>

// Class prototypes
class Wheel;
class Car;
class UltraSonic;
class Led;
class Infrared;

// Defines
// Distance states
#define NOT_FOUND 0
#define TOO_FAR 1
#define TOO_CLOSE 2
#define APPROPRIATE 3
// Car states
#define CAR_FORWARD 0
#define CAR_BACKWARD 1
#define CAR_STOP 2
#define CAR_LEFT 3
#define CAR_RIGHT 4

// Global variables
int8_t distanceState;
Car* car;
UltraSonic* ultraSonic;
Infrared* leftInfrared;
Infrared* rightInfrared;

unsigned long lastNotFound;

class Wheel {
  private:
    int8_t enablePin;
    int8_t in1Pin;
    int8_t in2Pin;
  public:
    Wheel(int8_t enable, int8_t in1, int8_t in2): enablePin(enable), in1Pin(in1), in2Pin(in2) {
      pinMode(enablePin, OUTPUT);
      pinMode(in1Pin, OUTPUT);
      pinMode(in2Pin, OUTPUT);

      // Default
      this->Stop();
      this->SetSpeed(255);
    }

    // Wheel::RotateFront()
    //  Make wheel spin clockwisely
    void RotateFront() {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
    }

    // Wheel::RotateBack()
    //  Make wheel spin anti-clockwisely
    void RotateBack() {
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
    }

    // Wheel::Stop()
    //  Stop the wheel spinning immediately
    void Stop() {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
    }

    // Wheel::SetSpeed(uint8_t)
    //  RotateSpeed should set in range [0, 255]
    void SetSpeed(uint8_t rotateSpeed) {
      analogWrite(enablePin, rotateSpeed);
    }
};

class Car {
  private:
    Wheel* leftWheel;
    Wheel* rightWheel;
    int8_t carState = CAR_STOP;
  public:
    Car(int8_t leftEnable, int8_t leftIn1, int8_t leftIn2, int8_t rightEnable, int8_t rightIn1, int8_t rightIn2) {
      leftWheel = new Wheel(leftEnable, leftIn1, leftIn2);
      rightWheel = new Wheel(rightEnable, rightIn1, rightIn2);

      this->Stop();
    }
    
    // Car::Forward()
    //  Move car forward
    void Forward() {
      if (carState != CAR_FORWARD) {
        carState = CAR_FORWARD;
        leftWheel->RotateFront();
        rightWheel->RotateFront();
      }
    }

    // Car::Backward()
    //  Move car backward
    void Backward() {
      if (carState != CAR_BACKWARD) {
        carState = CAR_BACKWARD;
        leftWheel->RotateBack();
        rightWheel->RotateBack();
      }
    }

    // Car::Left()
    //  Turn car left
    void Left() {
      if (carState != CAR_LEFT) {
        carState = CAR_LEFT;
        leftWheel->Stop();
        rightWheel->RotateFront();
      }
    }

    // Car::Right()
    //  Turn car right
    void Right() {
      if (carState != CAR_RIGHT) {
        carState = CAR_RIGHT;
        leftWheel->RotateFront();
        rightWheel->Stop();
      }
    }

    // Car::Stop()
    //  Stop car moving immediately
    void Stop() {
      if (carState != CAR_STOP) {
        carState = CAR_STOP;
        leftWheel->Stop();
        rightWheel->Stop();
      }
    }

    // Car::SetSpeed()
    //  Set car running speed
    //  rotateSpeed should in range [0, 255]
    void SetSpeed(uint8_t rotateSpeed) {
      leftWheel->SetSpeed(rotateSpeed);
      rightWheel->SetSpeed(rotateSpeed);
    }

    // Car::Debug()
    //  Output carState
    void Debug() {
      Serial.print("CarState: ");
      Serial.println(carState);
    }
};

class UltraSonic {
  private:
    int8_t trigPin;
    int8_t echoPin;

  public:
    UltraSonic(int8_t trig, int8_t echo): trigPin(trig), echoPin(echo) {
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
    }

    // UltraSonic::Measure()
    //  Measure the distance from ultra sonic
    //  Will block CPU for total 12 micro seconds
    float Measure() {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
  
      float distance = pulseIn(echoPin, HIGH) / 58.0;
      return distance;
    }

    // UltraSonic::GetDistanceState()
    //  Measure 10 times, get average distance, then update distanceState
    void UpdateDistanceState() {
      float average = 0.0;
      for (int8_t i = 0; i < 10; i++) {
        average += this->Measure();
      }
      average /= 10;

      if (average < 12) {
        distanceState = TOO_CLOSE;
      } else if (average < 18) { // average > 12
        distanceState = APPROPRIATE;
      } else if (average < 35) {
        distanceState = TOO_FAR;
      } else {
        distanceState = NOT_FOUND;
      }
    }
};

class Led {
  private:
    int8_t pin;
  public:
    Led(int8_t pin): pin(pin) {
      pinMode(pin, OUTPUT);
    }

    // Led::SetBrightness()
    //  Set brightness of LED in range [0, 255]
    void SetBrightness(uint8_t brightness) {
      analogWrite(pin, brightness);
    }
};

class Infrared {
  private:
    int8_t pin;
  public:
    Infrared(int8_t pin): pin(pin) {
      pinMode(pin, INPUT);
    }

    // Infrared::Detect
    bool Detect() {
      return (digitalRead(pin) == LOW) ? true : false;
    }
};

void setup() {
  Serial.begin(9600);
  car = new Car(5, 4, 3, 6, 7, 8);
  ultraSonic = new UltraSonic(13, 12);
  leftInfrared = new Infrared(2);
  rightInfrared = new Infrared(9);

  lastNotFound = -1;
}

void loop() {
  ultraSonic->UpdateDistanceState();

  if (distanceState == TOO_CLOSE) {
    Serial.println("BACK");
    car->Backward();
  } else if (distanceState == TOO_FAR) {
    Serial.println("FORWARD");
    car->Forward();
  } else if (distanceState == APPROPRIATE) {
    Serial.println("STOP");
    car->Stop();
  } else {
    if (leftInfrared->Detect()) {
      Serial.println("LEFT");
      car->Left();
    } else if (rightInfrared->Detect()) {
      Serial.println("RIGHT");
      car->Right();
    } else {
      Serial.println("STOP");
      car->Stop();
    }
  }
}
