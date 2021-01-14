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
      this->SetSpeed(100);
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
  public:
    Car(int8_t leftEnable, int8_t leftIn1, int8_t leftIn2, int8_t rightEnable, int8_t rightIn1, int8_t rightIn2) {
      leftWheel = new Wheel(leftEnable, leftIn1, leftIn2);
      rightWheel = new Wheel(rightEnable, rightIn1, rightIn2);
    }
    
    // Car::Forward()
    //  Move car forward
    void Forward() {
      leftWheel->RotateFront();
      rightWheel->RotateFront();
    }

    // Car::Backward()
    //  Move car backward
    void Backward() {
      leftWheel->RotateBack();
      rightWheel->RotateBack();
    }

    // Car::Stop()
    //  Stop car moving immediately
    void Stop() {
      leftWheel->Stop();
      rightWheel->Stop();
    }

    // Car::SetSpeed()
    //  Set car running speed
    //  rotateSpeed should in range [0, 255]
    void SetSpeed(uint8_t rotateSpeed) {
      leftWheel->SetSpeed(rotateSpeed);
      rightWheel->SetSpeed(rotateSpeed);
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
    //  Will block CPU for total 12 micro seconds.
    double Measure() {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
  
      return pulseIn(echoPin, HIGH) / 58.0;
    }
};

// Global variables
Car car(5, 4, 3, 6, 7, 8);
UltraSonic ultraSonic(13, 12);
  
void setup() {
  Serial.begin(9600);

  car.SetSpeed(255);
  car.Forward();
}

void loop() {
  Serial.println(ultraSonic.Measure());

  car.Forward();
  delay(3000);

  car.Stop();
  delay(3000);
}
