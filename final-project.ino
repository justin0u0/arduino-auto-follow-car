// Author: justin0u0<mail@justin0u0.com>

#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <stdarg.h>

// Class prototypes
class Wheel;
class Car;
class UltraSonic;
class Led;
class Infrared;

// Defines
// Sensor states
#define NOT_FOUND 0
#define TOO_FAR 1
#define TOO_CLOSE 2
#define APPROPRIATE 3
#define TOO_LEFT 4
#define TOO_RIGHT 5
// Car states
#define CAR_FORWARD 0
#define CAR_BACKWARD 1
#define CAR_STOP 2
#define CAR_LEFT 3
#define CAR_RIGHT 4

// Global shared variables
int8_t sensorState;
int8_t timerCounter;
const int8_t wakeUpPin = 2;
TaskHandle_t sensorControlTaskHandle = NULL;
TaskHandle_t carControlTaskHandle = NULL;
TaskHandle_t lightControlTaskHandle = NULL;
bool suspendCarFlag = false;
bool suspendLightFlag = false;

// Locking
SemaphoreHandle_t sensorStateUpdated;

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

    // UltraSonic::UpdateSensorState()
    //  Measure 10 times, get average distance, then return with new sensorState
    int8_t UpdateSensorState() {
      float average = 0.0;
      for (int8_t i = 0; i < 10; i++) {
        average += this->Measure();
      }
      average /= 10;

      int8_t state;
      if (average < 12) {
        state = TOO_CLOSE;
      } else if (average < 18) { // average > 12
        state = APPROPRIATE;
      } else if (average < 35) {
        state = TOO_FAR;
      } else {
        state = NOT_FOUND;
      }
      return state;
    }
};

class Led {
  private:
    int8_t pin;
  public:
    Led(int8_t pin): pin(pin) {
      pinMode(pin, OUTPUT);

      // Default
      this->SetBrightness(0);
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

// sensorControlTask
//  Ultra sonic, infrared control task
//  signal carControlTask if sensorState changed
void sensorControlTask(void* pvParameters) {
  // Setup
  UltraSonic* ultraSonic = new UltraSonic(13, 12);
  Infrared* leftInfrared = new Infrared(A2);
  Infrared* rightInfrared = new Infrared(A1);
  int8_t newState = -1;

  // Loop
  for(;;) {
    if (suspendCarFlag) {
      xSemaphoreGive(sensorStateUpdated);
    }

    // Detech by ultra sonic
    newState = ultraSonic->UpdateSensorState();

    // Not found by ultra sonic, then detect by infrareds
    if (newState == NOT_FOUND) {
      if (leftInfrared->Detect()) {
        newState = TOO_LEFT;
      } else if (rightInfrared->Detect()) {
        newState = TOO_RIGHT;
      }
    }

    // Signal carControlTask
    if (sensorState != newState) {
      sensorState = newState;
      xSemaphoreGive(sensorStateUpdated);
    }
  }
}

void lightControlTask(void* pvParameters) {
  // Setup
  Led* led = new Led(11);
  int8_t value = 0;

  // Loop
  for (;;) {
    if (suspendLightFlag) {
      led->SetBrightness(0);
      suspendLightFlag = false;
      vTaskSuspend(NULL);
    }

    value = analogRead(A0);
    if (value >= 100) {
      led->SetBrightness(0);
    } else if (value >= 60) {
      led->SetBrightness(75);
    } else if (value >= 30) {
      led->SetBrightness(150);
    } else {
      led->SetBrightness(255);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void carControlTask(void* pvParameters) {
  // Setup
  Car* car = new Car(5, 4, 3, 6, 7, 8);

  // Loop
  for (;;) {
    // Wait until sensorState got updated
    xSemaphoreTake(sensorStateUpdated, portMAX_DELAY);

    if (suspendCarFlag) {
      car->Stop();
      suspendCarFlag = false;
      vTaskSuspend(sensorControlTaskHandle);
      vTaskSuspend(NULL);
    }

    switch (sensorState) {
      case TOO_CLOSE:
        car->Backward();
        break;
      case TOO_FAR:
        car->Forward();
        break;
      case APPROPRIATE:
        car->Stop();
        break;
      case TOO_LEFT:
        car->Left();
        break;
      case TOO_RIGHT:
        car->Right();
        break;
      case NOT_FOUND:
        // Start count 10 seconds, then sleep
        car->Stop();
        TCNT1 = 0; // Reset timer
        timerCounter = 0;
        break;
      default:
        car->Stop();
    }
  }
}

ISR(TIMER1_COMPA_vect) {
  timerCounter++;
  if (timerCounter == 10) {
    // Sleep
    // Serial.println(F("SLEEP"));
    suspendLightFlag = true;
    suspendCarFlag = true;
  }
}

void wakeUp() {
  // Serial.println(F("WakeUP"));
  xTaskResumeFromISR(sensorControlTaskHandle);
  xTaskResumeFromISR(carControlTaskHandle);
  xTaskResumeFromISR(lightControlTaskHandle);
}

void setup() {
  Serial.begin(9600);

  // Initialize Timer1, timer1 infect pin 9 and pin 10
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0; // Actual timer value
  OCR1A = 62500; // 16MHZ / 256 * 1
  TCCR1B |= (1 << WGM12); // Clean Timer on Compare Mode
  TCCR1B |= (1 << CS12); // Prescaler 256
  TIMSK1 |= (1 << OCIE1A); // Enable timer compare interrupt
  interrupts();

  // Setup external wakeup pin
  pinMode(wakeUpPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, CHANGE);

  sensorStateUpdated = xSemaphoreCreateBinary();

  // TODO: make turn until object found
  // initialSearch();

  xTaskCreate(sensorControlTask, "SensorControl", 64, NULL, 1, &sensorControlTaskHandle);
  xTaskCreate(carControlTask, "CarControl", 128, NULL, 1, &carControlTaskHandle);
  xTaskCreate(lightControlTask, "LightControl", 64, NULL, 1, &lightControlTaskHandle);
}

void loop() {}
