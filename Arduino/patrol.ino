// Existing motor and sensor pins
#define alarm 7
#define red_light 6
#define green_light 5

#define motorA1 2
#define motorA2 3
#define motorB1 8
#define motorB2 9

#define speedMotor 10

const int trigPinM = A1;
const int echoPinM = A0;
const int trigPinR = A3;
const int echoPinR = A2;
const int trigPinL = A4;
const int echoPinL = A5;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

#define leftEncoderPin 18  // Change according to your board (e.g., 2 or 3 on UNO)
#define rightEncoderPin 19

long duration;
int distance;
int distM, distR, distL;

int threshold_dist = 50;

void setup() {
  Serial.begin(9600);

  pinMode(alarm, OUTPUT);
  pinMode(red_light, OUTPUT);
  pinMode(green_light, OUTPUT);

  pinMode(trigPinM, OUTPUT); pinMode(echoPinM, INPUT);
  pinMode(trigPinR, OUTPUT); pinMode(echoPinR, INPUT);
  pinMode(trigPinL, OUTPUT); pinMode(echoPinL, INPUT);

  pinMode(motorA1, OUTPUT); pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT); pinMode(motorB2, OUTPUT);
  pinMode(speedMotor, OUTPUT);
  analogWrite(speedMotor, 125);

  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), countRight, RISING);

  delay(5000);
}

void loop() {
  distM = ultrasonic(trigPinM, echoPinM);
  distR = ultrasonic(trigPinR, echoPinR);
  distL = ultrasonic(trigPinL, echoPinL);

  if (distM < threshold_dist && distR < threshold_dist - 20) {
    Rmotor_forward();
    Lmotor_reverse();
  } else if (distM < threshold_dist && distL < threshold_dist - 20) {
    Lmotor_forward();
    Rmotor_reverse();
  } else if (distM < threshold_dist) {
    Rmotor_stop(); Lmotor_stop(); Rlight_on(); Glight_off(); //alarm_on();
  } else {
    Rmotor_forward(); Lmotor_forward(); Glight_on(); Rlight_off(); alarm_off();
  }

  // Send encoder ticks to Raspberry Pi
  Serial.print("ENC_L:"); Serial.print(leftTicks);
  Serial.print(" ENC_R:"); Serial.println(rightTicks);

  delay(100); // Limit the update rate
}

void countLeft() { leftTicks++; }
void countRight() { rightTicks++; }

int ultrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}

// Existing motor and light control functions below here…
