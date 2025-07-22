// === Pin Definitions ===
// Motors & Lights
#define alarm 7
#define red_light 6
#define green_light 5
#define motorA1 2
#define motorA2 3
#define motorB1 8
#define motorB2 9
#define speedMotor 10

// Ultrasonic Pins
const int trigPinM = A1;
const int echoPinM = A0;
const int trigPinR = A3;
const int echoPinR = A2;
const int trigPinL = A4;
const int echoPinL = A5;

// Encoder Pins
#define leftEncoderPin 18
#define rightEncoderPin 19
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// === Robot Settings ===
const int threshold_dist = 30;  // Stop if closer than this
int motorPower = 200;           // PWM speed (0-255)

float linear_vel = 0.0;
float angular_vel = 0.0;
bool obstacle_detected = false;

// Serial input
String inputString = "";
bool stringComplete = false;

// === Interrupts ===
void leftEncoderISR()  { leftTicks++; }
void rightEncoderISR() { rightTicks++; }

// === Ultrasonic Distance Function ===
int readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  int distance = duration * 0.034 / 2;
  return distance;
}

// === Motor Control ===
void setMotorSpeed(int pwmA, int pwmB) {
  analogWrite(speedMotor, abs(pwmA));
  digitalWrite(motorA1, pwmA > 0);
  digitalWrite(motorA2, pwmA < 0);
  digitalWrite(motorB1, pwmB > 0);
  digitalWrite(motorB2, pwmB < 0);
}

// === Stop Motors ===
void stopMotors() {
  analogWrite(speedMotor, 0);
  digitalWrite(motorA1, LOW); digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW); digitalWrite(motorB2, LOW);
}

// === Serial Command Handling ===
void processInput() {
  if (inputString.startsWith("v ")) {
    sscanf(inputString.c_str(), "v %f %f", &linear_vel, &angular_vel);
  }
  else if (inputString.startsWith("e")) {
    Serial.print("e ");
    Serial.print(leftTicks);
    Serial.print(" ");
    Serial.println(rightTicks);
  }
  inputString = "";
  stringComplete = false;
}

// === Setup ===
void setup() {
  Serial.begin(115200);

  pinMode(alarm, OUTPUT);
  pinMode(red_light, OUTPUT);
  pinMode(green_light, OUTPUT);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(speedMotor, OUTPUT);

  pinMode(trigPinM, OUTPUT); pinMode(echoPinM, INPUT);
  pinMode(trigPinL, OUTPUT); pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT); pinMode(echoPinR, INPUT);

  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, RISING);

  digitalWrite(green_light, HIGH);
}

// === Main Loop ===
void loop() {
  // Read ultrasonic sensors
  int distM = readUltrasonic(trigPinM, echoPinM);
  int distL = readUltrasonic(trigPinL, echoPinL);
  int distR = readUltrasonic(trigPinR, echoPinR);
  obstacle_detected = (distM < threshold_dist || distL < threshold_dist || distR < threshold_dist);

  if (obstacle_detected) {
    stopMotors();
    digitalWrite(red_light, HIGH);
    digitalWrite(alarm, HIGH);
  } else {
    // Convert velocity to motor commands (simplified differential logic)
    int pwmA = motorPower * (linear_vel - angular_vel);
    int pwmB = motorPower * (linear_vel + angular_vel);
    setMotorSpeed(pwmA, pwmB);
    digitalWrite(red_light, LOW);
    digitalWrite(alarm, LOW);
  }

  // Handle serial input
  if (stringComplete) {
    processInput();
  }

  // Read serial character-by-character
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
