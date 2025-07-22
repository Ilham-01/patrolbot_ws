// -----------------------------
// Pin Definitions
// -----------------------------
#define alarm 7
#define red_light 6
#define green_light 5
#define motorA1 2
#define motorA2 3
#define motorB1 8
#define motorB2 9
#define speedMotor 10

#define leftEncoderPin 18
#define rightEncoderPin 19

// -----------------------------
// State Variables
// -----------------------------
volatile long leftTicks = 0;
volatile long rightTicks = 0;

float leftSpeed = 0.0;
float rightSpeed = 0.0;
unsigned long lastCommandTime = 0;
const unsigned long timeoutMs = 1000;

String inputString = "";
bool stringComplete = false;

// -----------------------------
// Setup
// -----------------------------
void setup() {
  Serial.begin(115200);
  inputString.reserve(50);

  pinMode(alarm, OUTPUT);
  pinMode(red_light, OUTPUT);
  pinMode(green_light, OUTPUT);

  pinMode(motorA1, OUTPUT); pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT); pinMode(motorB2, OUTPUT);
  pinMode(speedMotor, OUTPUT);
  analogWrite(speedMotor, 200);  // fixed PWM speed

  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), countRight, RISING);
}

// -----------------------------
// Main Loop
// -----------------------------
void loop() {
  unsigned long now = millis();

  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
    lastCommandTime = now;
  }

  // Stop if no command received recently
  if (now - lastCommandTime > timeoutMs) {
    stopMotors();
  } else {
    applyMotorSpeeds();
  }

  sendEncoderData();
  delay(50);  // Adjust based on how fast you want updates
}

// -----------------------------
// Serial Event
// -----------------------------
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

// -----------------------------
// Parse ROS2 Velocity Command
// -----------------------------
void parseCommand(String cmd) {
  cmd.trim();
  int spaceIndex = cmd.indexOf(' ');
  if (spaceIndex > 0 && spaceIndex < cmd.length() - 1) {
    float l = cmd.substring(0, spaceIndex).toFloat();
    float r = cmd.substring(spaceIndex + 1).toFloat();
    leftSpeed = l;
    rightSpeed = r;
  }
}

// -----------------------------
// Apply Speeds to Motors
// -----------------------------
void applyMotorSpeeds() {
  setMotor(leftSpeed, 'L');
  setMotor(rightSpeed, 'R');
}

void setMotor(float speed, char side) {
  int in1, in2;
  if (side == 'L') {
    in1 = motorA1; in2 = motorA2;
  } else {
    in1 = motorB1; in2 = motorB2;
  }

  if (speed > 0.01) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < -0.01) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// -----------------------------
// Stop Motors
// -----------------------------
void stopMotors() {
  leftSpeed = 0.0;
  rightSpeed = 0.0;
  digitalWrite(motorA1, LOW); digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW); digitalWrite(motorB2, LOW);
}

// -----------------------------
// Encoder Output
// -----------------------------
void sendEncoderData() {
  Serial.print(leftTicks);
  Serial.print(" ");
  Serial.println(rightTicks);
}

// -----------------------------
// Encoder ISRs
// -----------------------------
void countLeft()  { leftTicks++; }
void countRight() { rightTicks++; }
