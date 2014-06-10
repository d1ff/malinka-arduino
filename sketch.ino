#include <AFMotor.h>
#include <Servo.h> 
 
#define FRONT_SERVO 9
#define US_SENSOR 5
#define DC_MOTOR 1

#define MAX_SPEED 255
#define MIN_SPEED 0

const int SCANNING = 0;
const int MOVING   = 1;

const int MOVE_FORWARD  = 1;
const int MOVE_BACKWARD = 2;
const int TURN_LEFT     = 3;
const int TURN_RIGHT    = 4;
const int STOP          = 5;
const int STRAIGHT      = 6;
const int START_SCANNING = 1;

typedef struct {
    byte mode : 2;
    byte action : 6;
} Command;

typedef void (*actionDef)(byte param);

//void (*action[2][5])(byte param);
actionDef action[2][10];

Servo servo;
AF_DCMotor motor(DC_MOTOR);
volatile long duration, distance;

volatile bool commandComplete = false;
byte frontAngle = 75;

volatile byte mode = SCANNING;
bool left = false;
char commandBuffer[3];
Command command;


void serialEvent() {
    byte in;
    while (Serial.available()) {
        if (Serial.readBytes(commandBuffer, 3) != 3)
            continue;
        memcpy(&command, &commandBuffer[0], sizeof(Command));
        actionDef a = action[command.mode][command.action];
        if (a != 0) {
            Serial.println("ok");
            Serial.flush();
            mode = command.mode;
            (*a)(commandBuffer[1]);
        } else {
            Serial.print("no such command: ");
            Serial.print(command.mode);
            Serial.print(" ");
            Serial.print(command.action);
            Serial.println();
            Serial.flush();
        }
    }
}

void moveForward(byte param) { 
    Serial.print("forward\n");
    motor.setSpeed(param);
    motor.run(FORWARD);
    delay(100);
}
void moveBackward(byte param) {
    Serial.print("backward\n");
    motor.setSpeed(param);
    motor.run(BACKWARD);
    delay(100);
}
void turnLeft(byte param) {
    Serial.print("turn left\n");
    frontAngle = 60;
    turnSensor(frontAngle);
}
void turnRight(byte param) { 
    Serial.print("turn right\n");
    frontAngle = 120;
    turnSensor(frontAngle);
}
void turnStraight(byte param) { 
    Serial.print("turn straight\n");
    frontAngle = 90;
    turnSensor(frontAngle);
}
void actionStop(byte param) {
    Serial.print("stop\n");
    motor.run(RELEASE);
    delay(100);
}
void startScan(byte param) {
    Serial.print("scanning\n");
    motor.run(RELEASE);
    delay(100);
}

void setup() 
{
    servo.attach(9);

    motor.setSpeed(200);
    motor.run(RELEASE);
    
    memset(action, 0, sizeof(action));
    
    action[MOVING][MOVE_FORWARD] = moveForward;
    action[MOVING][MOVE_BACKWARD] = moveBackward;
    action[MOVING][TURN_LEFT] = turnLeft;
    action[MOVING][TURN_RIGHT] = turnRight;
    action[MOVING][STOP] = actionStop;
    action[MOVING][STRAIGHT] = turnStraight;
    action[SCANNING][START_SCANNING] = startScan;
    Serial.begin(115200); 

}

void turnSensor(byte &angle) {
    servo.write(angle);
    delay(15);
    angle = servo.read();
}

void timedAction() {
   if (mode == SCANNING) {
        frontAngle = frontAngle + (left ? 5 : -5);
        turnSensor(frontAngle);
        if (frontAngle == 135 || frontAngle == 45)
            left = !left;
    }
    distance = distanceInCm();
    printData();
    if (distance < 10) 
        fullStop();
}

void printData() {
    Serial.print(duration);
    Serial.print(",");
    Serial.print(distance);
    Serial.print(",");
    Serial.print(frontAngle);
    Serial.println();
}

long distanceInCm() {
    pinMode(US_SENSOR, OUTPUT);
    digitalWrite(US_SENSOR, LOW);
    delayMicroseconds(2);
    digitalWrite(US_SENSOR, HIGH);
    delayMicroseconds(5);
    digitalWrite(US_SENSOR, LOW);

    pinMode(US_SENSOR, INPUT);
    duration = pulseIn(US_SENSOR, HIGH);
    return duration / 29 / 2;
}

void fullStop() {
    motor.run(RELEASE);
    delay(10 * 1000);
}
unsigned long time;int pos = 0;
void loop() 
{ 
    time = millis();
    if (time % 300 == 0) {
      timedAction();
    }
}
