// Description: Arduino code for controlling the motors and sprayers
// Author: Joel Ramsay and Michael Venz
// Date: 2023-05-21
// Version: 1.0
// License: MIT

// Define the pins used by the Arduino Mega
// Define encoder pins
#define RIGHT_CHA 2
#define RIGHT_CHB 8
#define LEFT_CHA 3
#define LEFT_CHB 4

// Define solenoid pins
#define SOL_GND A8
#define SOL_LEFT A9
#define SOL_LEFT_CENTRE A10
#define SOL_CENTRE A11
#define SOL_RIGHT_CENTRE A12
#define SOL_RIGHT A13

// Define PWM pins
#define LEFT_PWM 11
#define RIGHT_PWM 10

// Define motor power pins
#define LEFT_IN1 A2
#define LEFT_IN2 A3
#define RIGHT_IN1 A0
#define RIGHT_IN2 A1

#define LED1 A5 // Left spray solenoid - D13
#define LED2 A6 // Centre spray solenoid - D12
#define LED3 A7 // Right spray solenoid - D11

// Path state constants
int ROBOT_STATUS = 0;        // Path status state
const int ROBOT_STOP = 0;    // Path stop state
const int ROBOT_START = 1;   // Go state - Weed destruction mode
const int ROBOT_LOST = 2;    // Robot lost state
const int ROBOT_PENDING = 3; // Robot pending communiation state

// Robot constants
const int TICKS_PER_REV = 700;                          // Ticks per wheel revolution
const float WHEEL_DIAMETER = 0.1;                       // Wheel diameter (m)
const float WHEEL_CIRC = PI * WHEEL_DIAMETER;           // Wheel circumference (m)
const float DIST_PER_TICK = WHEEL_CIRC / TICKS_PER_REV; // Distance travelled per tick (m)

// Target speed and turning rate variables
const float TARGET_SPEED_KM = 0.2; // Target speed in km/h
float TARGET_SPEED_M_PER_SEC = TARGET_SPEED_KM / 3.6;
float TARGET_TICKS_PER_SEC = int((TARGET_SPEED_M_PER_SEC / WHEEL_CIRC) * TICKS_PER_REV);
float BASE_SPEED = TARGET_TICKS_PER_SEC;

// Set the motor encoder variables
float RIGHT_MOTOR_COUNTS_PER_SECOND = 0;
float LEFT_MOTOR_COUNTS_PER_SECOND = 0;
float LEFT_motor_SETPOINT_COUNTS_PER_SECOND = 0;
float RIGHT_motor_SETPOINT_COUNTS_PER_SECOND = 0;
volatile int LEFT_ENCODER_COUNT = 0;  // universal count
volatile int RIGHT_ENCODER_COUNT = 0; // universal count
volatile int LEFT_PWM_duty_cycle = 0;
volatile int RIGHT_PWM_duty_cycle = 0;

// Define time variables
volatile unsigned long ELAPSEDTIME = 0;
volatile float TIMESTEP = 4; // milliseconds (20Hertz)

// Define serial communication variables
int WEED_BYTE = 0;
int WATCHDOG_BYTE = 0;
int HEADING_ANGLE = 90;
int WATCHDOG = 0;
int SOL0 = 0;
int SOL1 = 0;
int SOL2 = 0;
int SOL3 = 0;
int SOL4 = 0;

void setup()
{
    // Test LEDS
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);

    // Define Solenoid
    pinMode(SOL_GND, OUTPUT);
    pinMode(SOL_LEFT, OUTPUT);
    pinMode(SOL_LEFT_CENTRE, OUTPUT);
    pinMode(SOL_CENTRE, OUTPUT);
    pinMode(SOL_RIGHT_CENTRE, OUTPUT);
    pinMode(SOL_RIGHT, OUTPUT);
    digitalWrite(SOL_GND, LOW);
    digitalWrite(SOL_LEFT, LOW);
    digitalWrite(SOL_LEFT_CENTRE, LOW);
    digitalWrite(SOL_CENTRE, LOW);
    digitalWrite(SOL_RIGHT_CENTRE, LOW);
    digitalWrite(SOL_RIGHT, LOW);
    // Define motor pins
    pinMode(LEFT_CHA, INPUT);
    pinMode(LEFT_CHB, INPUT);
    pinMode(RIGHT_CHA, INPUT);
    pinMode(RIGHT_CHB, INPUT);
    pinMode(LEFT_PWM, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
    analogWrite(LEFT_PWM, 0);
    analogWrite(RIGHT_PWM, 0);
    // Define motor power pins
    pinMode(RIGHT_IN1, OUTPUT);
    pinMode(RIGHT_IN2, OUTPUT);
    analogWrite(RIGHT_IN1, LOW);
    analogWrite(RIGHT_IN2, LOW);

    attachInterrupt(0, RIGHT_flag, RISING);
    attachInterrupt(1, LEFT_flag, RISING);
    Serial.begin(115200);
}

void loop()
{
    // Delay startup 1 seconds - SET TO SMALLER VALUE FOR LIVE VERSION
    delay(1000);

    static unsigned long previousMillis = 0;
    static unsigned long currentMillis;

    ROBOT_STATUS = ROBOT_STOP;
    LEFT_motor_direction(1);  // sets the motor direction
    RIGHT_motor_direction(1); // sets the motor direction

    while (true)
    {
        if (Serial.available() >= 4)
        { // Wait until all bytes are received
            // Read the incoming bytes
            byte buffer[4];
            Serial.readBytes(buffer, 4);
            // Reconstruct the integers from the received bytes
            WEED_BYTE = buffer[0];
            WATCHDOG_BYTE = buffer[1];
            HEADING_ANGLE = buffer[2] | (buffer[3] << 8);

            SOL0 = bitRead(WEED_BYTE, 0); // solenoid 0
            SOL1 = bitRead(WEED_BYTE, 1); // solenoid 1
            SOL2 = bitRead(WEED_BYTE, 2); // solenoid 2

            if (bitRead(WATCHDOG_BYTE, 0))
            {
                ROBOT_STATUS = ROBOT_STOP;
            }
            else if (bitRead(WATCHDOG_BYTE, 1))
            {
                ROBOT_STATUS = ROBOT_START;
            }
            else if (bitRead(WATCHDOG_BYTE, 2))
            {
                ROBOT_STATUS = ROBOT_LOST;
            }
        }

        if (ROBOT_STATUS == ROBOT_STOP)
        {
            robot_stop();
        }

        if (ROBOT_STATUS == ROBOT_START)
        {
            currentMillis = millis();
            ELAPSEDTIME = currentMillis - previousMillis;
            if (ELAPSEDTIME >= TIMESTEP)
            {
                previousMillis = currentMillis;
                direction_control_system();
                RIGHT_motor(LEFT_motor_SETPOINT_COUNTS_PER_SECOND); // encoder count per second, direction.
                LEFT_motor(RIGHT_motor_SETPOINT_COUNTS_PER_SECOND);
                set_solenoids();
            }
        }
    }
} // end loop

// Set the solenoids
void set_solenoids()
{
    if (SOL0 == 1)
    {
        digitalWrite(SOL_LEFT, HIGH);
        digitalWrite(LED1, HIGH);
    }
    else
    {
        digitalWrite(SOL_LEFT, LOW);
        digitalWrite(LED1, LOW);
    }
    if (SOL1 == 1)
    {
        digitalWrite(SOL_CENTRE, HIGH);
        digitalWrite(LED2, HIGH);
    }
    else
    {
        digitalWrite(SOL_CENTRE, LOW);
        digitalWrite(LED2, LOW);
    }
    if (SOL2 == 1)
    {
        digitalWrite(SOL_RIGHT, HIGH);
        digitalWrite(LED3, HIGH);
    }
    else
    {
        digitalWrite(SOL_RIGHT, LOW);
        digitalWrite(LED3, LOW);
    }
}

// Stop the motor
void robot_stop()
{
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(SOL_GND, LOW);
    digitalWrite(SOL_LEFT, LOW);
    digitalWrite(SOL_LEFT_CENTRE, LOW);
    digitalWrite(SOL_CENTRE, LOW);
    digitalWrite(SOL_RIGHT_CENTRE, LOW);
    digitalWrite(SOL_RIGHT, LOW);
    LEFT_motor(0);
    RIGHT_motor(0);
}

// Set the motor control
void direction_control_system()
{
    // Check the encoder counts per second
    float left_counts_per_sec = LEFT_ENCODER_COUNT / (ELAPSEDTIME / 1000.0);
    float right_counts_per_sec = RIGHT_ENCODER_COUNT / (ELAPSEDTIME / 1000.0);

    // Set the global variable
    LEFT_MOTOR_COUNTS_PER_SECOND = left_counts_per_sec;
    RIGHT_MOTOR_COUNTS_PER_SECOND = right_counts_per_sec;

    // Reset the encoder count to zero
    LEFT_ENCODER_COUNT = 0;
    RIGHT_ENCODER_COUNT = 0;

    // Turn right
    if (HEADING_ANGLE < 90)
    {
        float differential_speed = BASE_SPEED * 10.0;
        LEFT_motor_SETPOINT_COUNTS_PER_SECOND = BASE_SPEED + (double)((90.0 - HEADING_ANGLE) / 90.0 * differential_speed);
        RIGHT_motor_SETPOINT_COUNTS_PER_SECOND = BASE_SPEED;
    }
    // Turn left
    else if (HEADING_ANGLE > 90)
    {
        float differential_speed = BASE_SPEED * 10.0;
        LEFT_motor_SETPOINT_COUNTS_PER_SECOND = BASE_SPEED;
        RIGHT_motor_SETPOINT_COUNTS_PER_SECOND = BASE_SPEED + (double)((HEADING_ANGLE - 90.0) / 90.0 * differential_speed);
    }
    // Go straight
    else
    {
        LEFT_motor_SETPOINT_COUNTS_PER_SECOND = BASE_SPEED;
        RIGHT_motor_SETPOINT_COUNTS_PER_SECOND = BASE_SPEED;
    }
}

// Set the left motor speed
void LEFT_motor(int LEFT_motor_SETPOINT_COUNTS_PER_SECOND)
{
    LEFT_motor_control_system();       // sets the new pwm value for the left motor, proportionally, based on the measured speed.
    LEFT_PWM_set(LEFT_PWM_duty_cycle); // updates to pwm
}

// Set the right motor speed
void RIGHT_motor(int RIGHT_motor_SETPOINT_COUNTS_PER_SECOND)
{
    RIGHT_motor_control_system(); // sets the new pwm value for the left motor, proportionally, based on the measured speed.
    RIGHT_PWM_set(RIGHT_PWM_duty_cycle);
}

// Set the PWM to left motor
void LEFT_motor_control_system()
{
    if (abs(LEFT_MOTOR_COUNTS_PER_SECOND) < abs(LEFT_motor_SETPOINT_COUNTS_PER_SECOND))
    {
        LEFT_PWM_duty_cycle = 255; // incease PWM rate
    }
    if (abs(LEFT_MOTOR_COUNTS_PER_SECOND) > abs(LEFT_motor_SETPOINT_COUNTS_PER_SECOND))
    {
        LEFT_PWM_duty_cycle = 0; // incease PWM rate
    }
}

// Set the PWM to right motor
void RIGHT_motor_control_system()
{
    if (abs(RIGHT_MOTOR_COUNTS_PER_SECOND) < abs(RIGHT_motor_SETPOINT_COUNTS_PER_SECOND))
    {
        RIGHT_PWM_duty_cycle = 255; // incease PWM rate
    }
    if (abs(RIGHT_MOTOR_COUNTS_PER_SECOND) > abs(RIGHT_motor_SETPOINT_COUNTS_PER_SECOND))
    {
        RIGHT_PWM_duty_cycle = 0; // incease PWM rate
    }
}

// Set the left PWM
void LEFT_PWM_set(int LEFT_PWM_duty_cycle)
{
    analogWrite(LEFT_PWM, LEFT_PWM_duty_cycle);
}

// Set the right PWM
void RIGHT_PWM_set(int RIGHT_PWM_duty_cycle)
{
    analogWrite(RIGHT_PWM, RIGHT_PWM_duty_cycle);
}

// Set the left motor direction
void LEFT_motor_direction(int motor_direction)
{
    if (motor_direction == 0)
    { // if motor direction is 0, got forward.
        digitalWrite(LEFT_IN1, LOW);
        digitalWrite(LEFT_IN2, HIGH);
    }
    if (motor_direction == 1)
    { // if motor direction is 1, got reverse.
        digitalWrite(LEFT_IN1, HIGH);
        digitalWrite(LEFT_IN2, LOW);
    }
}

// Set the right motor direction
void RIGHT_motor_direction(int motor_direction)
{
    if (motor_direction == 1)
    { // if motor direction is 1, got forward.
        digitalWrite(RIGHT_IN1, LOW);
        digitalWrite(RIGHT_IN2, HIGH);
    }
    if (motor_direction == 0)
    { // if motor direction is 0, got reverse.
        digitalWrite(RIGHT_IN1, HIGH);
        digitalWrite(RIGHT_IN2, LOW);
    }
}

// Read the left encoder
void LEFT_flag()
{
    // add 1 to count for CW
    if (digitalRead(LEFT_CHA) && !digitalRead(LEFT_CHB))
    {
        LEFT_ENCODER_COUNT++;
    }
    // subtract 1 from count for CCW
    if (digitalRead(LEFT_CHA) && digitalRead(LEFT_CHB))
    {
        LEFT_ENCODER_COUNT--;
    }
}

// Read the right encoder
void RIGHT_flag()
{
    // add 1 to count for CW
    if (digitalRead(RIGHT_CHA) && !digitalRead(RIGHT_CHB))
    {
        RIGHT_ENCODER_COUNT--;
    }
    // subtract 1 from count for CCW
    if (digitalRead(RIGHT_CHA) && digitalRead(RIGHT_CHB))
    {
        RIGHT_ENCODER_COUNT++;
    }
}