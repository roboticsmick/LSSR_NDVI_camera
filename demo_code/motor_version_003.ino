// Description: Arduino code for controlling the motors and sprayers
// Author: Michael Venz
// Date: 2023-05-21
// Version: 1.0
// License: MIT

// Define the pins used by the Arduino Mega
// Sprayer pins
#define LEFT_SPRAY 5   // Left spray solenoid - D13
#define CENTRE_SPRAY 6 // Centre spray solenoid - D12
#define RIGHT_SPRAY 7  // Right spray solenoid - D11

// Left Motor pins
#define LEFT_CHA 3  // Left Encoder A - INT5 / D3
#define LEFT_CHB 8  // Left Encoder B - D8
#define LEFT_PWM 11 // Left L298N PWM - Grey - D11
#define LEFT_IN1 A2 // Left L298N IN1 - White/Blue - A2
#define LEFT_IN2 A3 // Left L298N IN2 - Purple - A3
// Right Motor pins
#define RIGHT_CHA 2  // Right Encoder A - INT4 / D2
#define RIGHT_CHB 4  // Right Encoder B - D4
#define RIGHT_PWM 10 // Right L298N PWM - Grey - D10
#define RIGHT_IN1 A0 // Right L298N IN1 - White/Blue - A0
#define RIGHT_IN2 A1 // Right L298N IN2 - Purple - A1

// Robot constants
const int TICKS_PER_REV = 700;                                     // Ticks per wheel revolution
const float WHEEL_DIAMETER = 0.1;                                  // Wheel diameter (m)
const float WHEEL_CIRC = PI * WHEEL_DIAMETER;                      // Wheel circumference (m)
const float DIST_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REV; // Distance travelled per tick (m)

// Motor speed constants
const int MAX_PWM_SPEED = 140; // Maximum motor speed
const int MIN_PWM_SPEED = 60;  // Minimum motor speed
const int STOP = 0;            // Stop motor speed

// Motor direction constants
const int MOTOR_FORWARD = 0; // Set motor direction to forward
const int MOTOR_REVERSE = 1; // Set motor direction to reverse

// Target speed and turning rate variables
const float TARGET_SPEED_KM = 1.0; // Target speed in km/h
const float MAX_ANGLE_SCALE = 1.5; // Maximum angle scale
const float MIN_SPEED_KM = 0.2;    // Target speed in km/h

// Path state constants
const int ROBOT_STATUS = 0;    // Path status state
const int PATH_LOST = 0;       // Path lost state
const int DETECTION_MODE = 1;  // Go state - Weed destruction mode
const int LOST_CONNECTION = 2; // Robot connection lost state
const int ROBOT_STOP = 3;      // Robot stop state

// Error constants
const int ROBOT_CONNECTION_ERROR_THRESHOLD = 100; // Error connection counter limit
const int ROBOT_LOST_ERROR_THRESHOLD = 50;        // Error connection counter limit

// Global variables
volatile int LEFT_TICKS = 0;          // Store previous Tick count
volatile int RIGHT_TICKS = 0;         // Store previous Tick count
volatile int LEFT_SPEED = 0;          // Left motor speed
volatile int RIGHT_SPEED = 0;         // Right motor speed
volatile int HEADING_ANGLE = 90;      // Robot heading angle
volatile int WEED = 0;                // Weed detected
volatile int WATCHDOG = 0;            // Watchdog status
volatile unsigned long PREV_TIME = 0; // Variable to store the previous time

// Setup function to initialise serial and pin modes
void setup()
{
    Serial.begin(115200); // Set the baud rate to match the Raspberry Pi
    // Initialize the sprayer pins and set to off
    EncoderInit();
    SprayerInit();
}

// Function to initialize the encoder pins
void EncoderInit()
{
    // Initialize left motor pins and set to off
    pinMode(LEFT_CHA, INPUT);
    pinMode(LEFT_CHB, INPUT);
    // Attach interrupts to the encoder pins
    attachInterrupt(digitalPinToInterrupt(LEFT_CHA), leftEncoder, CHANGE);
    pinMode(LEFT_PWM, OUTPUT);
    pinMode(LEFT_IN1, OUTPUT);
    pinMode(LEFT_IN2, OUTPUT);
    digitalWrite(LEFT_PWM, HIGH);
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(LEFT_PWM, 0);

    // Initialize right motor pins and set to off
    pinMode(RIGHT_CHA, INPUT);
    pinMode(RIGHT_CHB, INPUT);

    // Attach interrupts to the encoder pins
    attachInterrupt(digitalPinToInterrupt(RIGHT_CHA), rightEncoder, CHANGE);
    pinMode(RIGHT_PWM, OUTPUT);
    pinMode(RIGHT_IN1, OUTPUT);
    pinMode(RIGHT_IN2, OUTPUT);
    digitalWrite(RIGHT_PWM, HIGH);
    analogWrite(RIGHT_IN1, LOW);
    analogWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_PWM, 0);

    // Set the motor direction to forward by default
    setMotorDirection(MOTOR_FORWARD, MOTOR_FORWARD);
}

// Function to initialize the sprayer pins
void SprayerInit()
{
    // Initialize the sprayer pins and set to off
    pinMode(LEFT_SPRAY, OUTPUT);
    pinMode(CENTRE_SPRAY, OUTPUT);
    pinMode(RIGHT_SPRAY, OUTPUT);
    digitalWrite(LEFT_SPRAY, LOW);
    digitalWrite(CENTRE_SPRAY, LOW);
    digitalWrite(RIGHT_SPRAY, LOW);
}

// Function to read left motor encoder
void leftEncoder()
{
    // add 1 to count for CW
    if (digitalRead(LEFT_CHA) && !digitalRead(LEFT_CHB))
    {
        LEFT_TICKS++;
    }
    // subtract 1 from count for CCW
    if (digitalRead(LEFT_CHA) && digitalRead(LEFT_CHB))
    {
        LEFT_TICKS--;
    }
}

// Function to read right motor encoder
void rightEncoder()
{
    // add 1 to count for CW
    if (digitalRead(RIGHT_CHA) && !digitalRead(RIGHT_CHB))
    {
        RIGHT_TICKS++;
    }
    // subtract 1 from count for CCW
    if (digitalRead(RIGHT_CHA) && digitalRead(RIGHT_CHB))
    {
        RIGHT_TICKS--;
    }
}

// Function to reset the encoder tick counts
void resetEncoders()
{
    LEFT_TICKS = 0;
    RIGHT_TICKS = 0;
}

// Function to calculate the required speed for each motor based on angle
void setMotorSpeedAngle(int direction_angle, float target_speed)
{
    // If the robot is stopped, set the speed to the minimum speed
    if (LEFT_SPEED == STOP && RIGHT_SPEED == STOP)
    {
        LEFT_SPEED = MIN_PWM_SPEED;
        RIGHT_SPEED = MIN_PWM_SPEED;
    }
    else
    {
        // Initialise scale variables
        float left_scale = 1.0;
        float right_scale = 1.0;
        float angle_scale = 1.0;

        // Calculate target speed in metres per second and ticks per second
        float target_speed_m_per_sec = target_speed / 3.6;
        float target_ticks_per_sec = (target_speed_m_per_sec / WHEEL_CIRC) * TICKS_PER_REV;

        // Calculate the elapsed time since the last call
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - PREV_TIME;
        PREV_TIME = currentTime;

        // Calculate the current wheel speed in metres per second
        float left_speed = DIST_PER_TICK * LEFT_TICKS / elapsedTime;
        float right_speed = DIST_PER_TICK * RIGHT_TICKS / elapsedTime;

        Serial.println("Encoder: \n");
        Serial.println(LEFT_TICKS);
        Serial.println(RIGHT_TICKS);
        Serial.println("Encoder Speeds: \n");
        Serial.println(left_speed);
        Serial.println(right_speed);

        // Reset encoder ticks to zero
        resetEncoders();

        // Constrain the angle within the range of 0 to 180 degrees
        direction_angle = constrain(direction_angle, 0, 180);

        if (direction_angle == 90)
        {
            // Adjust the left and right target speeds based on the target speed
            left_scale = target_speed_m_per_sec / left_speed;
            right_scale = target_speed_m_per_sec / right_speed;
        }
        else if (direction_angle > 90)
        {
            // Calculate the scaling factor based on the angle greater than 90 degrees
            angle_scale = 1.0 + ((MAX_ANGLE_SCALE - 1.0) * (direction_angle - 90.0) / 90.0);

            // Increase the right motor speed and decrease the left motor speed
            float left_target = target_speed_m_per_sec / angle_scale;
            float right_target = target_speed_m_per_sec * angle_scale;

            // Adjust the left and right target speeds based on the target speed
            left_scale = left_target / left_speed;
            right_scale = right_target / right_speed;
        }
        else if (direction_angle < 90)
        {
            // Calculate the scaling factor based on the angle less than 90 degrees
            angle_scale = 1 + ((MAX_ANGLE_SCALE - 1) * (1 * direction_angle / 90));

            // Increase the left motor speed and decrease the right motor speed
            float left_target = target_speed_m_per_sec * angle_scale;
            float right_target = target_speed_m_per_sec / angle_scale;

            // Adjust the left and right target speeds based on the target speed
            left_scale = left_target / left_speed;
            right_scale = right_target / right_speed;
        }

        // Apply the scaling factor to the motor speeds
        int set_left_pwm = int(LEFT_SPEED * left_scale);
        int set_right_pwm = int(RIGHT_SPEED * right_scale);

        // Apply constraints to the motor speeds
        set_left_pwm = constrain(LEFT_SPEED, MIN_PWM_SPEED, MAX_PWM_SPEED);
        set_right_pwm = constrain(RIGHT_SPEED, MIN_PWM_SPEED, MAX_PWM_SPEED);
    }

    // Print the motor speeds to the serial monitor
    Serial.println("Motor Speeds: \n");
    Serial.println(set_left_pwm);
    Serial.println(set_right_pwm);

    // Use the calculated motor speeds to control the motors
    setMotorSpeed(set_left_pwm, set_right_pwm);
}

// Set the motors to spin slowly in opposite directions to find path
void spinRobotMotors()
{
    // Print the motor speeds to the serial monitor
    setMotorDirection(MOTOR_FORWARD, MOTOR_REVERSE);
    // Use the calculated motor speeds to control the motors
    // Set heading angle to 90 degrees to spin robot
    HEADING_ANGLE = 90;
    setMotorSpeedAngle(HEADING_ANGLE, MIN_SPEED_KM);
}

// Set the motor speeds to zero to stop the motors
void motorStop()
{
    // Set the motor speed to zero
    LEFT_SPEED = STOP;
    RIGHT_SPEED = STOP;
    // Stop the motors
    analogWrite(LEFT_PWM, LEFT_SPEED);
    analogWrite(RIGHT_PWM, RIGHT_SPEED);
    Serial.println("Motor stopped.\n");
}

// Set the motor speeds
void setMotorSpeed(int left_speed, int right_speed)
{
    // Update the motor speeds
    LEFT_SPEED = left_speed;
    RIGHT_SPEED = right_speed;
    // Set the motor speed
    analogWrite(LEFT_PWM, LEFT_SPEED);
    analogWrite(RIGHT_PWM, RIGHT_SPEED);
}

// Set the motor directions
void setMotorDirection(int left_direction, int right_direction)
{
    if (left_direction == MOTOR_FORWARD)
    {
        // If left motor direction is 0, go forward.
        digitalWrite(LEFT_IN1, HIGH);
        digitalWrite(LEFT_IN2, LOW);
    }
    else if (left_direction == MOTOR_REVERSE)
    {
        // If left motor direction is 1, go reverse.
        digitalWrite(LEFT_IN1, LOW);
        digitalWrite(LEFT_IN2, HIGH);
    }

    if (right_direction == MOTOR_FORWARD)
    {
        // If right motor direction is 0, go forward.
        digitalWrite(RIGHT_IN1, LOW);
        digitalWrite(RIGHT_IN2, HIGH);
    }
    else if (right_direction == MOTOR_REVERSE)
    {
        // If right motor direction is 1, go reverse.
        digitalWrite(RIGHT_IN1, HIGH);
        digitalWrite(RIGHT_IN2, LOW);
    }
    Serial.println("Motor direction set\n");
}

// Main loop
void loop()
{
    // Delay startup 1 seconds - SET TO SMALLER VALUE FOR LIVE VERSION
    delay(1000);

    // Set the spray to off
    int LEFT_SPRAY_BYTE = 0;
    int CENTRE_SPRAY_BYTE = 0;
    int RIGHT_SPRAY_BYTE = 0;

    // Initialise error counter
    int connection_lost_counter = 0;
    int path_lost_counter = 0;
    int path_not_found_counter = 0;

    while (true)
    {
        // Read the serial buffer and parse the data
        if (Serial.available() >= 3)
        {
            // Read the first three bytes from the serial buffer
            byte WEED_BYTE = Serial.read();     // Signal to send solonoids - Read first 3 bits of byte e.g. 010)
            byte HEADING_BYTE = Serial.read();  // Angle (degrees) between 0 and 180 degrees
            byte WATCHDOG_BYTE = Serial.read(); // Watchdog - Stop byte (0 or 1)

            // Read byte and set the solenoid spray states
            LEFT_SPRAY_BYTE = bitRead(WEED, 0);
            CENTRE_SPRAY_BYTE = bitRead(WEED, 1);
            RIGHT_SPRAY_BYTE = bitRead(WEED, 2);

            // Convert the angle byte to an integer to set the motor speeds
            HEADING_ANGLE = (int)HEADING_BYTE;

            // Read the WATCHDOG byte to determine the robot state
            WATCHDOG = bitRead(WATCHDOG_BYTE, 0);

            // Reset connection lost counter
            connection_lost_counter = 0;
        }
        else
        {
            // Set watchdog to pending Rasberry Pi communication state
            WATCHDOG = 3;
        }
        switch (WATCHDOG)
        {
        case 0:
            // Set the path status stop
            ROBOT_STATUS = ROBOT_STOP;

            // Stop the robot motors
            motorStop();

            // Turn off the solenoids
            digitalWrite(LEFT_SPRAY, LOW);
            digitalWrite(CENTRE_SPRAY, LOW);
            digitalWrite(RIGHT_SPRAY, LOW);

            // Set watchdog to pending Rasberry Pi communication state
            WATCHDOG = 3;
            break;

        // Path detected - normal operation
        case 1:
            // If path was in lost state and now found, set the motor direction to forward
            if (ROBOT_STATUS != DETECTION_MODE)
            {
                // Set the path status to found
                ROBOT_STATUS = DETECTION_MODE;
                // Reset the path lost counter
                path_lost_counter = 0;
                path_not_found_counter = 0;
                // Set the motor direction to forward
                setMotorDirection(MOTOR_FORWARD, MOTOR_FORWARD);
            }

            // If path detected set the wheel directions to the angle of the path
            setMotorSpeedAngle(HEADING_ANGLE, TARGET_SPEED_KM);

            // Extract individual bits from the value and control the solenoid spray states
            digitalWrite(LEFT_SPRAY, LEFT_SPRAY_BYTE);
            digitalWrite(CENTRE_SPRAY, CENTRE_SPRAY_BYTE);
            digitalWrite(RIGHT_SPRAY, RIGHT_SPRAY_BYTE);

            // Set watchdog to pending Raspberry Pi communication state
            WATCHDOG = 3;
            break;

        // The robot has lost the path
        case 2:
            // Set the path status to lost
            ROBOT_STATUS = PATH_LOST;

            // Set the motor direction to minimum speed
            setMotorSpeedAngle(HEADING_ANGLE, MIN_SPEED_KM);

            // Increase path lost counter by 1
            if (path_lost_counter >= ROBOT_LOST_ERROR_THRESHOLD)
            {
                // The robot has lost the path for too long, set the robot to spinning state to find path
                spinRobotMotors();

                // If path not found for too long, stop the robot
                if (path_not_found_counter >= ROBOT_LOST_ERROR_THRESHOLD)
                {
                    // Stop the robot
                    Serial.println("Path not found. Stopping robot.\n");
                    motorStop();
                    WATCHDOG = 0;
                }
                else
                {
                    path_not_found_counter++;
                }
            }
            else
            {
                path_lost_counter++;
            }

            // Set watchdog to pending Raspberry Pi communication state
            WATCHDOG = 3;
            break;

        // No communication from the Raspberry Pi was detected
        case 3:
            // Increase connection lost counter by 1
            if (connection_lost_counter >= ROBOT_CONNECTION_ERROR_THRESHOLD)
            {
                // Stop the robot motors
                motorStop();

                // Turn off the solenoids
                digitalWrite(LEFT_SPRAY, LOW);
                digitalWrite(CENTRE_SPRAY, LOW);
                digitalWrite(RIGHT_SPRAY, LOW);

                // Set the path status to lost
                ROBOT_STATUS = ROBOT_STOP;

                // Stop the robot
                Serial.println("Connection lost with Raspberry Pi. Robot stopped.\n");
            }
            else
            {
                connection_lost_counter++;
            }
            break;
        }
    }
}