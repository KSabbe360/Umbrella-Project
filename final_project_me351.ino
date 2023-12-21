const byte PIN_BUTTON_INTERRUPT = 2;  // Pin for emergency stop button interrupt

#include <Servo.h>  // Include Servo library
Servo servo;        // Create servo object
const byte SERVO_PIN = 6;  // Servo control pin

#include <Filters.h>                     // Include Filters library for sensor data processing
#include <AH/STL/cmath>                  // Include for standard math functions
#include <AH/Timing/MillisMicrosTimer.hpp> // Include for timing functions
#include <Filters/MedianFilter.hpp>      // Include for MedianFilter class
const double f_s = 100; // Sampling frequency in Hz for median filter
MedianFilter<10, uint16_t> medfilt = {512}; // Median filter for photoresistor data smoothing

#include <Stepper.h>  // Include Stepper motor library
volatile const int stepsPerRevolution = 2048;  // Steps per revolution for the stepper motor
volatile const int RevolutionsPerMinute = 2;   // Speed of stepper motor in RPM
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11); // Initialize stepper motor on pins 8-11

// Photoresistor pin assignments
int photo_pin_north = 0;  // North photoresistor connected to A0
int photo_pin_west = 1;   // West photoresistor connected to A1
int photo_pin_south = 2;  // South photoresistor connected to A2
int photo_pin_east = 3;   // East photoresistor connected to A3
int photo_pin_main = 4;   // Top-center photoresistor connected to A4

// Variable initialization
float photo[5];  // Array to store photoresistor values
int small_index[] = {0,0};  // Array to store indices of brightest and second brightest sensors
int pos = 90;    // Initial servo position
int counter = 0; // Counter for determining when to move
bool direction = true;  // Direction of stepper motor movement
int total_turn_move = 0;  // Total movement of the stepper motor
bool turn_home = true;  // Flag for returning stepper to home position

volatile bool estop = true;  // Emergency stop flag
volatile unsigned long time_interrupt = 0;  // Time of last interrupt

void setup() {
  myStepper.setSpeed(RevolutionsPerMinute);  // Set speed of stepper motor
  servo.attach(SERVO_PIN);  // Attach servo to its control pin

  pinMode(PIN_BUTTON_INTERRUPT, INPUT_PULLUP);  // Set emergency stop button as input with pull-up
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_INTERRUPT), button_interrupt, FALLING);  // Attach interrupt for emergency stop
  
  Serial.begin(115200);  // Start serial communication at 115200 baud rate
}

void loop() {
  while (!estop){  // Run loop unless emergency stop is activated
    // Read and store sensor data (Data adjusted based on sensor error)
    photo[0] = analogRead(photo_pin_north)-15;
    photo[1] = analogRead(photo_pin_west)-80;
    photo[2] = analogRead(photo_pin_south);
    photo[3] = analogRead(photo_pin_east)-4;
    photo[4] = medfilt(analogRead(photo_pin_main));  // Apply median filter to main sensor reading

    // Serial print statements for debugging sensor values
    Serial.print("North:");
    Serial.print(photo[0]);
    Serial.print(",");
    Serial.print("West:");
    Serial.print(photo[1]);
    Serial.print(",");
    Serial.print("South:");
    Serial.print(photo[2]);
    Serial.print(", ");
    Serial.print("East:");
    Serial.print(photo[3]);
    Serial.print(", ");
    Serial.print("Main:");
    Serial.println(photo[4]);
    
    // Condition to determine if the umbrella needs to move or if it's dark enough
    if (photo[4] < 300 && (photo[0] > 200 || photo[1] > 200 || photo[2] > 200 || photo[3] > 200 || photo[4] > 200)){
      if (counter > 2){
        // Determine the brightest and second brightest light sources
        int brightest = 0;
        int bright = 0;
        for (int i = 0; i < 4; i++){
          if (photo[i] > brightest){
            bright = brightest;
            small_index[1] = small_index[0];
            brightest = photo[i];
            small_index[0] = i;
          }
          else if (photo[i] > bright){
            bright = photo[i];
            small_index[1] = i; 
          }
        }
        
        // Determine direction for the turntable to rotate based on sensor readings
        int primary_photo;
        if ((small_index[0] == 3 && small_index[1] == 0) ||  (small_index[0] == 0 && small_index[1] == 3)){
          direction = false; // Rotate right for northeast direction
          primary_photo = 0;
        }
        if ((small_index[0] == 0 && small_index[1] == 1) ||  (small_index[0] == 1 && small_index[1] == 0)){
          direction = true; // Rotate left for northwest direction
          primary_photo = 0;
        }
        if ((small_index[0] == 1 && small_index[1] == 2) ||  (small_index[0] == 2 && small_index[1] == 1)){
          direction = false; // Rotate right for southwest direction
          primary_photo = 2;
        }
        if ((small_index[0] == 2 && small_index[1] == 3) ||  (small_index[0] == 3 && small_index[1] == 2)){
          direction = true; // Rotate left for southeast direction
          primary_photo = 2;
        }
        
        // Variables for controlling turntable movement
        bool turntable = true; // Flag to control turntable loop
        int counter2 = 0; // Counter for filtering number of rotations
        int max_bright = 0; // Variable to store maximum brightness detected
        turn_home = false; // Set flag to indicate turntable has moved from home position
        while (turntable && !estop){  // Loop to control turntable movement
          // Read sensor data at each iteration
          photo[0] = analogRead(photo_pin_north);
          photo[1] = analogRead(photo_pin_west);
          photo[2] = analogRead(photo_pin_south);
          photo[3] = analogRead(photo_pin_east);
          photo[4] = medfilt(analogRead(photo_pin_main));

          // Serial print statements for debugging sensor values
          Serial.print("North:");
          Serial.print(photo[0]);
          Serial.print(",");
          Serial.print("West:");
          Serial.print(photo[1]);
          Serial.print(",");
          Serial.print("South:");
          Serial.print(photo[2]);
          Serial.print(", ");
          Serial.print("East:");
          Serial.print(photo[3]);
          Serial.print(", ");
          Serial.print("Main:");
          Serial.println(photo[4]);

          // Control logic for turning the turntable based on sensor data
          if (direction){   // Check which direction to turn (left = true)
            if (counter2 < 3){  // Motor searching for highest light value
              myStepper.step(stepsPerRevolution/80);  // Step motor for partial rotation
              total_turn_move++;  // Increment total movement counter
              // Update maximum brightness if current reading is higher
              if (photo[primary_photo] >= max_bright-10){
                if (photo[primary_photo] >= max_bright){
                  max_bright = photo[primary_photo];
                }
              }
              else counter2++;  // Increment counter if current brightness is lower
            }
            else{ // When highest light value is reached
              myStepper.step(-stepsPerRevolution/80);  // Step motor in opposite direction for fine adjustment
              total_turn_move--;  // Decrement total movement counter
              // Check if turntable has returned to highest light value within a threshold
              if (photo[primary_photo] >= max_bright-20 || photo[primary_photo] < max_bright-40){
                turntable = false;  // Stop turntable movement
              }
            }
          }
          else{ // Control logic for opposite direction (right = false)
            if (counter2 < 3){
              myStepper.step(-stepsPerRevolution/80);
              total_turn_move--;
              if (photo[primary_photo] >= max_bright-10){
                if (photo[primary_photo] >= max_bright){
                  max_bright = photo[primary_photo];
                }
              }
              else counter2++;
            }
            else{
              myStepper.step(stepsPerRevolution/80);
              total_turn_move++;
              if (photo[primary_photo] >= max_bright-20 || photo[primary_photo] < max_bright-40){
                turntable = false;
              }
            }
          }
          delay(10);  // Short delay for stability
        }
        // Control logic for adjusting the umbrella tilt using the servo motor
        bool umbrella = true;  // Flag to control umbrella adjustment loop
        int counter3 = 0; // Counter for filtering number of adjustments
        int max_bright2 = 0; // Variable to store maximum brightness detected for umbrella adjustment
        while (umbrella && !estop){
          // Read sensor data at each iteration
          photo[0] = analogRead(photo_pin_north);
          photo[1] = analogRead(photo_pin_west);
          photo[2] = analogRead(photo_pin_south);
          photo[3] = analogRead(photo_pin_east);
          photo[4] = medfilt(analogRead(photo_pin_main));

          // Serial print statements for debugging sensor values
          Serial.print("North:");
          Serial.print(photo[0]);
          Serial.print(",");
          Serial.print("West:");
          Serial.print(photo[1]);
          Serial.print(",");
          Serial.print("South:");
          Serial.print(photo[2]);
          Serial.print(", ");
          Serial.print("East:");
          Serial.print(photo[3]);
          Serial.print(", ");
          Serial.print("Main:");
          Serial.println(photo[4]);

          // Adjust umbrella tilt based on primary light source direction
          if (primary_photo == 0){
            if (counter3 < 2){  // Servo motor searching for highest light value
              pos = pos - 1;  // Adjust servo position
              servo.write(pos);  // Write new position to servo
              // Update maximum brightness if current reading is higher
              if (photo[4] >= max_bright2-10){
                if (photo[4] >= max_bright2){
                  max_bright2 = photo[4];
                }
              }
              else counter3++;  // Increment counter if current brightness is lower
              delay(100);  // Delay for stability
            }
            else{ // When highest light value is reached
              pos = pos + 1;  // Adjust servo position in opposite direction
              servo.write(pos);  // Write new position to servo
              // Check if umbrella has returned to highest light value within a threshold
              if (photo[4] >= max_bright2 - 10 || photo[4] < max_bright2-60){
                umbrella = false;  // Stop umbrella adjustment
              }
              delay(100);  // Delay for stability
            }
          }
          else{
            if (counter3 < 2){  // Servo motor searching for highest light value
              pos = pos + 1;
              servo.write(pos);
              if (photo[4] >= max_bright2-10){
                if (photo[4] >= max_bright2){
                  max_bright2 = photo[4];
                }
              }
              else counter3++;
              delay(100);
            }
            else{ // When highest light value is reached
              pos = pos - 1;
              servo.write(pos);
              if (photo[4] >= max_bright2 - 10 || photo[4] < max_bright2-60){
                umbrella = false;
              }
              delay(100);
            }
          }
          delay(20);  // Short delay for stability
        }
        counter = 0;  // Reset counter for next cycle
      }
      else {
        counter++;  // Increment counter if conditions are not met for movement
      }      
    }
    else counter = 0;  // Reset counter if light conditions are not sufficient
    delay(200);  // Delay to avoid too much computing/power consumption
  }
  // Return the turntable to the home position if it has moved
  if (!turn_home){
    myStepper.step( (stepsPerRevolution * (-total_turn_move)) / 80);  // Move stepper to home position
    total_turn_move = 0;  // Reset total movement counter
    turn_home = true;  // Set flag indicating turntable is at home position
  }
}

void button_interrupt() {
  // Debounce and toggle emergency stop flag
  if (millis() - time_interrupt > 500){  // Check if 500ms have passed since last interrupt
    time_interrupt = millis();  // Update time of last interrupt
    estop = !estop;  // Toggle emergency stop flag
  }
}
