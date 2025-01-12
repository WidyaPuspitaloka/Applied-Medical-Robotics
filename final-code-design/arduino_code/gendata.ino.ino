// Communicaiton between Arduino and MATLAB
//   @author         Alejandro Granados
//   @organisation   King's College London
//   @module         Medical Robotics Hardware Development
//   @year           2023

// Global variables
int i = 0;                // counter
String matlabStr = "";    // receives the string from matlab, it is empty at first

// Variables for motor 1
const float PPR_m1 = 3575.0855; // Pulses per Revolution of outer shaft
const float GR_m1 = 297.924;  // Gear Ratio
const float CPR_m1 = 3;

// Variables for motor 2
const float PPR_m2 = 82.77; // Pulses per Revolution of outer shaft
const float GR_m2 = 298;  // Gear Ratio
const float CPR_m2 = 14;


// Variables for tracking encoder positions
volatile long counter_m1 = 0;
volatile long counter_m2 = 0;
int aLastState_m1;
int aLastState_m2;

// PID gains
float Kp = 50; // proportional gaind
float Kd = 3; // derivative gain
float Ki = 0; // integral gain

// Error values
float errorPositionInDegrees_prev_m1 = 0;
float errorPositionInDegrees_sum_m1 = 0;
float errorPositionInDegrees_prev_m2 = 0;
float errorPositionInDegrees_sum_m2 = 0;
int controllerOutput1 = 0;
int controllerOutput2 = 0;

// Time parameters
unsigned long currentTime;
unsigned long previousTime = 0;
unsigned long deltaT;

// Pins for reading encoders of motor 1 and 2
const int encoderPinA_m1 = 2;
const int encoderPinB_m1 = 10;
// const int encoderPinA_m2 = 3;
// const int encoderPinB_m2 = 11;
const int encoderPinA_m2 = 11; // pos: counterclockwise
const int encoderPinB_m2 = 3; // pos: counterclockwise

// Pins for setting the direction of motor 1 and 2
const int motorPin1_m1 = 4;
const int motorPin2_m1 = 5; 
// const int motorPin1_m2 = 8;
// const int motorPin2_m2 = 7;
const int motorPin1_m2 = 7; // follow the new encoder setup
const int motorPin2_m2 = 8;

// Pins for setting the speed of rotation (Enable pin) of motors 1 and 2
const int enablePin_m1 = 6;
const int enablePin_m2 = 9;

// position
long encoderPosition1 = 0;
long encoderPosition2 = 0;
float currentPositionInDegrees_m2;
float currentPositionInDegrees_m1;

// Variables for matlab
bool readyToSend = false;
char c;                   // characters received from matlab
float demandPosition1 = 0.0;         // input1 from matlab
float demandPosition2 = 0.0;         // input2 from matlab


/* Initialisation function of Arduino */
void setup() {
  // configure serial communication speed
  Serial.begin(9600);

  // Setting interrupt pins as inputs
  pinMode(encoderPinA_m1, INPUT_PULLUP);
  pinMode(encoderPinB_m1, INPUT_PULLUP);
  pinMode(encoderPinA_m2, INPUT_PULLUP);
  pinMode(encoderPinB_m2, INPUT_PULLUP);

  // Set the motor control pins as output
  pinMode(motorPin1_m1, OUTPUT);
  pinMode(motorPin2_m1, OUTPUT);
  pinMode(enablePin_m1, OUTPUT);

  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);
  pinMode(enablePin_m2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA_m1), updateEncoder_m1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m2), updateEncoder_m2, CHANGE);

  // Transform position in degrees and store in variable
  aLastState_m1 = digitalRead(encoderPinA_m1);
  aLastState_m2 = digitalRead(encoderPinA_m2);
  
}

/* Continuous loop function in Arduino */
void loop() {
  // First wait to receive data from matlab to arduino
  // if (readyToSend == false) { 
    delay(30);

    if (readyToSend == false) {
      if (Serial.available()>0)       // is there anything received?
      {
        c = Serial.read();            // read characters
        matlabStr = matlabStr + c;    // append characters to string as these are received
        
        if (matlabStr.indexOf(";") != -1) // have we received a semi-colon (indicates end of command from matlab)?
        {
          readyToSend = true;

          // parse incomming data, e.g. C40.0,3.5;
          int posComma1 = matlabStr.indexOf(",");                     // position of comma in string
          demandPosition1 = matlabStr.substring(1, posComma1).toFloat();         // float from substring from character 1 to comma position
          int posEnd = matlabStr.indexOf(";");                        // position of last character
          demandPosition2 = matlabStr.substring(posComma1+1, posEnd).toFloat();  // float from substring from comma+1 to end-1
        }
      }
    }

    if (readyToSend) {
      currentPositionInDegrees_m1 = ((counter_m1 * 360)/ (CPR_m1 * GR_m1 * 2));
      currentPositionInDegrees_m2 = ((counter_m2 * 360)/ (CPR_m2 * GR_m2)); // new motor

      // resetting the degrees when it exceeds -+ 360
      if (currentPositionInDegrees_m1 >= 360.0 || currentPositionInDegrees_m1 <= -360.0) {
        counter_m1 -= ((GR_m1 * CPR_m1 *2) * ((int)(currentPositionInDegrees_m1 / 360)));
      };
      if (currentPositionInDegrees_m2 >= 360.0 || currentPositionInDegrees_m2 <= -360.0) {
        counter_m2 -= ((GR_m2 * CPR_m2 *2) * ((int)(currentPositionInDegrees_m2 / 360)));
      };

      // start time in microseconds
      currentTime = micros();
      deltaT = currentTime - previousTime;
      previousTime = currentTime;

      //
      //
      // Motor 1
      //
      // Calculate PID errors
      float errorPositionInDegrees_m1 = currentPositionInDegrees_m1 - demandPosition1;
      float errorPositionInDegrees_diff_m1 = (errorPositionInDegrees_m1 - errorPositionInDegrees_prev_m1)/deltaT; 
      errorPositionInDegrees_sum_m1 += errorPositionInDegrees_m1;
      errorPositionInDegrees_prev_m1 = errorPositionInDegrees_m1;
  
      // PID terms
      float pOutput1 = errorPositionInDegrees_m1 * Kp;
      float dOutput1 = errorPositionInDegrees_diff_m1 * Kd;
      float iOutput1 = errorPositionInDegrees_sum_m1 * Ki * deltaT;

      // Sum PID
      controllerOutput1 = pOutput1 + dOutput1 + iOutput1;

      // Bound PID output 
      if ((controllerOutput1 >= 0) && (controllerOutput1 < 130)) {
        controllerOutput1 = 130;
      } else if ((controllerOutput1 < 0) && (controllerOutput1 > -130)) {
        controllerOutput1 = -130;
      } else if (controllerOutput1 > 150) {
          controllerOutput1 = 150;
      } else if (controllerOutput1 < -150) {
          controllerOutput1 = -150;
      };

      //  If currentPosition near demandPosition, then stops the motor
      if (abs(errorPositionInDegrees_m1) < 1.0) {
        digitalWrite(motorPin1_m1, LOW);
        digitalWrite(motorPin2_m1, LOW);
      } else {
        // If PID output > 0: set motor to clockwise rotation by the speed of PID output
        if (errorPositionInDegrees_m1 > 1.0) {
          digitalWrite(motorPin1_m1, HIGH);
          digitalWrite(motorPin2_m1, LOW);
          analogWrite(enablePin_m1, controllerOutput1);
        } else {
          digitalWrite(motorPin1_m1, LOW);
          digitalWrite(motorPin2_m1, HIGH);
          analogWrite(enablePin_m1, -controllerOutput1);
        };
      };

      //
      //
      // Motor 2
      // Calculate PID errors
      float errorPositionInDegrees_m2 = currentPositionInDegrees_m2 - demandPosition2;
      float errorPositionInDegrees_diff_m2 = (errorPositionInDegrees_m2 - errorPositionInDegrees_prev_m2)/deltaT; 
      errorPositionInDegrees_sum_m2 += errorPositionInDegrees_m2;
      errorPositionInDegrees_prev_m2 = errorPositionInDegrees_m2;

        // PID terms
      float pOutput2 = errorPositionInDegrees_m2 * Kp;
      float dOutput2 = errorPositionInDegrees_diff_m2 * Kd;
      float iOutput2 = errorPositionInDegrees_sum_m2 * Ki * deltaT;

        // Sum PID
      controllerOutput2 = pOutput2 + dOutput2 + iOutput2;

      // Bound PID output 
      if ((controllerOutput2 >= 0) && (controllerOutput2 < 100)) {
        controllerOutput2 = 100;
      } else if ((controllerOutput2 < 0) && (controllerOutput2 > -100)) {
        controllerOutput2 = -100;
      } else if (controllerOutput2 > 120) {
          controllerOutput2 = 120;
      } else if (controllerOutput2 < -120) {
          controllerOutput2 = -120;
      };
      
      //  If currentPosition near demandPosition, then stops the motor
      if (abs(errorPositionInDegrees_m2) < 1.0) {
        digitalWrite(motorPin1_m2, LOW);
        digitalWrite(motorPin2_m2, LOW);
      } else {
        // If PID output > 0: set motor to clockwise rotation by the speed of PID output
        if (errorPositionInDegrees_m2 > 1.0) {
          digitalWrite(motorPin1_m2, HIGH);
          digitalWrite(motorPin2_m2, LOW);
          analogWrite(enablePin_m2, controllerOutput2);
        } else {
          digitalWrite(motorPin1_m2, LOW);
          digitalWrite(motorPin2_m2, HIGH);
          analogWrite(enablePin_m2, -controllerOutput2);
        };
      };

      if ((abs(errorPositionInDegrees_m1) < 0.5) && (abs(errorPositionInDegrees_m2) < 0.5)) {
        readyToSend = false;
        matlabStr = "";
      }


      // e.g. c1,100
      Serial.print("c");                          // command
      Serial.print(currentPositionInDegrees_m1);  // series 1
      Serial.print(",");                          // delimiter
      Serial.print(currentPositionInDegrees_m2);  // series 2
      Serial.print(","); 
      Serial.print(deltaT); 
      // Serial.print(val2*sin(i*val1/360.0));       // series 2
      Serial.write(13);                           // carriage return (CR)
      Serial.write(10);                           // new line (NL)
      // i += 1;
    }
     else {
        
    }
}


// Interrupt functions for tracking the encoder positions
void updateEncoder_m1() {
  // Code to update counter_m1 based on the state of the encoder pins
  // Read current states of channels A and B
  int aState = digitalRead(encoderPinA_m1);
  int bState = digitalRead(encoderPinB_m1);

  // Check if the state of channel A has changed
  if (aState != aLastState_m1) {
    // Determine the direction of rotation by comparing A and B states
    if (aState != bState) {
      counter_m1++;  // Clockwise rotation
    } else {
      counter_m1--;  // Counterclockwise rotation
    }
      // Update the last known state of channel A
    aLastState_m1 = aState;
  }

}

void updateEncoder_m2() {
  // Code to update counter_m2 based on the state of the encoder pins
    // Read current states of channels A and B
  int aState_m2 = digitalRead(encoderPinA_m2);
  int bState_m2 = digitalRead(encoderPinB_m2);

  // Check if the state of channel A has changed
  if (aState_m2 != aLastState_m2) {
    // Determine the direction of rotation by comparing A and B states
    if (aState_m2 != bState_m2) {
      counter_m2++;  // Clockwise rotation
    } else {
      counter_m2--;  // Counterclockwise rotation
    }
      // Update the last known state of channel A
    aLastState_m2 = aState_m2;
  }
}


