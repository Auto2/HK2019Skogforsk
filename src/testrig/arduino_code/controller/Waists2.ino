//      This code is used to control the steering of the XT-28 test rig. A value sent to
//      the Arduino will be interpreted as a desired steering angle for waist 1 which this 
//      code will aim to steer the XT-28 test rig to.

//      2019-09-05
//      Axel Sundkvist

//      How to? Write a desired value (0-1024 (10-bit)) that you want the forward-most waist 
//      to turn to.

//      |---------------------------------------------------------------------------|
//      |            The hardware connections are as follows:                       |
//      |--------------------------------|------------------------------------------|
//      |  Connector at Arduino          |      Connector at BB-VNH3SP30      no 1  |
//      |--------------------------------|------------------------------------------|
//      |    Not connected!              |          CTRL<1>,  VIN                   |
//      |    Power, 5V                   |          CTRL<2>,  +5V                   |
//      |    Power, GND                  |          CTRL<3>,  GND                   |
//      |    Digital<13>                 |          CTRL<4>,  INA                   |
//      |    Digital<12>                 |          CTRL<5>,  INB                   |
//      |    Digital<11>                 |          CTRL<6>,  PWM                   |
//      |--------------------------------|------------------------------------------|

//      |---------------------------------------------------------------------------|
//      |            The hardware connections are as follows:                       |
//      |--------------------------------|------------------------------------------|
//      |  Connector at Arduino          |      Connector at BB-VNH3SP30      no 2  |
//      |--------------------------------|------------------------------------------|
//      |    Not connected!              |          CTRL<1>,  VIN                   |
//      |    Power, 5V                   |          CTRL<2>,  +5V                   |
//      |    Power, GND                  |          CTRL<3>,  GND                   |
//      |    Digital<7>                  |          CTRL<4>,  INA                   |
//      |    Digital<6>                  |          CTRL<5>,  INB                   |
//      |    Digital<5>                  |          CTRL<6>,  PWM                   |
//      |--------------------------------|------------------------------------------|


// ----------------- Arduino Pins ----------------------------------------
const int M1Apin = 13;    // Motor control output for M1 forward
const int M1Bpin = 12;    // Motor control output for M1 backward
const int M2Apin = 7;     // Motor control output for M2 forward
const int M2Bpin = 6;    // Motor control output for M2 backward
const int pwm1Pin = 11;   // pwm output for M1
const int pwm2Pin = 5;    // pwm output for M2

const int pot1Pin = A0;   // Potentiometer input for M1
const int pot2Pin = A1;   // Potentiometer input for M2

//-------------------------------------------------------------------------

//------------------ Variables to change -----------------------------------
float pot1Ref = 500;      // Potentiometer reference value for M1 (to be chosen), 500 is in the middle
float pot2Ref = 500;      // Potentiometer reference value for M2 (to be chosen), 500 is in the middle
float factor = 0.5;       // Scaling factor (speed and distance) between waist 1 and 2 (M1 and M2)
float pwm1 = 200;         // PWM duty cycle for waist 1 (M1)
float REF_MAX = 900;	  // Maximum turning reference for waists
float REF_MIN = 100;      // Minimum turning reference for waists

//--------------------------------------------------------------------------

//------------------ Variables ---------------------------------------------
float pwm2 = pwm1*factor; // PWM duty cycle for waist 2 (M2), calculated as "pwm1*factor"
float lastpot1Ref;        // To save last iteration's reference value for M1
float lastpot2Ref;        // To save last iteration's reference value for M2
float pot1Val;            // Up to date input value from potentiometer in waist 1 (M1)
float pot2Val;            // Up to date input value from potentiometer in waist 2 (M2)
int resol = 2;            // Resolution of when position is accepted ("potRef +- resol" is accepted as "goal reached")
int twist_rate = 5;       //This number determines how fast the waists twist when a twist command is issued
int input;

ros::NodeHandle nh; 						  // Nodehandle is an object representing the ROS node, start the ROS node
ros::Subscriber<std_msgs::Int64> pin_sub("motor_action", twist_callback);     

void setup() {

  nh.initNode();
  //nh.subscribe(waist_angle);

  // configure pins
  pinMode(M1Apin, OUTPUT);
  pinMode(M1Bpin, OUTPUT);
  pinMode(M2Apin, OUTPUT);
  pinMode(M2Bpin, OUTPUT);
  pinMode(pwm1Pin, OUTPUT);
  pinMode(pot1Pin, INPUT);
  pinMode(pot2Pin, INPUT);
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps

  // set PWM duty cycle
  analogWrite(pwm1Pin, pwm1);
  analogWrite(pwm2Pin, pwm2);
}//setup

void twist_callback(const std_msgs::Int64& action_msg){ //constant address, checks which move command
	int action = action_mgs.data;
	if(action == 6 || action == 0){		//if forward/reverse left
		pot1Ref -= twist_rate;		//twist left
		if(pot1Ref <= REF_MIN){
			pot1Ref = pot1Val;
		}
	}
	else if(action == 8 || action == 2){	//if forward/reverse left
		pot1Ref += twist_rate;		//twist right
		if(pot1Ref >= REF_MAX){
			pot1Ref = pot1Val;
		}
	}
	else if(action == 7 || action == 1){
		pot1Ref = 500;
	}
	else{ //specifiera att action == 4?wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwa
		pot1Ref = pot1Val; //no turning of waist 
	}
	
}//twist_callback

void loop() {
    //------------------------------ Read desired value --------------------------------------
    if (Serial.available() > 0) {                                // if serial message is availabe
      input = Serial.parseInt();                                 // store as int
      if (input > 0 && input < 1024)                             // if value between min and max (1024 since 10-bit)
      {
        lastpot2Ref = pot2Ref;                                   // store last value for M1
        lastpot1Ref = pot1Ref;                                   // store last value for M2
        pot1Ref = input;                                         // set desired value as reference for M1
        pot2Ref = lastpot2Ref+(pot1Ref-lastpot1Ref)*factor;      // calculate reference value for M2 and set it
      }
    }
    //----------------------------------------------------------------------------------------

    //------------------------------ Waist 1 (M1) --------------------------------------------
    pot1Val = analogRead(pot1Pin);                               // Read potentiometer value for M1
  
    if (pot1Val < pot1Ref - resol) {
      // if reference value smaller than actual - go "backward"
      digitalWrite(M1Bpin, HIGH);
      digitalWrite(M1Apin, LOW);
    }
    else if (pot1Val > pot1Ref + resol) {
      // if reference value larger than actual - go "forward"
      digitalWrite(M1Bpin, LOW);
      digitalWrite(M1Apin, HIGH);
    }
    else {
      // goal reached!
      digitalWrite(M1Bpin, LOW);
      digitalWrite(M1Apin, LOW);
    }
    //---------------------------------------------------------------------------------------

    //------------------------------ Waist 2 (M2) --------------------------------------------
    pot2Val = analogRead(pot2Pin);                               // Read potentiometer value for M1
  
    if (pot2Val < pot2Ref - resol) {
      // if reference value smaller than actual - go "backward"
      digitalWrite(M2Bpin, HIGH);
      digitalWrite(M2Apin, LOW);
    }
    else if (pot2Val > pot2Ref + resol) {
      // if reference value larger than actual - go "forward"
      digitalWrite(M2Bpin, LOW);
      digitalWrite(M2Apin, HIGH);
    }
    else {
      // goal reached!
      digitalWrite(M2Bpin, LOW);
      digitalWrite(M2Apin, LOW);
    }
    //---------------------------------------------------------------------------------------
}
