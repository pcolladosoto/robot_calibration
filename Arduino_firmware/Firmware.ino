 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            DIFFERENTIAL ROBOT FIRMWARE                                            //
//                                      DEVELOPED AT THE UNIVERSITY OF ALCALÁ                                        //
// You can find more information at: www.hindawi.com/journals/js/2019/8269256/?utm_medium=author&utm_source=Hindawi  //
//                                              Please include reference                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Date 4/07/19

// Version
#define VERSION "V1.00"

//Max number of digits for updated data
#define N_DIGS 5

// Define to use L298 test board or the Padrino Tecnologico one
//#define L298TEST

// Defines to setup for LOLA robot or for SRE platform
//#define ROBOT_LOLA
#define ROBOT_SRE

//////////////////////////////////
//      ERROR VARIABLE          //
// IDENTIFICATION ERRORS CODES  //
//////////////////////////////////
unsigned char ERROR_CODE = 0;
#define NO_ERROR              0
#define NO_NUMBER             1   // Waiting for a number, time-out
#define OUT_RANGE             2   // Received number out of range
#define SPEED_OUT_RANGE       3   // Received speed out of range
#define RR_OUT_RANGE          4   // Radio Ratio out of range
#define NO_AVAILABLE          5   // Received Command non available
#define INERTIA_LIMIT_ERROR   6   // Distance lower than inertia limit

  //Left Ultrasound sensor
  #define L_US_TRIG 41
  #define L_US_ECHO 39
  //Right Ultrasound sensor
  #define R_US_TRIG 53
  #define R_US_ECHO 51

// Global robot position
float X = 0, Y = 0, Theta = 0;

// Calibration constant
#ifdef ROBOT_LOLA
  float KKI = 1;                  // deviation from theoretical left wheel diameter,rigth wheel as reference >1 wheel bigger than nominal

  int   WHEEL_DIST = 190;         // Wheel distance in mm
  float mmperpulse = 0.2094;      // mm per pulse in the encoders
  #define INERTIA_LIMIT  40       // Inertia limit to stop few pulses before the limit
  #define PID_TIME       200      // Time in miliseconds for each iteration of the PID
  #define BREAK_PULSES   150      // Number of pulses from the end to start breaking
  #define IGNORE_PULSE   11000    // time in micros to ignore encoder pulses if faster

  // PID (PD) constants
  // float kp = 0.1;
  // float kd = 0.4;
  float kp = 0.5; // 0.23*2;
  float kd = 2; // 10.0*2;
#else
  float KKI = 1; // Deviation from theoretical left wheel diameter; the rigth wheel is the reference; >1 wheel bigger than the nominal one

  //Calibration correction factor for turning
  int c_factor = 1;

  //Current_pulses and last_pulses for PID calibration
  float WHEEL_DIST = 535;         //UAH 578; //UMB          //575;      // Wheel distance in mm
  float mmperpulse = 1.68;         // mm per pulse in the encoders
  #define INERTIA_LIMIT  1            // Inertia limit to stop few pulses before the limit
  #define PID_TIME       500          // Time in miliseconds for each iteration of the PID
  #define BREAK_PULSES   1            // Number of pulses from the end to start breaking
  #define IGNORE_PULSE   11000        // time in micros to ignore encoder pulses if faster
  #define PID_PULSES 10

  // PID (PD) constants
  //float kp =  0.5;
  //float kd = 20.0; //kd = 20.0
  float kp =  0.23 / 5;
  float kd = 10.0 / 5;
#endif
unsigned char SPEED_INI_L = 255;  // 170
unsigned char SPEED_INI_R = 255;  // 100

unsigned char TEST_distances[2000];
unsigned char TEST_pulses[2000];
unsigned int test_counter = 0;

// PINS SPECIFICATIONS
// Connect motor controller pins to Arduino digital pins
#ifdef L298TEST
  // Motor One
  int MOT_L_PWM_PIN = 10;
  int MOT_L_B_PIN = 9;      // Input 1
  int MOT_L_A_PIN = 8;      // Input 2
  //#define MOT_R_ENC_A_PIN 21
  #define MOT_L_ENC_B_PIN 19

  // Motor Two
  int MOT_R_PWM_PIN =  5;
  int MOT_R_B_PIN = 7;      // Input 3
  int MOT_R_A_PIN = 6;      // Input 4
  //#define MOT_L_ENC_A_PIN 19
  #define MOT_R_ENC_B_PIN 20

  //Front Ultrasound sensor
  #define F_US_ECHO 11
  #define F_US_TRIG 12

#else
  // R Motor
  #define MOT_R_PWM_PIN   10 // PB4
  #define MOT_R_A_PIN     28 // PA6
  #define MOT_R_B_PIN     24 // PA2
  // #define MOT_R_ENC_A_PIN 21
  #define MOT_R_ENC_B_PIN 20
  // L Motor
  #define MOT_L_PWM_PIN   11  //PB5
  #define MOT_L_A_PIN     26 // PA4
  #define MOT_L_B_PIN     22 // PA0
  // #define MOT_L_ENC_A_PIN 19
  #define MOT_L_ENC_B_PIN 18

  // Front Ultrasound sensor
  #define F_US_ECHO 47
  #define F_US_TRIG 49

#endif

#define LED 13


// Direction of movement
unsigned char dir_right, dir_left;

// Variables to keep each encoder pulse
volatile unsigned int encoderIZQ = 0, encoderDER = 0;

//These variables keep track of the pulses received with a delay shorter than IGNORE_PULSE microseconds
volatile unsigned int ignored_left = 0, ignored_right = 0;

// Variables to obtain the robot's position and orientation (X, Y, Theta)
unsigned int aux_encoderIZQ = 0, aux_encoderDER = 0;
volatile signed int encoder = 0;
//unsigned long pulsesDER=0;
//unsigned long pulsesIZQ=0;

// Auxiliary variables to filter false impulses from encoders
volatile unsigned long auxr = 0, auxl = 0;

// Auxiliary variables to keep micros() at the encoders
unsigned long tr, tl;

// Radii relation allows to describe a circular movement
float radii_relation = 1.0;

// Indicate the PWM duty cycle that is applied to the motors
int velr = SPEED_INI_R, vell = SPEED_INI_L, error = 0, encoder_ant;
unsigned int PULSES_NUM;

// FSM's STATES
unsigned char STATE = 0;
#define REST_STATE        0
#define ROTATE_CCW_STATE  2
#define ROTATE_CW_STATE    3
#define MOVE_STRAIGHT_STATE  4
#define RIGHT_FASTER_STATE  5
#define LEFT_FASTER_STATE   6
#define CIRC_TEST_R_STATE   7
#define CIRC_TEST_L_STATE   8

unsigned char order[1];

// Indicate if the rotation is clockwise or counterclockwise
unsigned char clockwise = 0;

// Keeps the last measurement taken by the ultrasound sensors
int dist_us_sensor_central = 0;
int dist_us_sensor_left = 0;
int dist_us_sensor_right = 0;

int breaking_period = 0;

// Variable to check the theta for test
float theta_max = 0;

///////////////////////////////
//  Right encoder interrupt  //
///////////////////////////////
void cuentaDER() {
  tr = micros();
  // If a pulses arrives much faster than expected, filter it out!
  // We have disabled checking for a given latency. To enable it back again cooment out this line and uncomment the one before
  //if (tr-auxr>IGNORE_PULSE)
  if (1) {
    auxr = tr;
    encoderDER++;
  }
  else
    ignored_right++;
}  // End of cuentaDER()

//////////////////////////////
//  Left encoder interrupt  //
//////////////////////////////
void cuentaIZQ() {
  tl = micros();
  // If a pulses arrives much faster than expected, filter it out!
  // We have disabled checking for a given latency. To enable it back again cooment out this line and uncomment the one below
  //if (tl-auxl>IGNORE_PULSE)
  if (1) {
    auxl = tl;
    encoderIZQ++;
  }
  else
    ignored_left++;
}  // End of cuentaIZQ()

/////////////
//  SETUP  //
/////////////
void setup() {

  // Add the interrupt lines for encoders
  attachInterrupt(digitalPinToInterrupt(MOT_R_ENC_B_PIN), cuentaDER, FALLING);
  attachInterrupt(digitalPinToInterrupt(MOT_L_ENC_B_PIN), cuentaIZQ, FALLING);

  // Set all the motor control pins to outputs
  pinMode(MOT_R_PWM_PIN, OUTPUT);
  pinMode(MOT_L_PWM_PIN, OUTPUT);
  pinMode(MOT_R_A_PIN, OUTPUT);
  pinMode(MOT_R_B_PIN, OUTPUT);
  pinMode(MOT_L_A_PIN, OUTPUT);
  pinMode(MOT_L_B_PIN, OUTPUT);

  digitalWrite(MOT_R_A_PIN, LOW);
  digitalWrite(MOT_R_B_PIN, LOW);
  digitalWrite(MOT_L_A_PIN, LOW);
  digitalWrite(MOT_L_B_PIN, LOW);

  analogWrite(MOT_R_PWM_PIN, 0);
  analogWrite(MOT_L_PWM_PIN, 0);

  pinMode(MOT_R_PWM_PIN, OUTPUT);
  pinMode(MOT_L_PWM_PIN, OUTPUT);
  pinMode(MOT_R_A_PIN, OUTPUT);
  pinMode(MOT_R_B_PIN, OUTPUT);
  pinMode(MOT_L_A_PIN, OUTPUT);
  pinMode(MOT_L_B_PIN, OUTPUT);

  // Set encoder pins to inputs
  pinMode(MOT_L_ENC_B_PIN, INPUT);
  pinMode(MOT_R_ENC_B_PIN, INPUT);

  pinMode(L_US_TRIG,OUTPUT);
  pinMode(L_US_ECHO,INPUT);
  pinMode(R_US_TRIG,OUTPUT);
  pinMode(R_US_ECHO,INPUT);

  // Pin 9 will be the output of the ultarsound sensor's pulse
  pinMode(F_US_TRIG, OUTPUT);

  // Pin 8 will be an input, said input is proportional to the time it takes the wave reflection to reach the sensor
  pinMode(F_US_ECHO, INPUT);

  pinMode(LED, OUTPUT);

  // Initialize the seiral port at a speed of 38400 Bauds
  Serial.begin(38400);
  Serial.print("LOLA INI ");
  Serial.println(VERSION);
}  // End of setup()

///////////////////////////////////////////
//              MOVE MOTORS              //
//  dir_right (1: foward / 0: backwards) //
//  dir_left  (1: foward / 0: backwards) //
///////////////////////////////////////////
void move_motors() {
  // now turn off motors
  // Adaptation for L298n
  unsigned int inhib_r = 0xBB, inhib_l = 0xEE;

  encoderIZQ = encoderDER = ignored_left = ignored_right = aux_encoderIZQ = aux_encoderDER = encoder = encoder_ant = 0;

  //Deactivate both motor's H-bridge
  PORTA &= 0xAA;

  velr = SPEED_INI_R;
  vell = SPEED_INI_L;

  if (!velr)
    PORTB |= 0x10;

  else {
    inhib_r |= 0x44;
    analogWrite(MOT_R_PWM_PIN, velr);
  }

  if (!vell)
    PORTB |= 0x20;

  else {
    inhib_l |= 0x11;
    analogWrite(MOT_L_PWM_PIN, vell);
  }  

  if (dir_right && dir_left)
    PORTA |= 0x14 & inhib_r & inhib_l;

  else if (!dir_right && dir_left)
    PORTA |= 0x50 & inhib_r & inhib_l;

  else if (dir_right && !dir_left)
    PORTA |= 0x5 & inhib_r & inhib_l;
  
  else
    PORTA |= 0x11 & inhib_r & inhib_l;
}  // End of move_motors()


/////////////////////////////////////////////////
//         ULTRA SOUND RANGE DETECTOR          //
//  Return ->  -1:   Distance above 3m         //
//              0:   No return pulse detected  //
//            (int): Distance in cm            //
//                                             //
// Inputs -> TriggerPin                        //
//           EchoPin                           //
/////////////////////////////////////////////////
int us_range(int TriggerPin, int EchoPin) {
   long duration, distanceCm;

   digitalWrite(TriggerPin, LOW);   // Keep the line LOW for 4us for a clean flank
   delayMicroseconds(4);
   digitalWrite(TriggerPin, HIGH);  // Generate a high transition
   delayMicroseconds(10);
   digitalWrite(TriggerPin, LOW);   // After 10us, lower the line

   // Measure the duration in microseconds
   // Must be taken into account that this implies a delay of 10ms if no return is detected.
   duration = pulseIn(EchoPin, HIGH, 10000);

   // Check this number
   distanceCm = duration / 58.2;  // Convert the distance to cm

   if (distanceCm == 0)
    return 0;

   if (distanceCm < 170)
    return distanceCm;

  else
    return 175;
}  // End of us_range()

///////////////////
//  STOP_MOTORS  //
///////////////////
void stop_motors() {
  // Adaptation for L298n
  //We will fix the same duty cycle for the PWM outputs to make the braking spped equal!
  //analogWrite(MOT_R_PWM_PIN, 255);
  //analogWrite(MOT_L_PWM_PIN, 255);
  PORTB |= 0x3 << 4;
  PORTA &= 0xAA;
  delay(300);
}  // End of stop_motors()

//////////////////////////////////////////////
//           SPEED_NORMALIZATION            //
//  Speeds are normalized in order to work  //
//  at maximum speed given as SPEED_INI_X   //
//////////////////////////////////////////////
void speed_normalization() {
    if (velr > vell) {
      vell -= (velr - SPEED_INI_R);
      velr = SPEED_INI_R;
      if (vell < 0)
        vell = 0;
    }
    else {
      velr -= (vell - SPEED_INI_L);
      vell = SPEED_INI_L;
      if (velr < 0)
        velr = 0;
    }
} // End of speed_normalization()


///////////////////
//  STRAIGH_DIST //
///////////////////
void straigh_dist() {
  long encoder_long;
  unsigned int temp_encDER, temp_encIZQ;
  float s, sl, sr, aux_float;

  temp_encDER = encoderDER;
  temp_encIZQ = encoderIZQ;

  if (test_counter < 2000) {
      TEST_distances[test_counter] = dist_us_sensor_central;
      TEST_pulses[test_counter++ ] = (unsigned int) (0x00FF & temp_encDER);
    }

  // Encoder (below) is the difference of both encoders times the normalization constant accounting for the diameter's error
  aux_float = KKI * (float) temp_encIZQ - (float) temp_encDER;

  if (aux_float > 0)
    aux_float += 0.499999;
  if (aux_float < 0)
    aux_float -= 0.499999;
  encoder = (int) aux_float;

  error = encoder_ant - encoder;
  encoder_ant = encoder;
  // Implement PID (just PD)
  // Right wheel speed is updated if it is not braking at the end of the movement
  if (breaking_period == 0) {
    aux_float = (float) encoder * kp - (float) error * kd;

    if (aux_float > 0)
      aux_float += 0.5;
    if (aux_float < 0)
      aux_float -= 0.5;

  velr += (int) aux_float;

  speed_normalization();

  // Write, as PWM duty cycles, the speeds for each wheel
  analogWrite(MOT_R_PWM_PIN, velr);
  analogWrite(MOT_L_PWM_PIN, vell);
  delay(200);
  }
}  // End of straigh_dist()


///////////////////////////////////////////////////////////
//                  ONE_FASTER_DIST                      //
//  Control the PID algorithm when the right wheel must  //
//  be faster than the left one to perform a circular    //
//                     movement.                         //
///////////////////////////////////////////////////////////
void one_faster_dist() {
  int interval;
  unsigned int temp_encDER, temp_encIZQ;

  update_global_positions();

  temp_encDER = encoderDER;
  temp_encIZQ = encoderIZQ;

  if (STATE == CIRC_TEST_R_STATE) {
    if (test_counter < 2000) {
      TEST_distances[test_counter] = dist_us_sensor_central;
      TEST_pulses[test_counter++ ] = (unsigned int) (0x00FF & temp_encDER);
    }
  }
  else
    if (STATE == CIRC_TEST_L_STATE) {
      if (test_counter < 2000) {
        TEST_distances[test_counter] = dist_us_sensor_central;
        TEST_pulses[test_counter++ ] = (unsigned int) (0x00FF & temp_encIZQ);
      }
    }

  if (radii_relation)
    if (STATE == RIGHT_FASTER_STATE)
      encoder=(int) (KKI * (float) temp_encIZQ - (float) temp_encDER * radii_relation + 0.499999);
    else
      encoder=(int) ((float) temp_encDER - (float) KKI * (float) temp_encIZQ * radii_relation + 0.499999);
  else
    encoder = 0;

  error = encoder_ant - encoder;
  encoder_ant = encoder;

  // Implement PID (just PD)
  // Right wheel speed is updated if it is not braking at the end of the movement
  if (breaking_period == 0) {
    velr += (encoder * kp - error * kd);

    // Speeds are normalized in order to work at maximum speed
    speed_normalization();
  }

  // Write, as PWM duty cycles, the speeds for each wheel
  if (velr == 0) {
    digitalWrite(MOT_R_A_PIN, LOW);
    digitalWrite(MOT_R_B_PIN, LOW);
    analogWrite(MOT_R_PWM_PIN, 0);
  }
  else {
    if (dir_right == 2) {
    digitalWrite(MOT_R_A_PIN, HIGH);
    digitalWrite(MOT_R_B_PIN, LOW);
    analogWrite(MOT_R_PWM_PIN, velr);
    }
    else {
    digitalWrite(MOT_R_A_PIN, LOW);
    digitalWrite(MOT_R_B_PIN, HIGH);
    analogWrite(MOT_R_PWM_PIN, velr);
    }
  }
  if (vell == 0) {
    digitalWrite(MOT_L_A_PIN, LOW);
    digitalWrite(MOT_L_B_PIN, LOW);
    analogWrite(MOT_L_PWM_PIN, 255);
  }
  else {
    if (dir_left == 2) {
    digitalWrite(MOT_L_A_PIN, LOW);
    digitalWrite(MOT_L_B_PIN, HIGH);
    analogWrite(MOT_L_PWM_PIN, vell);
    }
    else {
    digitalWrite(MOT_L_A_PIN, HIGH);
    digitalWrite(MOT_L_B_PIN, LOW);
    analogWrite(MOT_L_PWM_PIN, vell);
    }
  }
}  // End of one_faster_dist()


///////////////////////
//  DISP_GLOBAL_POS  //
///////////////////////
void disp_global_pos() {
        Serial.println(" ");
        Serial.print(" X: ");
        Serial.print((int) X);
        Serial.print(" Y: ");
        Serial.print((int) Y);
        Serial.print(" Theta: (rad)");
        Serial.print(Theta);
        Serial.print(" Theta: (degrees)");
        Serial.println(Theta * 57.29);
}  // End of disp_global_pos()


/////////////////////////////////////////////////////////////////////
//                  UPDATE GLOBAL POSITIONS                        //
//  Update the X position,Y position and orientation of the robot  //
//      using the enoderIZQ and encoderDER received pulses.        //
/////////////////////////////////////////////////////////////////////
void update_global_positions() {
  float s, sl, sr;
  unsigned int temp_encDER, temp_encIZQ;

  temp_encDER = encoderDER;
  temp_encIZQ = encoderIZQ;
  sl = mmperpulse * KKI * (temp_encIZQ - aux_encoderIZQ);
  sr = mmperpulse * (temp_encDER - aux_encoderDER);
  aux_encoderDER = temp_encDER;
  aux_encoderIZQ = temp_encIZQ;

  if (dir_right == 0)
    sr = -sr;
  if (dir_left == 0)
    sl = -sl;
  Theta += (sr - sl) / WHEEL_DIST;
  s = (sr + sl) / 2;

  X += s * cos(Theta);
  Y += s * sin(Theta);
}  // End of update_global_positions()

//////////////////////////////////////////
//                  DEP                 //  
// This function is used for debugging  //
//////////////////////////////////////////
void dep() {
  Serial.print(" VR: ");
  Serial.print(velr);
  Serial.print(" VL: ");
  Serial.print(vell);

  if (encoder > 0) {
    Serial.print("  +");
    Serial.print(encoder);
  }
  else {
    Serial.print("  -");
    Serial.print(-encoder);
  }

  if (error > 0) {
    Serial.print("    Error: +");
    Serial.print(error);
  }
  else {
    Serial.print("    Error: -");
    Serial.print(-error);
  }

  Serial.print(" encoderDER: ");
  Serial.print(encoderDER);
  Serial.print(" encoderIZQ: ");
  Serial.print(encoderIZQ);
  Serial.print(" Igr_IZQ: ");
  Serial.print(ignored_left);
  Serial.print(" Igr_DER: ");
  Serial.println(ignored_right);
}  // End of dep()

///////////////////////////////////////////////////////////////
//                      READ_NUMBER                          //
//  Read a number from the serial port with <number> digits  //
///////////////////////////////////////////////////////////////
short int read_number(int n_digits) {
  char speed[5];

  // Wait to be sure the bytes have arrived
  delay(5);

  if (Serial.available() > n_digits- 1) {
    Serial.readBytes(speed, n_digits);
    speed[n_digits] = '\0';  // Append a NULL character to terminate the string! Not needed if initialized with char speed[5] = {0}...
    return (unsigned int) atoi(speed);
  }
  else {
    ERROR_CODE = NO_NUMBER;
    return 0;
  }
}  // End of read_number()

////////////////////////////////////////////////////////////////////////////
//                            PARSE_INPUT                                 //
// Read data from the serial port in order to update operation constants  //
////////////////////////////////////////////////////////////////////////////
void parse_input(void) {
  unsigned char incoming_byte = '\0', incoming_number[N_DIGS] = {'\0'};
  int i = 0;
  while (Serial.available() > 0) {
    incoming_byte = Serial.read();
    if (incoming_byte <= '9' || incoming_byte == '.')
      incoming_number[i++] = incoming_byte;
    else {
      if (i >= 0)
        update_param(incoming_number, incoming_byte);
      i = 0;
      for (int k = 0; k < N_DIGS; k++)
        incoming_number[k] = '\0';
    }
  }
  return;
}  // End of parse_input()

//////////////////////////////////////////////////////
//                    UPDATE_PARAM                  //
// Update operation constants based on parsed data  //
//////////////////////////////////////////////////////
void update_param(char* number, char parameter) {
  switch (parameter) {
    case 'W': //Adjust the Wheelbase
      WHEEL_DIST = atof(number);
      Serial.print("The parameter we are updating is: ");
      Serial.print("Wheelbase");
      Serial.print(" with a value of: ");
      Serial.println(WHEEL_DIST);
      break;
    case 'M': //Adjust the MM to pulses factor
      mmperpulse = atof(number);
      Serial.print("The parameter we are updating is: ");
      Serial.print("MM per pulse");
      Serial.print(" with a value of: ");
      Serial.println(mmperpulse);
      break;
    case 'D': //Adjust the diameter's error
      KKI = atof(number);
      Serial.print("The parameter we are updating is: ");
      Serial.print("KKI");
      Serial.print(" with a value of: ");
      Serial.println(KKI);
      break;
    case 'R': //Adjust the number of turns during calibration
      c_factor = atoi(number);
      Serial.print("The parameter we are updating is: ");
      Serial.print("Correction factor");
      Serial.print(" with a value of: ");
      Serial.println(c_factor);
      break;
    case 'S':
      Serial.println("Paramenter values: ");
      Serial.print("\tWheelbase: ");
      Serial.println(WHEEL_DIST, 10);
      Serial.print("\tMM per pulse: ");
      Serial.println(mmperpulse, 10);
      Serial.print("\tKKI: ");
      Serial.println(KKI, 10);
      Serial.print("\tC Factor: ");
      Serial.println(c_factor);
      break;
    default:
      Serial.println("ERROR!");
  }
  return;
}  // End of update_param()

////////////////////////////////////////////////////////////////////////
//                            ANALYZE_ORDER                           //
// Parse command information and trigger all the necessary functions  //
////////////////////////////////////////////////////////////////////////
void analyze_order() {
  char str[35];
  float s,sl,sr;
  int num, n;

  switch (order[0]) {
    // '0'  rotate unclockwise with different speeds
    //      Right wheel is faster
    // '1'  rotate clockwise with different speeds
    //      Left Wheel is faster
    // 'C'  Test circular right
    // 'D'  Test circular left
    //      make a circular move for at least 10 rounds and in the loop the keep the
    //      from central sensor to the object in from and the number of pulses
    case   0x30:  // '0'
    case   0x31:  // '1'
    case   0x43:  // 'C'
    case   0x44:  // 'D'
    case   0x4A:  // 'J'
    case   0x4B:  // 'K'
       // The test is done with a lower speed to reduce the error
       if (order[0] == 0x43 || order[0] == 0x44) {
         test_counter = 0;
         SPEED_INI_R = 220;
         SPEED_INI_L = 220;
       }
       else
         SPEED_INI_R = SPEED_INI_L = 255;

      // The command's syntax is "0XXXYYYY", where XXX is a 3 byte string that indicates the radii ratio * 999
      num = read_number(3);
      Serial.print(" ");
      Serial.println(num);
      if (num < 0 || num > 999) {
        ERROR_CODE = RR_OUT_RANGE;
        break;
      }
      // For a circular test we must ensure that num == 0 to avoid the error, so the code for test is: C000, D000
      if (order[0] == 0x43 || order[0] == 0x44)
        if (num != 0) {
          ERROR_CODE = RR_OUT_RANGE;
          break;
        }
      radii_relation = (float) num / 999;

      /* The command's syntax is "0XXXYYYY" where YYYY is a 4 bytes string indicating the distance that the faster wheel should
         traverse. For this circular test the distance is given in cm (* 10 factor), as we will be traersing large distances to
         reduce the non-systematic and estimation errors when computing the number of pulses per turn */
      num = read_number(4);
      if (num < 0 || num > 9999) {
        ERROR_CODE = OUT_RANGE;
        break;
      }

      if (order[0] == 0x43 || order[0] == 0x44)
        PULSES_NUM = (unsigned int) ( 10 * num / mmperpulse);
      else
        PULSES_NUM = (unsigned int) (num / mmperpulse);

        // The is the minimum space to move
        if (PULSES_NUM > INERTIA_LIMIT) {
          switch (order[0]) {
            case '0':
              STATE = RIGHT_FASTER_STATE;
              dir_right = dir_left = 1;
              SPEED_INI_L=(int) (SPEED_INI_R * radii_relation * 0.8);
              move_motors();
              break;

            case '1':
              STATE = LEFT_FASTER_STATE;
              dir_right = dir_left = 1;
              SPEED_INI_R = (int) (SPEED_INI_L * radii_relation * 0.8);
              move_motors();
              break;

            case 'C':
              STATE = CIRC_TEST_R_STATE;
              dir_right = dir_left = 1;
              SPEED_INI_L = (int) (SPEED_INI_R * radii_relation * 0.8);
              move_motors();
              break;

            case 'D':
              STATE = CIRC_TEST_L_STATE;
              dir_right = dir_left = 1;
              SPEED_INI_R = (int) (SPEED_INI_L * radii_relation * 0.8);
              move_motors();
              break;

            case 'J':
              STATE = RIGHT_FASTER_STATE;
              dir_right = 2;
              dir_left = 0;
              SPEED_INI_L = (int) (SPEED_INI_R * radii_relation * 0.8);
              move_motors();
              break;

            case 'K':
              STATE = LEFT_FASTER_STATE;
              dir_right = 0;
              dir_left = 2;
              SPEED_INI_R = (int) (SPEED_INI_L * radii_relation * 0.8);
              move_motors();
          }
        }
        else
          ERROR_CODE = OUT_RANGE;
      break;

    // '2' -> Rotate counterclockwise with respect to the wheel's axis center
    // '3' -> Rotate clockwise with respect to the wheel's axis center
    case   0x32: // '2'
    case   0x33: // '3'
      SPEED_INI_R = SPEED_INI_L = 200;

      test_counter = 0;

      radii_relation = 1;
      // The command's syntax is "3XXX", where XXX is a 3 byte string indicating the rotation in degrees
      num = read_number(3);

      if (num && num < 361) {
        PULSES_NUM = c_factor * ((num * 3.1416 * WHEEL_DIST) / (360 * mmperpulse)); 
        if (order[0] == '2') {
          clockwise = dir_left = 0;
          STATE = ROTATE_CCW_STATE;
          dir_right = 1;
          move_motors();
        }
        else {
          clockwise = dir_left = 1;
          STATE = ROTATE_CW_STATE;
          dir_right = 0;
          move_motors();
        }
      }
      else
        ERROR_CODE = OUT_RANGE;
      break;
      /* ## Move forward ##
        Command: "4XXXX", where XXXX is a 4 byte string representing the distance in mm.
       ## Move backward ##
        Command: "5XXXX", where XXXX is a 4 byte string representing the distance in mm.
       We recomend to traverse small distances in backwards movement due to the lack of an US sensor at the rear...
       ## Move forward at a specific speed ##
        Command: "6XXXXYYY", where XXXX is a 4 byte string representing the distance in mm and YYY is the maximum speed (PWM Duty Cycle)
       ## Move backward at specific speed ##
        Command: "7XXXXYYY", where XXXX is a 4 byte string representing the distance in mm and YYY is the maximum speed (PWM Duty Cycle)
       We recomend to traverse small distances in backwards movement due to the lack of an US sensor at the rear... */
    case   0x34: // '4'
    case   0x35: // '5'
    case   0x36: // '6'
    case   0x37: // '7'
      SPEED_INI_R = SPEED_INI_L = 255;
      radii_relation = 1;
      num = read_number(4);
      if (num) {
        PULSES_NUM = num / mmperpulse;
        // The is the minimum space to move
        if (PULSES_NUM > INERTIA_LIMIT) {
          if (order[0] == 0x36 || order[0] == 0x37) {
            num = read_number(3);
            if (num && num < 256)
              SPEED_INI_R = SPEED_INI_L = num;
            else
              ERROR_CODE = SPEED_OUT_RANGE;
          }
          if (ERROR_CODE == NO_ERROR) {
            STATE = MOVE_STRAIGHT_STATE;
            if (order[0] == 0x34 || order[0] == 0x36) 
              dir_right = dir_left = 1;
            else
              dir_right = dir_left = 0;
            move_motors();
          }
        }
        else
          ERROR_CODE = INERTIA_LIMIT_ERROR;
      }
      else
        ERROR_CODE = OUT_RANGE;
      break;

    // Bumping and falling Sensors status
    case 0x39: // '9'
      for (int k = 0; k < 6; k++)
        str[k] = '1';
      str[6] = '\0';
      Serial.print("Sensors :");
      Serial.println(str);
      break;

    // Send back the last measurements from the US sensors
    case 0x3A: // ':'
      sprintf(str,"%03d %03d %03d", dist_us_sensor_central,
                                    dist_us_sensor_left,
                                    dist_us_sensor_right);
      Serial.print("US Sensors :");
      Serial.println(str);
      break;

    // Reset the global position
    case 0x3C: // '<'
      X = Y = Theta = 0;
      Serial.println(0x15);
      break;

    // Send back the firmware's version
    case 0x3E: // '>'
      Serial.println(VERSION);
      break;

    // Stop motors and calcucompute the new positions
    case 0x3F: // '?'
      stop_motors();
      STATE = REST_STATE;
      sl = mmperpulse * (encoderIZQ - aux_encoderIZQ);
      sr = mmperpulse * (encoderDER - aux_encoderDER);
      if (dir_right == 0)
        sr = -sr;
      if (dir_left == 0)
        sl =- sl;
      Theta += (sr - sl) / WHEEL_DIST;
      s = (sr + sl) / 2;

      X += s * cos(Theta);
      Y += s * sin(Theta);
      dep();
      break;

    // Send back the global position variables. The position is in mm and the orientation in degrees
    case 0x41: // 'A'
      while(Theta > 6.2832)
        Theta -= 6.28318531;
      while(Theta < -6.2832)
        Theta += 6.28318531;
      disp_global_pos();
      Serial.print(" encoderDER ");
      Serial.print(encoderDER);
      Serial.print(" encoderIZQ ");
      Serial.println(encoderIZQ);
      break;
    // Send back the state of the system. STATE is the variable and the different states are #define(d) at the beginning
    // The state is sent by adding 0x30
    case 0x42: // 'B'
      Serial.print("B");
      Serial.write(0x30 + STATE);
      Serial.println("");
      break;

    // The error is sent back by adding 0x30 to the error code
    case 0x45: // 'E'
      Serial.write(0x45);  // 'E'
      Serial.write(0x30 + ERROR_CODE);
      Serial.println("");
      break;

    case 0x46: // 'F'
      Serial.print("Counter :");
      Serial.println(test_counter);
      for (n = 1; n < test_counter; n++) {
        Serial.print(" ");
        Serial.print(TEST_pulses[n]);
        Serial.print(" ");
        Serial.print(TEST_distances[n]);
        Serial.println(" ;... ");
        delay(50);
      }
      Serial.println("END");
      break;
    case 0x53: // 'S'
      parse_input();
      break;
    default:
      ERROR_CODE = NO_AVAILABLE;
  }
}  // End of analyze_order()

////////////
//  LOOP  //
////////////
void loop() {
 static unsigned char us_sensor = 0;
 unsigned char aux, first_time = 1, flag = 1, dbg_counter = 0;
 static unsigned long time = 0, time1 = 0;
 unsigned long reference, last_pulses = 0, current_pulses;

 while(1) {
    // Sequence to read the ultra-sound sensors. In each iteration one sensor is read. We must take into account that we get a
    // delay by reading the ultra-sound sensor...
  if (us_sensor == 0)
  dist_us_sensor_central = us_range(F_US_TRIG, F_US_ECHO);

  else if (us_sensor == 2)
    dist_us_sensor_left = us_range(L_US_TRIG, L_US_ECHO);

  else if (us_sensor == 4)
    dist_us_sensor_right = us_range(R_US_TRIG, R_US_ECHO);

  if (++us_sensor == 5)
    us_sensor = 0;

  //  Read from the serial port if there is anything available
  if (Serial.available() > 0) {
    order[0] = 'Z';
    theta_max = 0;
    Serial.readBytes(order, 1);
    // Every time a command is going to be reveived the error is reset if it's not asking for an error code
    if (order[0] != 'E')
        ERROR_CODE = NO_ERROR;

    analyze_order();

    // Clean the buffer
    while(Serial.available())
      Serial.readBytes(&aux, 1);
  }

  // Each state performs a different movement
  switch (STATE) {
    case   REST_STATE:
      first_time = 1;
      breaking_period = 0;
      if((millis() - time1) > 5000) {
        Serial.print(".");
        time1 = millis();
      }
      // Just in case an error produced the movement of the motors
      stop_motors();
      break;

    // Movement where the right wheel should rotate faster. The right wheel will be the reference
    case RIGHT_FASTER_STATE:
    case LEFT_FASTER_STATE:
    case CIRC_TEST_R_STATE:
    case CIRC_TEST_L_STATE:
      // Take the reference of the faster wheel to compare distances
      if (STATE == RIGHT_FASTER_STATE || STATE == CIRC_TEST_R_STATE)
        reference = encoderDER;
      else
        reference = encoderIZQ;
      // Stop just a few pulses before because of the inertia
      if(reference < PULSES_NUM - INERTIA_LIMIT) {
        // Reduce speed in order to get objective pulses
        if (reference > PULSES_NUM - BREAK_PULSES) {
          breaking_period = 1;
          if (vell > 125)
            SPEED_INI_L = vell = 125;

          if (velr > 125)
            SPEED_INI_R = velr = 125;

          if (first_time == 1) {
            first_time = 0;
           // Write the speeds as a PWM duty cycle for each wheel
           analogWrite(MOT_R_PWM_PIN, velr);
           analogWrite(MOT_L_PWM_PIN, vell);
          }
        }
        if(millis() - time > PID_TIME) {
          time = millis();
          one_faster_dist();
          dbg_counter++;
          if (dbg_counter > 4) {
            dep();
            dbg_counter = 0;
          }
        }
      }
      else {
        stop_motors();
        // Make a delay in order to be sure that the wheels have stopped
        delay(1000);
        dep();
        update_global_positions();
        STATE = REST_STATE;
      }
    break;

    // Movement where both wheels should rotate the same distance
    case ROTATE_CCW_STATE:
    case ROTATE_CW_STATE:
    case MOVE_STRAIGHT_STATE:
      if(encoderDER < PULSES_NUM - INERTIA_LIMIT) {
        // Reduce speed in order to get objective pulses
        if (encoderDER > PULSES_NUM - BREAK_PULSES) {
          breaking_period = 1;
          if (vell > 125)
            SPEED_INI_L = vell = 125;

          if (velr>125)
            SPEED_INI_R = velr = 125;

          if (first_time == 1) {
            first_time=0;
            // Write the speeds as a PWM duty cycle for each wheel
            analogWrite(MOT_R_PWM_PIN, velr);
            if (vell == 0) {
              digitalWrite(MOT_L_A_PIN, LOW);
              digitalWrite(MOT_L_B_PIN, LOW);
              analogWrite(MOT_L_PWM_PIN, 255);
            }
            else {
              digitalWrite(MOT_L_A_PIN, HIGH);
              digitalWrite(MOT_L_B_PIN, LOW);
              analogWrite(MOT_L_PWM_PIN, vell);
            }
          }
        }
          update_global_positions();

        if(current_pulses = (unsigned int) (((encoderDER + encoderIZQ) / 2) + 0.4999) - last_pulses > PID_PULSES) {
          last_pulses = current_pulses;
          straigh_dist();
          // delay(250);
          if (theta_max < abs(Theta))
            theta_max = abs(Theta);
          dep();
        }
      }
      else {
        stop_motors();
        dep();
        // Make a delay in order to be sure that the wheels have stopped
        delay(1000);
        STATE = REST_STATE;
        update_global_positions();
      }
      break;
      default:
      break;
  }
 } // End of while(1)
} // End of loop
