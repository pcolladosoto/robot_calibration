// You can find more information at:
// www.
// Please include reference
// ................................

// Date 3/07/19

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
#define NO_NUMBER             1 // Waiting for a number, time-out
#define OUT_RANGE             2 // Received number out of range
#define SPEED_OUT_RANGE       3 // Received speed out of range
#define RR_OUT_RANGE          4 // Radio Ratio out of range
#define NO_AVAILABLE          5 // Received Command non available
#define INERTIA_LIMIT_ERROR   6 // Distance lower than inertia limit

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
  #define INERTIA_LIMIT  40     // Inertia limit to stop few pulses before the limit
  #define TIME_PID       200    // Time in miliseconds for each iteration of the PID
  #define BREAK_PULSES   150    // Number of pulses from the end to start breaking
  #define IGNORE_PULSE   11000  // time in micros to ignore encoder pulses if faster

  // PID (PD) constants
  //float kp = 0.1;
  //float kd = 0.4;
  float kp = 0.5; // 0.23*2;
  float kd = 2; // 10.0*2;
#else
  float KKI = 1;               // Deviation from theoretical left wheel diameter; the rigth wheel is the reference; >1 wheel bigger than the nominal one

  //Calibration correction factor for turning
  int c_factor = 1;

  //Current_pulses and last_pulses for PID calibration
  float WHEEL_DIST = 535;           //UAH 578; //UMB          //575;      // Wheel distance in mm
  float mmperpulse = 1.68;          // mm per pulse in the encoders
  #define INERTIA_LIMIT  1        // Inertia limit to stop few pulses before the limit
  #define TIME_PID        500     // Time in miliseconds for each iteration of the PID
  #define BREAK_PULSES   1        // Number of pulses from the end to start breaking
  #define IGNORE_PULSE   11000    // time in micros to ignore encoder pulses if faster
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
unsigned int counter_test = 0;

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
int velr = SPEED_INI_R;
int vell = SPEED_INI_L;
int error = 0;
int encoder_ant;
unsigned int PULSES_NUM;

// FSM's STATES
unsigned char ESTADO = 0;
#define EST_REPOSO        0
#define EST_ROTATE_UNCLK  2
#define EST_ROTATE_CLK    3
#define EST_MOV_STRAITGH  4
#define EST_RIGHT_FASTER  5
#define EST_LEFT_FASTER   6
#define EST_TEST_CIRC_R   7
#define EST_TEST_CIRC_L   8

unsigned char orden[1];

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

  velr=SPEED_INI_R;
  vell=SPEED_INI_L;

  if (!velr)
    PORTB |= 0x10;

  else {
    inhib_r |= 0x44;
    analogWrite(MOT_R_PWM_PIN, velr)
  }

  if (!vell)
    PORTB |= 0x20;

  else {
    inhib_l |= 0x11;
    analogWrite(MOT_L_PWM_PIN, vell)
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
   distanceCm = duration/58.2;  // Convert the distance to cm

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

  if (counter_test < 2000) {
      TEST_distances[counter_test] = dist_us_sensor_central;
      TEST_pulses[counter_test++ ] = (unsigned int) (0x00FF & temp_encDER);
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

  if (ESTADO == EST_TEST_CIRC_R) {
    if (counter_test < 2000) {
      TEST_distances[counter_test] = dist_us_sensor_central;
      TEST_pulses[counter_test++ ] = (unsigned int) (0x00FF & temp_encDER);
    }
  }
  else
    if (ESTADO == EST_TEST_CIRC_L) {
      if (counter_test < 2000) {
        TEST_distances[counter_test] = dist_us_sensor_central;
        TEST_pulses[counter_test++ ] = (unsigned int) (0x00FF & temp_encIZQ);
      }
    }

  if (radii_relation)
    if (ESTADO == EST_RIGHT_FASTER)
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
short int read_number(int number) {
  char speed[5];

  // Wait to be sure the bytes have arrived
  delay(5);

  if (Serial.available() > number- 1) {
    Serial.readBytes(speed, n_digits);
    speed[n_digits] = '\0';  // Append a NULL character to terminate the string! Not needed if initialized with char speed[5] = {0}...
    return (unsigned int) atoi(speed);
  }
  else {
    ERROR_CODE = NO_NUMBER;
    return 0;
  }
}  // End of read_number()


void parse_input(void) {
  unsigned char incoming_byte = '\0', incoming_number[N_DIGS] = {'\0'};
  int i = 0;
  while (Serial.available() > 0) {
    incoming_byte = Serial.read();
    if (incoming_byte <= '9' || incoming_byte == '.')
      incoming_number[i++] = incoming_byte;
    else {
      if (i > 0)
        update_param(incoming_number, incoming_byte);
      i = 0;
      for (int k = 0; k < N_DIGS; k++)
        incoming_number[k] = '\0';
    }
  }
  return;
}  // End of parse_input()

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
    default:
      Serial.println("ERROR!");
  }
  return;
}  // End of update_param()

void analizar_orden()
{
  float s,sl,sr;
  int num;
  char cadena[35];
  int n;


  switch (orden[0])
  {
    // '0'  rotate unclockwise with different speeds
    //      Right wheel is faster
    // '1'  rotate clockwise with different speeds
    //      Left Wheel is faster
    // 'C'  Test circular right
    // 'D'  Test circular left
    //      make a circular move for at least 10 rounds and in the loop the keep the
    //      from central sensor to the object in from and the number of pulses
    case   0x30:  //'0'
    case   0x31:  //'1'
    case   0x43:  //'C'
    case   0x44:  //'D'
    case   0x4A:  //'J'
    case   0x4B:  //'K'
       // The test is done with lower speed to reduce error
       if (orden[0]==0x43 || orden[0]==0x44)
       {
         counter_test=0;
         SPEED_INI_R=220;
         SPEED_INI_L=220;
       }
       else
       {
         SPEED_INI_R=255;
         SPEED_INI_L=255;
       }

      // Command is "0XXX YYYY" XXX is a 3 bytes number indicating the radios ratio *999
      num=read_number(3);
      Serial.print(" ");
      Serial.println(num);
      if (num<0 || num>999)
      {
        ERROR_CODE=RR_OUT_RANGE;
        break;
      }
      // Is mandatory num==0 for circular test just to avoid error, so the code for test is:
      // C000
      // D000
      if (orden[0]==0x43 || orden[0]==0x44)
        if (num!=0)
        {
          ERROR_CODE=RR_OUT_RANGE;
          break;
        }
      radii_relation=(float)num/999;

      // Command is "0XXX YYYY" YYYY is a 4 bytes number indicating the distance
      // that should move the faster wheel
      // for the circular test the distance is given in cm (*10) because we are
      // looking for long distances to reduce non-systematic error and the estimation
      // error of the number of pulses per turn.
      num=read_number(4);
      if (num<0 || num>9999)
      {
        ERROR_CODE=OUT_RANGE;
        break;
      }

      if (orden[0]==0x43 || orden[0]==0x44)
        PULSES_NUM=(unsigned int) ((float)10*(float)num/mmperpulse);
      else
        PULSES_NUM=(unsigned int) ((float)((float)num/mmperpulse));


        // The is a minimum space to move
        if (PULSES_NUM>INERTIA_LIMIT)
        {
          if (orden[0]=='0')
          {
            ESTADO=EST_RIGHT_FASTER;
            dir_right=1;
            dir_left=1;
            SPEED_INI_L=(int)((float)SPEED_INI_R*radii_relation*0.8);
            move_motors();
          }
          else
            if (orden[0]=='1')
            {
              ESTADO=EST_LEFT_FASTER;
              dir_right=1;
              dir_left=1;
              SPEED_INI_R=(int)((float)SPEED_INI_L*radii_relation*0.8);
              move_motors();
            }
            else
              if (orden[0]=='C')
              {
                ESTADO=EST_TEST_CIRC_R;
                dir_right=1;
                dir_left=1;
                SPEED_INI_L=(int)((float)SPEED_INI_R*radii_relation*0.8);
                move_motors();
              }
              else
                if (orden[0]=='D')
                {
                  ESTADO=EST_TEST_CIRC_L;
                  dir_right=1;
                  dir_left=1;
                  SPEED_INI_R=(int)((float)SPEED_INI_L*radii_relation*0.8);
                  move_motors();
                }
                else
                  if (orden[0]=='J')
                  {
                    ESTADO=EST_RIGHT_FASTER;
                    dir_right=2;
                    dir_left=0;
                    SPEED_INI_L=(int)((float)SPEED_INI_R*radii_relation*0.8);
                    move_motors();
                  }
                  else
                    if (orden[0]=='K')
                    {
                      ESTADO=EST_LEFT_FASTER;
                      dir_right=0;
                      dir_left=2;
                      SPEED_INI_R=(int)((float)SPEED_INI_L*radii_relation*0.8);
                      move_motors();
                    }
        }
        else
          ERROR_CODE=OUT_RANGE;
      break;

    // '2'  rotate unclockwise respect to the wheel axis center
    // '3'  rotate clockwise respect to the wheel axis center
    case   0x32: //'2'
    case   0x33: //'3'
      SPEED_INI_R=200;
      SPEED_INI_L=200;

      counter_test=0;

      radii_relation=1;
      // Command is "3XXX" XXX is a 3 bytes number indicating the rotation in degrees.
      num=read_number(3);

//      Serial.print("  NUM: ");
//      Serial.println(num);
      if (num!=0 && num<361)
      {
        PULSES_NUM= c_factor * ((float)num*3.1416*(float)WHEEL_DIST)/((float)360*(float)mmperpulse);
//        Serial.print("PULSES: ");
//        Serial.print(PULSES_NUM);
        if (orden[0]=='2')
        {
          clockwise=0;
          ESTADO=EST_ROTATE_UNCLK;
          dir_right=1;
          dir_left=0;
          move_motors();
        }
        else
        {
          clockwise=1;
          ESTADO=EST_ROTATE_CLK;
          dir_right=0;
          dir_left=1;
          move_motors();
        }
      }  // if (num!=0 ...
      else
      {
        ERROR_CODE=OUT_RANGE;
      }
      break;
      // Move forward
      // Command: "4XXXX", where XXXX is a 4 bytes distance in mm.
      // Move backward
      // Command: "5XXXX", where XXXX is a 4 bytes distance in mm.
      // We recomend small distance in backwards movement due
      // to the non sensor information in that situation.
      // Move forward at specific speed
      // Command: "6XXXX YYY", where XXXX is a 4 bytes distance in mm.
      //          and YYY the maximum speed (pwm)
      // Move backward at specific speed
      // Command: "7XXXX YYY", where XXXX is a 4 bytes distance in mm.
      //          and YYY the maximum speed (pwm)
      // We recomend small distance in backwards movement due
      // to the non sensor information in that situation.
    case   0x34: //'4'
    case   0x35: //'5'
    case   0x36: //'6'
    case   0x37: //'7'
      SPEED_INI_R=255;
      SPEED_INI_L=255;
      radii_relation=1;
      num=read_number(4);
//      Serial.print("  NUM: ");
//      Serial.println(num);
      if (num!=0)
      {
        PULSES_NUM=num/mmperpulse;
        Serial.print("PULSES_NUM: ");
        Serial.println(PULSES_NUM);
        // The is a minimum space to move
        if (PULSES_NUM>INERTIA_LIMIT)
        {
          if (orden[0]==0x36 || orden[0]==0x37)
          {
            num=read_number(3);
            if (num!=0 && num<256)
            {
              SPEED_INI_R=num;
              SPEED_INI_L=num;
            }
            else
            {
              ERROR_CODE=SPEED_OUT_RANGE;
            }
          }
          if (ERROR_CODE==NO_ERROR)
          {
            ESTADO=EST_MOV_STRAITGH;
            if (orden[0]==0x34 || orden[0]==0x36)
            {
              dir_right=1;
              dir_left=1;
            }
            else
            {
              dir_right=0;
              dir_left=0;
            }
            move_motors();
          }
        }
        else
          ERROR_CODE=INERTIA_LIMIT_ERROR;
      }
      else
      {
        ERROR_CODE=OUT_RANGE;
      }
      break;

    // Bumping and falling Sensors state
    case 0x39: //'9'
      cadena[0]='1';
      cadena[1]='1';
      cadena[2]='1';
      cadena[3]='1';
      cadena[4]='1';
      cadena[5]='1';
      cadena[6]=0;
      Serial.print("Sensors :");
      Serial.println(cadena);
      break;
    // Send back the last measurements from US sensors
    case 0x3A: //':'
      sprintf(cadena,"%03d %03d %03d", dist_us_sensor_central,
                                       dist_us_sensor_left,
                                       dist_us_sensor_right);

      Serial.print("US Sensors :");
      Serial.println(cadena);
      break;

    // Reset global position
    case 0x3C: //'<'
      X=0;
      Y=0;
      Theta=0;
      Serial.println(0x15);
      break;

    // Send back version
    case 0x3E: //'>'
      Serial.println(VERSION);
      break;

    // Stop motors and calculate new positions
    case 0x3F: // '?' (0x3F)
      stop_motors();
      ESTADO=EST_REPOSO;
      sl=mmperpulse*(encoderIZQ-aux_encoderIZQ);
      sr=mmperpulse*(encoderDER-aux_encoderDER);
      if (dir_right==0)
        sr=-sr;
      if (dir_left==0)
        sl=-sl;
      Theta+=(sr-sl)/WHEEL_DIST;
      s=(sr+sl)/2;

      X=X+s*cos(Theta);
      Y=Y+s*sin(Theta);
      dep();
      break;

    // Send back the global position variables
    // Position X, X (in mm)
    // Orientation (in degree)
    case 0x41: //'A'
      while(Theta>6.2832)
        Theta=Theta-6.28318531;
      while(Theta<-6.2832)
        Theta=Theta+6.28318531;
      disp_global_pos();
      Serial.print(" encoderDER ");
      Serial.print(encoderDER);
      Serial.print(" encoderIZQ ");
      Serial.println(encoderIZQ);
      //sprintf(cadena,"X%05d Y%05d T%03d",(int)X,(int)Y,(int)((float)Theta*(float)57.29578));
      //Serial.println(cadena);
      break;
    // Send back the state of the system
    // ESTADO is the variable and the different states are in #define at the beggining
    // The state is sent by adding 0x30
    case 0x42: //'B'
      Serial.print("B");
      Serial.write(0x30+ESTADO);
      Serial.println("");
      break;

//    Programed in the case 0x30 and 0x31
//    case 0x43:  //'C'
//    case 0x44:  //'D'

    // The error is sent back by adding 0x30 to the error code
    case 0x45: //'E'
      Serial.write(0x45);  // 'E'
      Serial.write(0x30+ERROR_CODE);
      Serial.println("");
      break;

    case 0x46: //'F'
      Serial.print("Counter :");
      Serial.println(counter_test);
      for (n=1;n<counter_test;n++)
      {
        Serial.print(" ");
        Serial.print(TEST_pulses[n]);
        Serial.print(" ");
        Serial.print(TEST_distances[n]);
        Serial.println(" ;... ");
        delay(50);
      }
      Serial.println("END");
      break;
    case 0x53: //'S'
      parse_input();
      break;
    default:
      ERROR_CODE=NO_AVAILABLE;
    break;
  }
}  // end of analizar_orden()


//////////////////////////////////////////////////
//  LOOP
//////////////////////////////////////////////////
void loop()
{
 static unsigned long time1=0;
 static unsigned long time=0;
 unsigned char aux[1];
 unsigned char primera_vez=1;
 static unsigned char us_sensor=0;
 unsigned long referencia, last_pulses = 0, current_pulses;
 unsigned char flag=1;
 unsigned char contador_dep=0;

 while(1)
 {
    // Secuence to read the Ultra-sound sensors
    // in each iteration one sensor is read
    // Must be taken into account that a delay
    // is obtained by reading ultra-sound sensor
    if (us_sensor==0)
    {
      // Faltan asignar los pines
      dist_us_sensor_central= us_range(F_US_TRIG, F_US_ECHO);
      us_sensor=1;
    }
    else
      if (us_sensor==1)
      {
        // No faltan asignar los pines
        dist_us_sensor_left= us_range(L_US_TRIG, L_US_ECHO);
        us_sensor=2;
      }
      else
        {
          // No faltan asignar los pines
        dist_us_sensor_right= us_range(R_US_TRIG, R_US_ECHO);
        us_sensor=0;
        }

/*  if (flag==1)
  {
  orden[0]='4';
  analizar_orden();     //Llama a la funcion "analizar_cadena".
  flag=0;
  }
 */
  //Si hay algo disponible en el puerto serie lo lee y lo
  if (Serial.available() > 0)
  {
    orden[0]='Z';
    theta_max=0;
    Serial.readBytes(orden, 1);
    // Every time a command is going to be reveived the error is reseted,
    // if it is not asking for error code.
    if (orden[0]!='E')
        ERROR_CODE=NO_ERROR;

    analizar_orden();     //Llama a la funcion "analizar_cadena".
    // Clean the buffer
    while(Serial.available()!=0)
      Serial.readBytes(aux,1);
  }

  // Each state perform a different movement
  switch (ESTADO)
  {
    case   EST_REPOSO:
      primera_vez=1;
      breaking_period=0;
      if((millis()-time1)>5000)
      {
        Serial.print(".");
        time1=millis();
      }
      // Just in case an error produce movement of the motors
      stop_motors();
      break;
    // Movement where the right wheel should rotate faster
    // Right wheel will be the reference.
    case EST_RIGHT_FASTER:
    case EST_LEFT_FASTER:
    case EST_TEST_CIRC_R:
    case EST_TEST_CIRC_L:
      // Take the reference of the faster wheel to compare distances
      if (ESTADO==EST_RIGHT_FASTER || ESTADO==EST_TEST_CIRC_R)
        referencia=encoderDER;
      else
        referencia=encoderIZQ;
      // Stop just a few pulses before because of the inertia
      if((referencia)<PULSES_NUM-INERTIA_LIMIT)
      {
        // Reduce speed in order to get objective pulses
        if (referencia>(PULSES_NUM-BREAK_PULSES))
        {
          breaking_period=1;
          if (vell>125)
          {
            SPEED_INI_L=125;
            vell=125;
          }
          if (velr>125)
          {
            SPEED_INI_R=125;
            velr=125;
          }
          if (primera_vez==1)
          {
            primera_vez=0;
           // Write in PWM the speeds for each wheel
           analogWrite(MOT_R_PWM_PIN, velr);
           analogWrite(MOT_L_PWM_PIN, vell);
          }
        }
        if(millis()-time>TIME_PID)
        {
          time=millis();
          one_faster_dist();
          contador_dep++;
          if (contador_dep>4)
          {
            dep();
            contador_dep=0;
          }
        }
      }
      else
      {
        stop_motors();
        // Make a delay in order to be sure that the wheels are stopped
        delay(1000);
        dep();
        update_global_positions();
        ESTADO=EST_REPOSO;
      }
    break;

    // Movement where both wheels should rotate the same distance
    case EST_ROTATE_UNCLK:
    case EST_ROTATE_CLK:
    case EST_MOV_STRAITGH:
      if(encoderDER<PULSES_NUM-INERTIA_LIMIT)
      {
        // Reduce speed in order to get objective pulses
        if (encoderDER>(PULSES_NUM-BREAK_PULSES))
        {
          breaking_period=1;
          if (vell>125)
          {
            SPEED_INI_L=125;
            vell=125;
          }
          if (velr>125)
          {
            SPEED_INI_R=125;
            velr=125;
          }
          if (primera_vez==1)
          {
            primera_vez=0;
            // Write in PWM the speeds for each wheel
            analogWrite(MOT_R_PWM_PIN, velr);
            if (vell==0)
            {
              digitalWrite(MOT_L_A_PIN, LOW);
              digitalWrite(MOT_L_B_PIN, LOW);
              analogWrite(MOT_L_PWM_PIN, 255);
            }
            else
            {
              digitalWrite(MOT_L_A_PIN, HIGH);
              digitalWrite(MOT_L_B_PIN, LOW);
              analogWrite(MOT_L_PWM_PIN, vell);
            }
          }
        }
          update_global_positions();

        //if(millis()-time>TIME_PID)
        if(current_pulses = (unsigned int) (((encoderDER + encoderIZQ) / 2) + 0.4999) - last_pulses > PID_PULSES)
        {
          //time=millis();
          //Serial.println("Entered PID calibration!");
          last_pulses = current_pulses;
          straigh_dist();
  //        delay(250);
          if (theta_max<abs(Theta))
            theta_max=abs(Theta);
          dep();
        }
      }
      else
      {
        stop_motors();
        dep();
        // Make a delay in order to be sure that the wheels are stopped
        delay(1000);
        ESTADO=EST_REPOSO;
        update_global_positions();
      }
      break;
      default:
      break;
  }
 }// end of while(1)
}  // end of loop
