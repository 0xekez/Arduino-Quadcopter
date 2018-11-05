# include <Wire.h>

# define MOTOR_LF 1
# define MOTOR_LB 2
# define MOTOR_RF 3
# define MOTOR_RB 4

// GYRO
//store time information
float elapsedTime, time, timePrev;
// used to calculate the gyro error
int gyro_error = 0;
// raw data read
int16_t gyro_raw_x, gyro_raw_y;
// cleaned up angles
float gyro_x, gyro_y;
//gyro error
float gyro_error_x, gyro_error_y;

// ACCEL
//for calculating acc error initially
int acc_error = 0;
float rad_to_deg = 180/3.141592654;
float acc_raw_x, acc_raw_y, acc_raw_z;
float acc_x, acc_y;
float acc_error_x, acc_error_y;

//TOTAL ANGLES
float total_angle_x, total_angle_y;

/*
 * PID
 * we need two sets of pid constants,
 * one for pitch (x) and one for roll (y)
 */
float roll_kp = 1;
float pitch_kp = 1;
float roll_ki = 0;
float pitch_ki = 0;
float roll_kd = 0;
float pitch_kd =0;
// vars to hold actual vals of pid
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;
float roll_pid, pitch_pid;
//vars for holding error vals for PID
float pitch_error, roll_error;
float pitch_prev_error, roll_prev_error;

/*
 * CONTROLLER INPUTS
 */
float roll_desired_angle;
float pitch_desired_angle;
float input_throttle;

/*
 * MOTOR VALUES
 */
float motor_l_f, motor_l_b, motor_r_f, motor_r_b;

void setup() {
  // SETUP MPU 6050
  Wire.begin();
  //reset sensor
  Wire.beginTransmission(0x68);
  // make the reset by putting a zero in the 6B register            
  Wire.write(0x6B);
  Wire.write(0x00);
  //end the transmition
  Wire.endTransmission(true);
  // Gyro config
  Wire.beginTransmission(0x68);
  // write to gyro congig register
  Wire.write(0x1B);
  // set the register bits as 00010000 (1000dps full scale)
  Wire.write(0x10);
  Wire.endTransmission(true);
  //Acc config
  Wire.beginTransmission(0x68);
  // write to the ACCEL_CONFIG register
  Wire.write(0x1C);
  // set the register bits as 00010000 (+/- 8g full scale range)
  Wire.write(0x10);
  Wire.endTransmission(true); 

  Serial.begin(9600);
  // start counting time
  time = millis();

  /*
   * caclulate the gyro error, using the averge of 200 measurements.
   * the drone should be sitting on a level surface for this
   */
  if (gyro_error == 0){ // if its not already set to a value
    for (int i = 0; i < 200; i++){
      Wire.beginTransmission(0x68);
      // address the gyro data
      Wire.write(0x43);
      Wire.endTransmission(false);
      // ask for 4 registers of the data
      Wire.requestFrom(0x68, 4, true);
      // shift and sum the data
      gyro_raw_x=Wire.read()<<8|Wire.read();
      gyro_raw_y=Wire.read()<<8|Wire.read();

      // the actual angle is raw/32.8
      gyro_error_x = gyro_error_x + gyro_raw_x/32.8;
      gyro_error_y = gyro_error_y + gyro_raw_y/32.8;
    } // end for
  } // end if
  // calculate the average
  gyro_error = 1;
  gyro_error_x = gyro_error_x/200;
  gyro_error_y = gyro_error_y/200;
  Serial.println("calculated gyro error:");
  Serial.println(gyro_error_x);
  Serial.println(gyro_error_y);

  /*
   * calculate the accel error using the avereage of 200 measurements
   * the drone should not be moving around for this
   */
  if (acc_error == 0){
    for (int i = 0; i < 200; i++){
      Wire.beginTransmission(0x68);
      // ask for the 0x3B register- correspond to AcX
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true); 

      // really similar process here as the one above
      acc_raw_x=(Wire.read()<<8|Wire.read())/4096.0 ;
      acc_raw_y=(Wire.read()<<8|Wire.read())/4096.0 ;
      acc_raw_z=(Wire.read()<<8|Wire.read())/4096.0 ;

      acc_error_x = acc_error_x + atan(acc_raw_x/sqrt(pow(acc_raw_x,2)+pow(acc_raw_z,2)))*rad_to_deg;
      acc_error_y = acc_error_y + atan((-1)*acc_raw_x/sqrt(pow(acc_raw_y,2)+pow(acc_raw_z,2)))*rad_to_deg;
    } // end for
  } // end if
  acc_error = 1;
  acc_error_x = acc_error_x/200;
  acc_error_y = acc_error_y/200;
} // end setup

void loop() {
  timePrev = time;
  time = millis();
  // in seconds
  elapsedTime = (time-timePrev)/1000;

  /*
   * GYRO READ
   */
  Wire.beginTransmission(0x68);
  // address gyro data
  Wire.write(0x43);
  Wire.endTransmission(false);
  // ask for 4 registers of the data
  Wire.requestFrom(0x68, 4, true);
  // shift and sum the data
  gyro_raw_x=Wire.read()<<8|Wire.read();
  gyro_raw_y=Wire.read()<<8|Wire.read();

  // the actual angle is raw/32.8 - error
  gyro_raw_x = gyro_raw_x/32.8 - gyro_error_x;
  gyro_raw_y = gyro_raw_y/32.8 - gyro_error_y;

  //integrate to obtain actual values
  gyro_x = gyro_raw_x * elapsedTime;
  gyro_y = gyro_raw_y * elapsedTime;

  /*
   * ACCEL READ
   */
  Wire.beginTransmission(0x68);
  // ask for the 0x3B register- correspond to AcX
  Wire.write(0x3B);
  Wire.endTransmission(false);
  // ask for 6 registers of data
  Wire.requestFrom(0x68,6,true); 
  
  // really similar process here as the one above
  acc_raw_x=(Wire.read()<<8|Wire.read())/4096.0 ;
  acc_raw_y=(Wire.read()<<8|Wire.read())/4096.0 ;
  acc_raw_z=(Wire.read()<<8|Wire.read())/4096.0 ;
  
  acc_x = atan(acc_raw_x/sqrt(pow(acc_raw_x,2)+pow(acc_raw_z,2)))*rad_to_deg - acc_error_x;
  acc_y = atan((-1)*acc_raw_x/sqrt(pow(acc_raw_y,2)+pow(acc_raw_z,2)))*rad_to_deg - acc_error_y;

  /*
   * TOTAL ANGLE
   * total angle is a weighted combination of the two values
   * mostly gyro but with a little accel
   */

   total_angle_x = 0.98*(gyro_x + total_angle_x) + 0.2*(acc_x);
   total_angle_y = 0.98*(gyro_y + total_angle_y) + 0.2*(acc_y);
   
   Serial.print("Xº: ");
   Serial.print(gyro_x);
   Serial.print("   |   ");
   Serial.print("Yº: ");
   Serial.print(gyro_y);
   Serial.println(" ");

   /*
    * PID
    * when bluetooth is going - make the desired angles the input from bluetooth
    * roll coresponds with y-axis
    * pitch with x-axis
    */
    roll_desired_angle = 0;
    pitch_desired_angle = 0;
    
    // fill this in with data from bluetooth
    input_throttle = 100;
    
    roll_error = total_angle_y - roll_desired_angle;
    pitch_error = total_angle_x - pitch_desired_angle;
    
    // we only want to use the intergral if we are within +- 5 degrees
    if (-5 < roll_error < 5){
      roll_pid_i = roll_pid_i+(roll_ki*roll_error);
    }
    if (-5 < pitch_error < 5){
      pitch_pid_i = pitch_pid_i+(pitch_ki*pitch_error);
    }
    
    roll_pid_d = roll_kd*((roll_error - roll_prev_error)/elapsedTime);
    pitch_pid_d = pitch_kd*((pitch_error - pitch_prev_error)/elapsedTime);
    
    // final values are the sum of it all
    roll_pid = roll_pid_p + roll_pid_i + roll_pid_d;
    pitch_pid = pitch_pid_p + pitch_pid_i + pitch_pid_d;
    
    /*
     * there is some need for tuning here:
     * in a nutshell, we don't want PID values to be too big (I think)
     * so we constrain the values.
     * here, im constraining them to +- 75, but that should probably change in tuning.
     */
    roll_pid = constrain(roll_pid, -75, 75);
    pitch_pid = constrain(pitch_pid, -75, 75);
    
    /*
     * calculate the amount of thrust to give to each motor
     */
    motor_r_f  = input_throttle - roll_pid - pitch_pid;
    motor_r_b  = input_throttle - roll_pid + pitch_pid;
    motor_l_b  = input_throttle + roll_pid + pitch_pid;
    motor_l_f  = input_throttle + roll_pid - pitch_pid;
    
    /*
     * once, again, just to be safe, we constrain all these values to be within 0 and 255 inclusive
     * that is the range of values that our motors can be throttled to
     */
    motor_r_f  = constrain(motor_r_f, 0, 255);
    motor_r_b  = constrain(motor_r_b, 0, 255);
    motor_l_b  = constrain(motor_l_b, 0, 255);
    motor_l_f  = constrain(motor_l_f, 0, 255);
    
    drive_motor(MOTOR_RF, motor_r_f);
    drive_motor(MOTOR_RB, motor_r_b);
    drive_motor(MOTOR_LF, motor_l_f);
    drive_motor(MOTOR_LB, motor_l_b);
    
    // store previous errors
    roll_prev_error = roll_error;
    pitch_prev_error = pitch_error;

   Serial.print("RF: ");
   Serial.print(motor_r_f);
   Serial.print("   |   ");
   Serial.print("RB: ");
   Serial.print(motor_r_b);
   Serial.print("   |   ");
   Serial.print("LB: ");
   Serial.print(motor_l_b);
   Serial.print("   |   ");
   Serial.print("LF: ");
   Serial.print(motor_l_f);
   Serial.println("");
}// end loop

// TODO: if I have time, make this modify values based on the amount of battery left
// probably wont have time 
void drive_motor(int gate, int value){
  analogWrite(gate, value);
}
