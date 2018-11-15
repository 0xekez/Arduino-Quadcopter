/*
 * MPU
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

/*
 * PID
 */

# define MOTOR_LF 6
# define MOTOR_LB 11
# define MOTOR_RF 10
# define MOTOR_RB 9

//TOTAL ANGLES
float total_angle_x, total_angle_y;
/*
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

//store time information
float elapsedTime, time, timePrev;

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
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

void loop() {
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//        Serial.print("ypr\t");
//        Serial.print(ypr[0] * 180/M_PI);
//        Serial.print("\t");
//        Serial.print(ypr[1] * 180/M_PI);
//        Serial.print("\t");
//        Serial.println(ypr[2] * 180/M_PI);
        
      // pitch = x
      // roll = y
      total_angle_x = ypr[1]* 180/M_PI;
      total_angle_y = ypr[2]* 180/M_PI;

      /*
       * PID
       */
      timePrev = time;
      time = millis();
      // in seconds
      elapsedTime = (time-timePrev)/1000;
      
      //TODO - update with bluetooth control
      roll_desired_angle = 0;
      pitch_desired_angle = 0;
      input_throttle = 200;

      roll_error = total_angle_y - roll_desired_angle;
      pitch_error = total_angle_x - pitch_desired_angle;
      //Serial.println(roll_error);

      roll_pid_p = roll_error*roll_kp;
      pitch_pid_p = pitch_error*pitch_kp;
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
     if (roll_pid < -75){
      roll_pid = -75;
     }
     if (roll_pid > 75){
      roll_pid = 75;
     }
    if (pitch_pid < -75){
      pitch_pid = -75;
    }
    if (pitch_pid > 75){
      pitch_pid = 75;
    }
    
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
    Serial.print("lf :");
    Serial.print(motor_l_f);
    Serial.print("\t");
    Serial.print("lb: ");
    Serial.print(motor_l_b);
    Serial.print("\t");
    Serial.print("rf: ");
    Serial.print(motor_r_f);
    Serial.print("\t");
    Serial.print("rb: ");
    Serial.print(motor_r_b);
    Serial.println();
      
      drive_motor(MOTOR_RF, motor_r_f);
      drive_motor(MOTOR_RB, motor_r_b);
      drive_motor(MOTOR_LF, motor_l_f);
      drive_motor(MOTOR_LB, motor_l_b);
      
      // store previous errors
      roll_prev_error = roll_error;
      pitch_prev_error = pitch_error;
    }
}

void drive_motor(int gate, int value){
  analogWrite(gate, value);
}
