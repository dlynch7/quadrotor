#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <curses.h>
#include "vive.h"

//gcc -o student student.cpp -lwiringPi -lncurses -lm

#define frequency 25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C

#define AR 0.005 // complementary filter roll coefficient
#define AP 0.01 // complementary filter pitch coefficient

// Definitions for P controller in week 3
//add constants
#define PWM_MAX 1900
#define NEUTRAL_THRUST 1500
#define frequency 25000000.0
#define LED0 0x6
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define LED_MULTIPLYER 4


enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

// //global variables added in week 2
// struct Keyboard {
//   char key_press;
//   int heartbeat;
//   int version;
// };

//update shared memory struct
struct Keyboard {
  int keypress;
	float pitch;
	float roll;
	float yaw;
	float thrust;
  int version;
};

Position local_p;
int vive_prev_version = 0;

Keyboard* shared_memory;
int run_program = 1;

int setup_imu(void);
void calibrate_imu(void);
void read_imu(void);
void update_filter(void);
void safety_check(void);
void setup_keyboard(void);
void trap(int);
void init_pwm(void);
void init_motor(uint8_t);
void set_PWM(uint8_t,float);
void pid_update(void);
void get_joystick(void);
void get_vive(void);

unsigned char PAUSED = 1;

//pwm variable added in week 3
int pwm;

//global variables
int imu;
int safe_state = 1;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //gyro xyz, accel xyz
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;

struct timespec tv;
float prev_version_time = 0;
int vive_version = 0;
float vive_time_prev = 0;

float vive_x = 0;
float vive_y = 0;
float vive_z = 0;
float vive_yaw = 0;
float version_timer = 0;

float previous_pitch = 0;
float previous_roll = 0;

float roll_imu = 0;
float pitch_imu = 0;

float compFiltRoll = 0; // roll angle calculated by complementary filter in update_filter()
float compFiltPitch = 0; // pitch angle calculated by complementary filter in update_filter()

float Thrust = 0;
float desiredPitch = 0;
float desiredRoll = 0;
float desiredYaw = 0;

int main (int argc, char *argv[])
{
    int heartbeat_prev = 0;
    struct timeval tm;
    long curr_time = 0;
    long heart_beat_timer = 0;

    init_shared_memory();

    //motor initializations (added week 3)
    init_pwm();
    init_motor(0);
    init_motor(1);
    init_motor(2);
    init_motor(3);
    delay(1000);

    //to set motor speed call
    //set_PWM(<motor number>,<speed>); //speed between 1000 and PWM_MAX, motor 0-3
    if (run_program==0){
        set_PWM(0,1000);
        set_PWM(1,1000);
        set_PWM(2,1000);
        set_PWM(3,1000);
    }

    setup_imu();

    //in main before while(1) loop add...
    setup_keyboard();
    signal(SIGINT, &trap);

    //to refresh values from shared memory first
    Keyboard keyboard=*shared_memory;

    while(run_program==1)
    {

      //to refresh values from shared memory first
      local_p=*position;
      Keyboard keyboard=*shared_memory;
      // printf("%d\r\n",shared_memory->keypress);

      safety_check();

      get_vive();
      read_imu();
      update_filter();
      get_joystick();
      if (PAUSED==0) {
        pid_update();
      }
      previous_pitch = compFiltPitch;
      previous_roll = compFiltRoll;

    }

    set_PWM(0,1000);
    set_PWM(1,1000);
    set_PWM(2,1000);
    set_PWM(3,1000);

    return 0;
}

void get_vive() {

  vive_version = local_p.version;
  printf("version: %d\t previous: %d\t x: %f\t y: %f\t z: %f\t yaw: %f\n",vive_version,vive_prev_version,vive_x,vive_y,vive_z,vive_yaw);
  vive_x = local_p.x;
  vive_y = local_p.y;
  vive_z = local_p.z;
  vive_yaw = local_p.yaw;

  if(vive_version == vive_prev_version){ // if vive "heartbeat" has not changed yet
    printf("Why wont this thing change!!\n");
    //get current time in nanoseconds
    timespec_get(&tv,TIME_UTC);
    time_curr=tv.tv_nsec;
    //compute time since last execution
    float vive_diff=time_curr-vive_time_prev;
    // if (vive_diff > 500000) { // 500000 ns = 0.5 s
    //   // End this program:
    //   printf("Vive heartbeat stopped. Ending.\n");
    //   run_program = 0;
    //   set_PWM(0,1000);
    //   set_PWM(1,1000);
    //   set_PWM(2,1000);
    //   set_PWM(3,1000);
    // }
  } else { // vive "heartbeat" has changed
    printf("I hear the tell-tale heart\n");
    vive_time_prev = time_curr;
    vive_prev_version = vive_version;
  }

  if (fabs(vive_x) > 1000) {
    run_program = 0;
    set_PWM(0,1000);
    set_PWM(1,1000);
    set_PWM(2,1000);
    set_PWM(3,1000);
  }
  if (fabs(vive_y) > 1000) {
    run_program = 0;
    set_PWM(0,1000);
    set_PWM(1,1000);
    set_PWM(2,1000);
    set_PWM(3,1000);
  }


  // printf("version: %d\t x: %f\t y: %f\t z: %f\t yaw: %f\n",vive_version,vive_x,vive_y,vive_z,vive_yaw);
  // vive_prev_version = vive_version;
}

void get_joystick(void) { // grab cmds from joystick, added week 5
  Thrust = NEUTRAL_THRUST - (shared_memory->thrust - 128)*2.0;
  desiredPitch = 0 -(shared_memory->pitch - 128)/12.0;
  desiredRoll = 0 + (shared_memory->roll - 128)/12.0;
  desiredYaw = 0 +(shared_memory->yaw - 128)*1.2;

  if (shared_memory->keypress == ' '){
    set_PWM(0,1000);
    set_PWM(1,1000);
    set_PWM(2,1000);
    set_PWM(3,1000);
    run_program = 0;
    printf("space pressed\r\n");
    printf("ending program\n\r");
  }

  if (shared_memory->keypress==33) { // if PAUSE pressed
    set_PWM(0,1000);
    set_PWM(1,1000);
    set_PWM(2,1000);
    set_PWM(3,1000);

    PAUSED = 1;
  }

  if (shared_memory->keypress==34) { // if UN-PAUSE pressed
    PAUSED = 0;
  }

  if (shared_memory->keypress==35) { // if CALIBRATE pressed
    calibrate_imu();
  }
  // printf("Yaw Cmd: %f\n", desiredYaw);
}

// PID controller, added week 3
void pid_update(){
  float pP = 15; //16
  float pI = .01; //.25
  float pD = 200; //500
  float pitchError = desiredPitch -compFiltPitch;
  float pitchVelocity = compFiltPitch - previous_pitch;
  static float pitchIntegral = 0;
  static float pitchControl = 0;

  float rP = 15; // 16
  float rI = 0.01; // 0.25
  float rD = 200; // 100
  float rollError = desiredRoll -compFiltRoll;
  float rollVelocity = compFiltRoll - previous_roll;
  static float rollIntegral = 0;
  static float rollControl = 0;

  float yP = .5;
  static float yawControl = 0;

  pitchIntegral += pI*pitchError;
  if (pitchIntegral > 100) {
    pitchIntegral = 100;
  }
  if (pitchIntegral < -100) {
    pitchIntegral = -100;
  }

  rollIntegral += rI*rollError;
  if (rollIntegral > 100) {
    rollIntegral = 100;
  }
  if (rollIntegral < -100) {
    rollIntegral = -100;
  }

  pitchControl = pitchVelocity*pD - pP*pitchError - pitchIntegral; // add for m0,m2, subtract for m1,m3
  // pitchControl = 0;
  rollControl = rollVelocity*rD - rP*rollError - rollIntegral; // add for m0,m2, subtract for m1,m3
  // printf("Roll control: %f\n",rollControl);
  yawControl = yP*(desiredYaw - imu_data[2]);

  float m0PWM = Thrust + pitchControl + rollControl - yawControl;
  float m1PWM = Thrust - pitchControl + rollControl + yawControl;
  float m2PWM = Thrust + pitchControl - rollControl + yawControl;
  float m3PWM = Thrust - pitchControl - rollControl - yawControl;


  // set_PWM(0,int(m0PWM));
  // set_PWM(1,int(m1PWM));
  // set_PWM(2,int(m2PWM));
  // set_PWM(3,int(m3PWM));

  // printf("%f\n",yawControl);
  // printf("%f\t%f\t%f\t%f\n", m0PWM,m1PWM,m2PWM,m3PWM);
}

void calibrate_imu()
{
  float imuSum[6] = {0,0,0,0,0,0}; // array for storing IMU data to be used for calibration

  x_gyro_calibration=0;
  y_gyro_calibration=0;
  z_gyro_calibration=0;
  roll_calibration=0;
  pitch_calibration=0;
  accel_z_calibration=0;

  // read from IMU 1000 times and append data to array imuSum:
  for (size_t i = 0; i < 1000; i++) {
    read_imu();
    imuSum[0] += imu_data[0];
    imuSum[1] += imu_data[1];
    imuSum[2] += imu_data[2];
    imuSum[3] += roll_angle;
    imuSum[4] += pitch_angle;
    imuSum[5] += imu_data[5];
  }

  // compute averages:
  x_gyro_calibration=imuSum[0]/1000;
  y_gyro_calibration=imuSum[1]/1000;
  z_gyro_calibration=imuSum[2]/1000;
  roll_calibration=imuSum[3]/1000;
  pitch_calibration=imuSum[4]/1000;
  accel_z_calibration=imuSum[5]/1000;
printf("calibration complete, %f %f %f %f %f %f\r\n",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);


}

void read_imu()
{
  // Read order:
  //    x accel
  //    y accel
  //    z accel
  //    x gyro
  //    y gyro
  //    z gyro

  // Read x accel:
  int address=59;// set address value for accel x value
  float ax=0;
  float az=0;
  float ay=0;
  float rollRad = 0, rollDeg = 0;
  float pitchRad = 0, pitchDeg = 0;
  int vh,vl;
  //read in data
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  //convert 2 complement
  int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  // convert vw from raw values to "g's"
  imu_data[3] = ((float)vw)*2/32767;

  // Read y accel:
  address=61;// set address value for accel y value
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  // convert vw from raw valeus to "g's"
  imu_data[4] =((float)vw)*2/32767;

  // Read z accel:
  address=63;// set addres value for accel z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  // convert vw from raw valeus to "g's"
  imu_data[5] = ((float)vw)*2/32767;

  // Read x gyro:
  address=67;// set addres value for gyro x value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  // convert vw from raw values to degrees/second
  imu_data[0] = ((float)vw)*500/32767 - x_gyro_calibration;

  // Read y gyro:
  address=69;// set addres value for gyro y value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  // convert vw from raw values to degrees/second
  imu_data[1] = ((float)vw)*500/32767 - y_gyro_calibration;

  // Read z gyro:
  address=71;// set addres value for gyro z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  // convert vw from raw values to degrees/second
  imu_data[2] = ((float)vw)*500/32767 - z_gyro_calibration;

  // calculating roll and pitch angles
  roll_angle = atan2(imu_data[3],-imu_data[5])*180/3.14159 - roll_calibration; // maybe -imu_data[3]?
  pitch_angle = atan2(-imu_data[4],-imu_data[5])*180/3.14159 - pitch_calibration; // maybe -imu_data[4]?

// printf("%f\t%f\t%f\t%f\t%f\r\n",imu_data[0],imu_data[1],imu_data[2],roll_angle,pitch_angle);


}

void update_filter()
{
  float roll_gyro_delta;
  float pitch_gyro_delta;

  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev;

  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;

  roll_imu = roll_imu + imu_data[1]*imu_diff; // roll angle from integrating imu
  pitch_imu = pitch_imu + imu_data[0]*imu_diff; // pitch angle from integrating imu

  roll_gyro_delta = imu_data[1]*imu_diff; // integrate over one timestep, imu_data[1] is y_gyro
  pitch_gyro_delta = imu_data[0]*imu_diff; // integrate over one timestep, imu_data[0] is x_gyro

  //comp. filter for roll, pitch here:
  compFiltRoll = roll_angle*AR + (1-AR)*(roll_gyro_delta + compFiltRoll);
  compFiltPitch = pitch_angle*AP + (1-AP)*(pitch_gyro_delta + compFiltPitch);

  // printf("%f\t%f\t%f\r\n",compFiltRoll,roll_angle,imu_data[1]);
  // printf("%f\t%f\t%f\r\n",compFiltRoll, roll_angle, imu_data[1]);
}


int setup_imu()
{
  wiringPiSetup ();


  //setup imu on I2C
  imu=wiringPiI2CSetup (0x68) ; //accel/gyro address

  if(imu==-1)
  {
    printf("-----cant connect to I2C device %d --------\n",imu);
    return -1;
  }
  else
  {

    printf("connected to i2c device %d\n",imu);
    printf("imu who am i is %d \n",wiringPiI2CReadReg8(imu,0x75));

    uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS


    //init imu
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x00);
    printf("                    \n\r");
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00);
    wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04
    int c=wiringPiI2CReadReg8(imu,  GYRO_CONFIG);
    wiringPiI2CWriteReg8(imu,  GYRO_CONFIG, c & ~0xE0);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c | Ascale << 3);
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2, c & ~0x0F); //
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2,  c | 0x00);
  }
  return 0;
}

//function added in week 2
void setup_keyboard()
{
  int segment_id;
  struct shmid_ds shmbuffer;
  int segment_size;
  const int shared_segment_size = 0x6400;
  int smhkey=33222;

  /* Allocate a shared memory segment.  */
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
  /* Attach the shared memory segment.  */
  shared_memory = (Keyboard*) shmat (segment_id, 0, 0);
  printf ("shared memory attached at address %p\n", shared_memory);
  /* Determine the segment's size. */
  shmctl (segment_id, IPC_STAT, &shmbuffer);
  segment_size  =               shmbuffer.shm_segsz;
  printf ("segment size: %d\n", segment_size);
  /* Write a string to the shared memory segment.  */
  // sprintf(shared_memory, "test!!!!.");
  }

  //when cntrl+c pressed, kill motors
  void trap(int signal)
  {

    set_PWM(0,1000);
    set_PWM(1,1000);
    set_PWM(2,1000);
    set_PWM(3,1000);

     printf("ending program\n\r");
     run_program=0;

  }

void safety_check() {
  // safe_state = 1;

  if ((imu_data[0] > 300) | (imu_data[1] > 300) | (imu_data[2] > 300)) { // any gyro rate > 300 dps
    // safe_state = 0;
    set_PWM(0,1001);
    set_PWM(1,1001);
    set_PWM(2,1001);
    set_PWM(3,1001);
    run_program = 0;
    printf("Above gyro rate limit.\r\n");

  }
  // if ((fabs(imu_data[3]) > 1.8) | (fabs(imu_data[4]) > 1.8) | (fabs(imu_data[5]) > 1.8)) { // any accelerometer value > 1.8 g (< -1.8?)
  //   // safe_state = 0;
  //   set_PWM(0,1001);
  //   set_PWM(1,1001);
  //   set_PWM(2,1001);
  //   set_PWM(3,1001);
  //   run_program = 0;
  //   printf("Above accelerometer max.\r\n");
  //
  // }
  // if ((fabs(imu_data[3]) < 0.25) && (fabs(imu_data[4]) < 0.25) && (fabs(imu_data[5]) < 0.25)) { // ALL accelerometer values < 0.25 g
  //   // safe_state = 0;
  //   set_PWM(0,1001);
  //   set_PWM(1,1001);
  //   set_PWM(2,1001);
  //   set_PWM(3,1001);
  //   run_program = 0;
  //   printf("Below accelerometer min.\r\n");
  // }
  if (fabs(compFiltRoll) > 45) { // roll angle > 45 deg or < -45 deg
    // safe_state = 0;
    set_PWM(0,1001);
    set_PWM(1,1001);
    set_PWM(2,1001);
    set_PWM(3,1001);
    run_program = 0;
    printf("Exceeded roll angle range.\r\n");
  }
  if (fabs(compFiltPitch) > 45) { // pitch angle > 45 deg or < -45 deg
    // safe_state = 0;
    set_PWM(0,1001);
    set_PWM(1,1001);
    set_PWM(2,1001);
    set_PWM(3,1001);
    run_program = 0;
    printf("Exceeded pitch angle range.\r\n");
  }

  // return safe_state;

}

//
void init_pwm()
{

    pwm=wiringPiI2CSetup (0x40);
    if(pwm==-1)
    {
      printf("-----cant connect to I2C device %d --------\n",pwm);

    }
    else
    {

      float freq =400.0*.95;
      float prescaleval = 25000000;
      prescaleval /= 4096;
      prescaleval /= freq;
      prescaleval -= 1;
      uint8_t prescale = floor(prescaleval+0.5);
      int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
      int sleep	= settings | 0x10;
      int wake 	= settings & 0xef;
      int restart = wake | 0x80;
      wiringPiI2CWriteReg8(pwm, 0x00, sleep);
      wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
      wiringPiI2CWriteReg8(pwm, 0x00, wake);
      delay(10);
      wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
    }
}

// Motor initialization (added week 3)
void init_motor(uint8_t channel)
{
	int on_value=0;

	int time_on_us=900;
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1200;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1000;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

}
//PWM set function (added week 3)
void set_PWM( uint8_t channel, float time_on_us)
{
  if(run_program==1)
  {
    if(time_on_us>PWM_MAX)
    {
      time_on_us=PWM_MAX;
    }
    else if(time_on_us<1000)
    {
      time_on_us=1000;
    }
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }

}
