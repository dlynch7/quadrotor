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

//gcc -o week1 week_1.cpp -lwiringPi -lncurses -lm

#define frequency 25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C

#define AR 0.02 // complementary filter roll coefficient
#define AP 0.02 // complementary filter pitch coefficient


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

//global variables added in week 2
struct Keyboard {
  char key_press;
  int heartbeat;
  int version;
};
Keyboard* shared_memory;
int run_program=1;

int setup_imu();
void calibrate_imu();
void read_imu();
void update_filter();
void safety_check();

//global variables
int imu;
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

float roll_imu = 0;
float pitch_imu = 0;

float compFiltRoll = 0; // roll angle calculated by complementary filter in update_filter()
float compFiltPitch = 0; // pitch angle calculated by complementary filter in update_filter()


void setup_keyboard(void);
void trap(int);

int main (int argc, char *argv[])
{
    int heartbeat_prev = 0;
    struct timeval tm;
    long curr_time = 0;
    long heart_beat_timer = 0;

    setup_imu();
    calibrate_imu();

    //in main before while(1) loop add...
    setup_keyboard();
    signal(SIGINT, &trap);

    //to refresh values from shared memory first
    Keyboard keyboard=*shared_memory;

    //get current time
    gettimeofday(&tm,NULL);
    heart_beat_timer=tm.tv_sec*1000LL+tm.tv_usec/1000;

    while(run_program==1)
    {
      //to refresh values from shared memory first
      Keyboard keyboard=*shared_memory;
      if (shared_memory->key_press == ' '){
        run_program = 0;
        printf("space pressed\r\n");
      }

      //get current time
      gettimeofday(&tm,NULL);
      curr_time=tm.tv_sec*1000LL+tm.tv_usec/1000;

      // if heartbeat has not changed in 0.25 seconds, stop program.
      if(curr_time>heart_beat_timer+250)
      {
          run_program = 0;
          printf("heartbeat stopped. CPR STAT.\n\r");
      }

      if (heartbeat_prev < shared_memory->heartbeat) {
        heart_beat_timer=curr_time;
      }

      heartbeat_prev = shared_memory->heartbeat;

      safety_check();

      read_imu();
      update_filter();

    }
    return 0;
}

void calibrate_imu()
{
  float imuSum[6] = {0,0,0,0,0,0}; // array for storing IMU data to be used for calibration

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
printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);


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
  printf("%f\t%f\t%f\r\n",compFiltPitch,pitch_angle,imu_data[0]);
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



     printf("ending program\n\r");
     run_program=0;
  }

void safety_check() {
  if ((imu_data[0] > 300) | (imu_data[1] > 300) | (imu_data[2] > 300)) { // any gyro rate > 300 dps
    run_program = 0;
    printf("Above gyro rate limit.\r\n");
  }
  if ((fabs(imu_data[3]) > 1.8) | (fabs(imu_data[4]) > 1.8) | (fabs(imu_data[5]) > 1.8)) { // any accelerometer value > 1.8 g (< -1.8?)
    run_program = 0;
    printf("Above accelerometer max.\r\n");
  }
  if ((fabs(imu_data[3]) < 0.25) && (fabs(imu_data[4]) < 0.25) && (fabs(imu_data[5]) < 0.25)) { // ALL accelerometer values < 0.25 g
    run_program = 0;
    printf("Below accelerometer min.\r\n");
  }
  if (fabs(roll_angle) > 45) { // roll angle > 45 deg or < -45 deg
    run_program = 0;
    printf("Exceeded roll angle range.\r\n");
  }
  if (fabs(pitch_angle) > 45) { // pitch angle > 45 deg or < -45 deg
    run_program = 0;
    printf("Exceeded pitch angle range.\r\n");
  }
}
