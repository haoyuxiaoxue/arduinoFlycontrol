//2019-4-9 zhaohaoyu
//正在完善的arduino fly control
#include <timer.h>
// SBUS reading 4 channel signal and filting the signal cost 10ms
#include <sbus.h>
//MPU6050
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

Timer<1, micros> timer;
// used pins
#define SBUS_PIN 62
SBUS sbus;
int receiver_input_channel[4], receiver_input_channel_filted[4];

// timer variable
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2;

// filt variable
#define USED_NUM 5       // >=3
int n;                                   //count times
int arraycout = 0;
int arraymax = USED_NUM;
int channel_array[4][USED_NUM];
int maxnum, minnum;

// MPU6050 vars
MPU6050 accelgyro;

unsigned long now, lastTime = 0;
float dt;                                   //微分时间

int16_t ax, ay, az, gx, gy, gz;             //加速度计陀螺仪原始数据
float aax = 0, aay = 0, aaz = 0, agx = 0, agy = 0, agz = 0; //角度变量
long axo = 0, ayo = 0, azo = 0;             //加速度计偏移量
long gxo = 0, gyo = 0, gzo = 0;             //陀螺仪偏移量

float pi = 3.1415926;
float AcceRatio = 16384.0;                  //加速度计比例系数
float GyroRatio = 131.0;                    //陀螺仪比例系数

uint8_t n_sample = 8;                       //加速度计滤波算法采样个数
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};         //x,y轴采样队列
long aax_sum, aay_sum, aaz_sum;                     //x,y轴采样和

float a_x[10] = {0}, a_y[10] = {0}, a_z[10] = {0} , g_x[10] = {0} , g_y[10] = {0}, g_z[10] = {0}; //加速度计协方差计算队列
float Px = 1, Rx, Kx, Sx, Vx, Qx;           //x轴卡尔曼变量
float Py = 1, Ry, Ky, Sy, Vy, Qy;           //y轴卡尔曼变量
float Pz = 1, Rz, Kz, Sz, Vz, Qz;           //z轴卡尔曼变量

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 0;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0;                //Gain setting for the roll I-controller (0.3)
float pid_d_gain_roll = 40;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                   //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)
//Declaring Variables
int esc_1, esc_2, esc_3, esc_4, throttle;
//============  PID Variables  ============//
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
//============  Interrupt Control  ============//
int start;

void processSignal(int ch[4], int n)
{
  // the forward 9 numbers are  0, there is not filted
  if (channel_array[0][USED_NUM - 2] == 0)
  {
    for (int i = 0; i < 4; i++)channel_array[i][n] = receiver_input_channel[i];
  }
  // we get 9 numbers and a new channel_signal in receiver_input_channel. so we have 10 numbers
  else
  {
    for (int i = 0; i < 4; i++)channel_array[i][n] = receiver_input_channel[i]; // channel_array get numbers
    for (int i = 0 ; i < 4; i++)                                                                      // process data one channel by one channel
    {
      maxnum = max(channel_array[i][0], channel_array[i][1]);
      minnum = min(channel_array[i][0], channel_array[i][1]);
      int sum = 0;
      sum += channel_array[i][0]; sum += channel_array[i][1];
      for (int j = 2; j < USED_NUM; j++)
      {
        maxnum = max(maxnum, channel_array[i][j]);
        minnum = min(minnum, channel_array[i][j]);
        sum += channel_array[i][j];
      }
      sum -= maxnum; sum -= minnum;
      receiver_input_channel_filted[i] = sum / (USED_NUM - 2);
      receiver_input_channel_filted[i] = constrain(receiver_input_channel_filted[i], 900, 2000);
    }
  }
}

void getChannelSignal()
{
  for (int i = 1; i <= 4; i++)receiver_input_channel[i - 1] = sbus.getChannel(i);
}
void printChannelSignal()
{
  for (int i = 0 ; i < 4; i++)
  {
    Serial.print(receiver_input_channel[i]);
    Serial.print(" ");
    Serial.print(receiver_input_channel_filted[i]);
    Serial.print(" ");
  }
  //Serial.print(zero_timer);
  //Serial.print(" ");
  Serial.println(" ");
}

void checkSBUS()
{
  if (sbus.signalLossActive())
    Serial.print("SIGNAL_LOSS ");

  if (sbus.failsafeActive())
    Serial.print("FAILSAFE");
}

void kamf()
{
  unsigned long now = millis();             //当前时间(ms)
  dt = (now - lastTime) / 1000.0;           //微分时间(s)
  lastTime = now;                           //上一次采样时间(ms)

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值

  float accx = ax / AcceRatio;              //x轴加速度
  float accy = ay / AcceRatio;              //y轴加速度
  float accz = az / AcceRatio;              //z轴加速度

  aax = atan(accy / accz) * (-180) / pi;    //y轴对于z轴的夹角
  aay = atan(accx / accz) * 180 / pi;       //x轴对于z轴的夹角
  aaz = atan(accz / accy) * 180 / pi;       //z轴对于y轴的夹角

  aax_sum = 0;                              // 对于加速度计原始数据的滑动加权滤波算法
  aay_sum = 0;
  aaz_sum = 0;

  for (int i = 1; i < n_sample; i++)
  {
    aaxs[i - 1] = aaxs[i];
    aax_sum += aaxs[i] * i;
    aays[i - 1] = aays[i];
    aay_sum += aays[i] * i;
    aazs[i - 1] = aazs[i];
    aaz_sum += aazs[i] * i;

  }

  aaxs[n_sample - 1] = aax;
  aax_sum += aax * n_sample;
  aax = (aax_sum / (11 * n_sample / 2.0)) * 9 / 7.0; //角度调幅至0-90°
  aays[n_sample - 1] = aay;                      //此处应用实验法取得合适的系数
  aay_sum += aay * n_sample;                     //本例系数为9/7
  aay = (aay_sum / (11 * n_sample / 2.0)) * 9 / 7.0;
  aazs[n_sample - 1] = aaz;
  aaz_sum += aaz * n_sample;
  aaz = (aaz_sum / (11 * n_sample / 2.0)) * 9 / 7.0;

  float gyrox = - (gx - gxo) / GyroRatio * dt; //x轴角速度
  float gyroy = - (gy - gyo) / GyroRatio * dt; //y轴角速度
  float gyroz = - (gz - gzo) / GyroRatio * dt; //z轴角速度
  agx += gyrox;                             //x轴角速度积分
  agy += gyroy;                             //x轴角速度积分
  agz += gyroz;

  /* kalman start */
  Sx = 0; Rx = 0;
  Sy = 0; Ry = 0;
  Sz = 0; Rz = 0;

  for (int i = 1; i < 10; i++)
  { //测量值平均值运算
    a_x[i - 1] = a_x[i];                    //即加速度平均值
    Sx += a_x[i];
    a_y[i - 1] = a_y[i];
    Sy += a_y[i];
    a_z[i - 1] = a_z[i];
    Sz += a_z[i];

  }

  a_x[9] = aax;
  Sx += aax;
  Sx /= 10;                                 //x轴加速度平均值
  a_y[9] = aay;
  Sy += aay;
  Sy /= 10;                                 //y轴加速度平均值
  a_z[9] = aaz;
  Sz += aaz;
  Sz /= 10;

  for (int i = 0; i < 10; i++)
  {
    Rx += sq(a_x[i] - Sx);
    Ry += sq(a_y[i] - Sy);
    Rz += sq(a_z[i] - Sz);

  }

  Rx = Rx / 9;                              //得到方差
  Ry = Ry / 9;
  Rz = Rz / 9;

  Px = Px + 0.0025;                         // 0.0025在下面有说明...
  Kx = Px / (Px + Rx);                      //计算卡尔曼增益
  agx = agx + Kx * (aax - agx);             //陀螺仪角度与加速度计速度叠加
  Px = (1 - Kx) * Px;                       //更新p值

  Py = Py + 0.0025;
  Ky = Py / (Py + Ry);
  agy = agy + Ky * (aay - agy);
  Py = (1 - Ky) * Py;

  Pz = Pz + 0.0025;
  Kz = Pz / (Pz + Rz);
  agz = agz + Kz * (aaz - agz);
  Pz = (1 - Kz) * Pz;

  /* kalman end */
}

void updatechannels()
{
  if (!sbus.waitFrame(10))
  {
    //Serial.println("time out! ");
  }
  else
  {
    getChannelSignal();
    processSignal(receiver_input_channel[4], n);
    n++;
    if (n > (USED_NUM - 1))n -= USED_NUM;
    //printChannelSignal();
    checkSBUS();
  }
}


void lock_pwm() {
  esc_1 = 1050; esc_2 = 1050; esc_3 = 1050; esc_4 = 1050;
  while (zero_timer + 20000 > micros());                      //Start the pulse after 4000 micro seconds.
  zero_timer = micros();                                     //Reset the zero timer.
  PORTH |= B01111000;                                    // port 6 7 8 9 set to high
  timer_channel_1 = esc_1 + zero_timer;   //Calculate the time when digital port 6 is set low.
  timer_channel_2 = esc_2 + zero_timer;   //Calculate the time when digital port 7 is set low.
  timer_channel_3 = esc_3 + zero_timer;   //Calculate the time when digital port 8 is set low.
  timer_channel_4 = esc_4 + zero_timer;   //Calculate the time when digital port 9 is set low.

  while (PORTH & B00001000 || PORTH & B00010000 || PORTH & B00100000 || PORTH & B01000000) {                        //Execute the loop until digital port 8 til 11 is low.
    esc_loop_timer = micros();                                 //Check the current time.
    if (esc_loop_timer >= timer_channel_1 )PORTH &= B11110111;  //When the delay time is expired, digital port 6 is set low.
    if (esc_loop_timer >= timer_channel_2)PORTH &= B11101111;  //When the delay time is expired, digital port 7 is set low.
    if (esc_loop_timer >= timer_channel_3)PORTH &= B11011111;  //When the delay time is expired, digital port 8 is set low.
    if (esc_loop_timer >= timer_channel_4)PORTH &= B10111111;  //When the delay time is expired, digital port 9 is set low.
  }
}

bool generate_pwm(void *)
{
  throttle = receiver_input_channel_filted[2];                                      //We need the throttle signal as a base signal.
  esc_1 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
  esc_2 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
  esc_3 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
  esc_4 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

  esc_1 = constrain(esc_1, 950, 2000);
  esc_2 = constrain(esc_2, 950, 2000);
  esc_3 = constrain(esc_3, 950, 2000);
  esc_4 = constrain(esc_4, 950, 2000);

  while (micros() < zero_timer + 20000);            //Start the pulse after 4000 micro seconds.
  zero_timer = micros();                                     //Reset the zero timer.
  PORTH |= B01111000;                                    // port 6 7 8 9 set to high
  //PORTG |= 1 << 5; PORTE |= 1 << 3; PORTH |= 1 << 3; PORTH |= 1 << 4;  //4  5 6 7 port set to high
  timer_channel_1 = esc_1 + zero_timer;   //Calculate the time when digital port 6 is set high.
  timer_channel_2 = esc_2 + zero_timer;   //Calculate the time when digital port 7 is set high.
  timer_channel_3 = esc_3 + zero_timer;   //Calculate the time when digital port 8 is set high.
  timer_channel_4 = esc_4 + zero_timer;   //Calculate the time when digital port 9 is set high.

  while (PORTH & B00001000 || PORTH & B00010000 || PORTH & B00100000 || PORTH & B01000000) {                                //Execute the loop until digital port 6 til 9 is low.
    esc_loop_timer = micros();                                 //Check the current time.

    if (esc_loop_timer >= timer_channel_1 )PORTH &= B11110111;  //When the delay time is expired, digital port 6 is set low.
    if (esc_loop_timer >= timer_channel_2)PORTH &= B11101111;  //When the delay time is expired, digital port 7 is set low.
    if (esc_loop_timer >= timer_channel_3)PORTH &= B11011111;  //When the delay time is expired, digital port 8 is set low.
    if (esc_loop_timer >= timer_channel_4)PORTH &= B10111111;  //When the delay time is expired, digital port 9 is set low.

  }
  return true;
}

void setup() {
  //SBUS init
  pinMode(6, OUTPUT); pinMode(7, OUTPUT); pinMode(8, OUTPUT); pinMode(9, OUTPUT); pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  sbus.begin(SBUS_PIN, sbusBlocking);
  // mpu6050 init
  Wire.begin();
  accelgyro.initialize();                 //初始化

  unsigned short times = 200;             //采样次数
  for (int i = 0; i < times; i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
    axo += ax; ayo += ay; azo += az;      //采样和
    gxo += gx; gyo += gy; gzo += gz;

  }

  axo /= times; ayo /= times; azo /= times; //计算加速度计偏移
  gxo /= times; gyo /= times; gzo /= times; //计算陀螺仪偏移

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("ready start! unlock the drone! ");



    Serial.println(" start!  ");
    start = 2;
    zero_timer = micros();

    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;

    timer.every(20000, generate_pwm);
  
}


void loop() {
  timer.tick();

  
    //update the gyro data. this cost 10000 us

    timer_1 = micros();

    kamf();

    gyro_roll_input = agx;
    gyro_pitch_input = agy;
    gyro_yaw_input = 0;

    //update ESC data
    if (sbus.waitFrame(9))
    {
      getChannelSignal();
      processSignal(receiver_input_channel[4], n);
      n++;
      if (n > (USED_NUM - 1))n -= USED_NUM;
      //printChannelSignal();
      checkSBUS();
    }

    timer_2 = micros();
    //Serial.println(timer_2 - timer_1);

    pid_roll_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if (receiver_input_channel_filted[1] > 1505)pid_roll_setpoint = (receiver_input_channel_filted[1] - 1505) / 3.0;
    else if (receiver_input_channel_filted[1] < 1500)pid_roll_setpoint = (receiver_input_channel_filted[1] - 1500) / 3.0;

    //The PID set point in degrees per second is determined by the pitch receiver input.
    //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_pitch_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if (receiver_input_channel_filted[0] > 1503)pid_pitch_setpoint = -(receiver_input_channel_filted[0] - 1503) / 3.0;
    else if (receiver_input_channel_filted[0] < 1490)pid_pitch_setpoint = -(receiver_input_channel_filted[0] - 1490) / 3.0;

    //The PID set point in degrees per second is determined by the yaw receiver input.
    //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_yaw_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if (receiver_input_channel_filted[2] > 1100) { //Do not yaw when turning off the motors.
      if (receiver_input_channel_filted[3] > 1515)pid_yaw_setpoint = (receiver_input_channel_filted[3] - 1515) / 3.0;
      else if (receiver_input_channel_filted[3] < 1505)pid_yaw_setpoint = (receiver_input_channel_filted[3] - 1505) / 3.0;
    }
    //Serial.print(esc_1); Serial.print("\t"); Serial.print(esc_2); Serial.print("\t"); Serial.print(esc_3); Serial.print("\t"); Serial.print(esc_4); Serial.println("\t");
    
    calculate_pid();
    


/*
  timer_1 = micros();
  if (sbus.waitFrame(9))
  {
    getChannelSignal();
    processSignal(receiver_input_channel[4], n);
    n++;
    if (n > (USED_NUM - 1))n -= USED_NUM;
    //printChannelSignal();
    checkSBUS();
  }
  kamf();
  timer_2 = micros();
  Serial.println(timer_2 - timer_1);
  */
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating PID
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  pid_i_mem_roll = constrain(pid_i_mem_roll, -pid_max_roll, pid_max_roll);

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  pid_output_roll = constrain(pid_output_roll, -pid_max_roll, pid_max_roll);

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  pid_i_mem_pitch = constrain(pid_i_mem_pitch, -pid_max_pitch, pid_max_pitch);

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  pid_output_pitch = constrain(pid_output_pitch, -pid_max_pitch, pid_max_pitch);

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  pid_i_mem_yaw = constrain(pid_i_mem_yaw, -pid_max_yaw, pid_max_yaw);

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  pid_output_yaw = constrain(pid_output_yaw, -pid_max_yaw, pid_max_yaw);

  pid_last_yaw_d_error = pid_error_temp;
}
