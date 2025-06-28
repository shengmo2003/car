//导入库文件
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// 定义引脚和常量
#define PWMA 6  //左轮转速PWM信号
#define AIN1 8  //左轮方向控制信号
#define AIN2 7  //左轮方向控制信号
#define PWMB 10  //右轮转速PWM信号
#define BIN1 11  //右轮方向控制信号
#define BIN2 12  //右轮方向控制信号
#define STBY 13  //TB6612启动标志位
#define ENCODER_A_LEFT 2  // 左轮编码器A信号
#define ENCODER_B_LEFT 4  // 左轮编码器B信号 
#define ENCODER_A_RIGHT 3 // 右轮编码器A信号
#define ENCODER_B_RIGHT 5 // 右轮编码器B信号
#define WHEEL_BASE 0.13f    // 小车轮距(单位：米)
#define WHEEL_RADIUS 0.024f // 车轮半径(单位：米)
#define ENCODER_CPR 260     // 编码器每转脉冲数


// 定义变量
// 时间变量
static unsigned long prevTime = 0;
// 卡尔曼滤波相关变量
float Q_angle = 0.001;    // 角度过程噪声协方差
float Q_bias = 0.003;     // 偏差过程噪声协方差
float R_measure = 0.03;   // 测量噪声协方差
// mpu6050静止偏置值
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float accX_offset = 0, accY_offset = 0, accZ_offset = 0;
// 编码器相关变量
volatile int encoderCountLeft = 0; // 左轮编码器计数
volatile int encoderCountRight = 0; // 右轮编码器计数
int prevEncoderLeft = 0;
int prevEncoderRight = 0;
float theta = 0.0;
float posX = 0.0;
float posY = 0.0;

// PID参数，用于稳定控制，具体值可以根据实际情况调整
struct {
  float kp = 3, ki = 0.2, kd = 1;  // 角度PID
  float speed_kp = 0.8;                  // 转速比例系数
} pidParams;

// 补偿系统，用以校正左右轮机械结构差异产生的速度差
struct {
  float left = 1.00;     // 长期左轮补偿
  float right = 0.95;    // 长期右轮补偿
  float dynamicAdj = 0; // 动态调整量
} motorComp;

struct Kalman {
  float angle;    // 滤波器输出的角度
  float bias;     // 陀螺仪偏差
  float P[2][2];  // 误差协方差矩阵
};

// 分别为 Pitch 和 Roll 设置卡尔曼滤波器
Kalman kalmanPitch = {0, 0, {{0, 0}, {0, 0}}};
Kalman kalmanRoll = {0, 0, {{0, 0}, {0, 0}}};

typedef struct {
    float q; // 过程噪声协方差
    float r; // 测量噪声协方差
    float x; // 估计值
    float p; // 估计误差协方差
    float k; // 卡尔曼增益
} kalman_filter_t;

void kalman_init(kalman_filter_t *kf, float q, float r, float initial_value, float initial_p) {
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = initial_p;
    kf->k = 0;
}

float kalman_update(kalman_filter_t *kf, float measurement) {
    // 预测步骤
    kf->p = kf->p + kf->q;
    
    // 更新步骤
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1 - kf->k) * kf->p;
    
    return kf->x;
}

// 对MPU6050的三轴数据分别应用卡尔曼滤波
kalman_filter_t kf_x, kf_y, kf_z;

// 存储传感器数据
struct MPU_SensorData {
  //局部坐标系
  float accX_body, accY_body, accZ_body;    // 加速度
  float gyroX_body, gyroY_body, gyroZ_body; // 角速度
  //全局坐标系
  float global_accX, global_accY;
  float velocityX, velocityY, posX, posY;
  //倾角
  float pitch, roll, yaw;         
} mpu_sensorData;

//储存编码器数据
struct Encoder_SensorData{
  float speedLeft, speedRight, linearSpeed;
  float yaw, posX, posY;
  float displacementLeft, displacementRight, displacement;
} encoder_sensorData;

//最终控制数据
struct Controll_SensorData{
float yaw, displacement;
} controll_sensorData;

//左轮编码器计数函数
void countEncoderLeft() {
  if (digitalRead(ENCODER_A_LEFT) == HIGH) {
    if (digitalRead(ENCODER_B_LEFT) == LOW) {
      encoderCountLeft--;
    } else {
      encoderCountLeft++;
    }
  }
}

//右轮编码器计数函数
void countEncoderRight() {
  if (digitalRead(ENCODER_A_RIGHT) == HIGH) {
    if (digitalRead(ENCODER_B_RIGHT) == LOW) {
      encoderCountRight++;
    } else {
      encoderCountRight--;
    }
  }
}

void setup() {
  //初始化三轴卡尔曼滤波器
  kalman_init(&kf_x, 0.01, 0.1, 0, 1);
  kalman_init(&kf_y, 0.01, 0.1, 0, 1);
  kalman_init(&kf_z, 0.01, 0.1, 0, 1);
  //mpu6050初始化及校准
  Serial.begin(9600);
  if (!mpu.begin()) {
    Serial.println("无法初始化 MPU6050");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 初始化成功！");
  Serial.println("正在校准陀螺仪...");
  for (int i = 0; i < 100; i++) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    gyroX_offset += gyro.gyro.x;
    gyroY_offset += gyro.gyro.y;
    gyroZ_offset += gyro.gyro.z;
    accX_offset += accel.acceleration.x;
    accY_offset += accel.acceleration.y;
    accZ_offset += accel.acceleration.z - 9.8; // Z轴期望9.8
    delay(10);
  }
  gyroX_offset /= 100; gyroY_offset /= 100; gyroZ_offset /= 100;
  accX_offset /= 100; accY_offset /= 100; accZ_offset /= 100;
  Serial.println("校准完成！");

  //初始化电机和编码器
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(ENCODER_A_LEFT, INPUT);
  pinMode(ENCODER_B_LEFT, INPUT);
  pinMode(ENCODER_A_RIGHT, INPUT);
  pinMode(ENCODER_B_RIGHT, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), countEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), countEncoderRight, CHANGE);

  digitalWrite(STBY, HIGH); // 启动TB6612
  Serial.println("初始化完成！");
}

//用于数据融合的卡尔曼滤波器函数
float kalmanFilter(float newAngle, float newRate, float dt, Kalman &kalman) {
  // 预测阶段
  kalman.angle += dt * (newRate - kalman.bias);
  kalman.P[0][0] += dt * (dt * kalman.P[1][1] - kalman.P[0][1] - kalman.P[1][0] + Q_angle);
  kalman.P[0][1] -= dt * kalman.P[1][1];
  kalman.P[1][0] -= dt * kalman.P[1][1];
  kalman.P[1][1] += Q_bias * dt;

  // 更新阶段
  float S = kalman.P[0][0] + R_measure;
  float K[2]; // 卡尔曼增益
  K[0] = kalman.P[0][0] / S;
  K[1] = kalman.P[1][0] / S;

  float y = newAngle - kalman.angle; // 残差
  kalman.angle += K[0] * y;
  kalman.bias += K[1] * y;

  float P00_temp = kalman.P[0][0];
  float P01_temp = kalman.P[0][1];

  kalman.P[0][0] -= K[0] * P00_temp;
  kalman.P[0][1] -= K[0] * P01_temp;
  kalman.P[1][0] -= K[1] * P00_temp;
  kalman.P[1][1] -= K[1] * P01_temp;

  return kalman.angle;
}



void loop(){
//这里的主循环是控制小车走矩形
 GOstraight(0.5);
 RIGHT(-85.0);
 GOstraight(0.5);
 RIGHT(-85.0);
 GOstraight(0.5);
 RIGHT(-85.0);
 GOstraight(0.5);
 stop();
 while(1);
}



//直行控制函数
void moveStraight(int leftSpeed,int rightSpeed){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed);
}

//后退控制函数
void moveBack(int leftSpeed,int rightSpeed){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed);
}

//右转控制函数
void moveRight(int leftSpeed,int rightSpeed){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed);
}

//左转控制函数
void moveLeft(int leftSpeed,int rightSpeed){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed);
}

//停止函数，可以不写，直接用TB6612控制引脚来实现也可以
void stop(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

//控制小车直行任意距离
void GOstraight(float displacement){
controll_sensorData.displacement = 0.0;
encoderCountLeft = 0;
encoderCountRight = 0;
mpu_sensorData.yaw = 0.0;
  // 主循环（直到行驶1米）
  while ( controll_sensorData.displacement < displacement) {
  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0; // 时间间隔
  prevTime = currentTime;
  float alpha = 0.2, a = 0.90; // 滤波系数（0~1，越小越平滑）
  float last_global_accX = 0;
  float last_global_accY = 0;
  int currentEncoderLeft = encoderCountLeft;
  int currentEncoderRight = encoderCountRight;

  // 计算左轮和右轮速度
  encoder_sensorData.speedLeft = (currentEncoderLeft - prevEncoderLeft) / 260.0 * 2 * PI * 0.024 / dt;
  encoder_sensorData.speedRight = (currentEncoderRight - prevEncoderRight) / 260.0 * 2 * PI * 0.024 / dt;

  // 计算位移
  encoder_sensorData.displacementLeft = currentEncoderLeft / 260.0 * 2 * PI * 0.024;
  encoder_sensorData.displacementRight = currentEncoderRight / 260.0 * 2 * PI * 0.024;
  controll_sensorData.displacement = (encoder_sensorData.displacementLeft + encoder_sensorData.displacementRight) / 2.0;
  // 更新方向和位置
  encoder_sensorData.linearSpeed = (encoder_sensorData.speedLeft + encoder_sensorData.speedRight) / 2.0;
  float angularSpeed = (encoder_sensorData.speedRight - encoder_sensorData.speedLeft) / 0.13;
  encoder_sensorData.yaw += angularSpeed * dt;
  encoder_sensorData.posX += encoder_sensorData.linearSpeed * cos(encoder_sensorData.yaw) * dt;
  encoder_sensorData.posY += encoder_sensorData.linearSpeed * sin(encoder_sensorData.yaw) * dt;
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // 加速度数据
  mpu_sensorData.accX_body = accel.acceleration.x - accX_offset;
  mpu_sensorData.accY_body = accel.acceleration.y - accY_offset;
  mpu_sensorData.accZ_body = accel.acceleration.z - accZ_offset;

  // 角速度数据
  mpu_sensorData.gyroX_body = gyro.gyro.x * 180 / PI - gyroX_offset * 180 / PI ; // 转换为度/秒
  mpu_sensorData.gyroY_body = gyro.gyro.y * 180 / PI - gyroY_offset * 180 / PI ;
  mpu_sensorData.gyroZ_body = gyro.gyro.z * 180 / PI - gyroZ_offset * 180 / PI ;

  // 应用卡尔曼滤波器去除噪声
  mpu_sensorData.accX_body = kalman_update(&kf_x, mpu_sensorData.accX_body);
  mpu_sensorData.accY_body = kalman_update(&kf_y, mpu_sensorData.accY_body);
  mpu_sensorData.accZ_body = kalman_update(&kf_z, mpu_sensorData.accZ_body);
  //mpu_sensorData.gyroX_body = kalman_update(&kf_x, mpu_sensorData.gyroX_body);
  //mpu_sensorData.gyroY_body = kalman_update(&kf_y, mpu_sensorData.gyroY_body);
  //mpu_sensorData.gyroZ_body = kalman_update(&kf_z, mpu_sensorData.gyroZ_body);

  // 计算加速度倾角
  float accPitch = atan2(mpu_sensorData.accY_body, mpu_sensorData.accZ_body) * 180 / PI;
  float accRoll = atan2(-mpu_sensorData.accX_body, sqrt(mpu_sensorData.accY_body * mpu_sensorData.accY_body + mpu_sensorData.accZ_body * mpu_sensorData.accZ_body)) * 180 / PI;

  // 应用卡尔曼滤波器融合数据
  mpu_sensorData.pitch = kalmanFilter(accPitch, mpu_sensorData.gyroX_body, dt, kalmanPitch);
  mpu_sensorData.roll = kalmanFilter(accRoll, mpu_sensorData.gyroY_body, dt, kalmanRoll);
  mpu_sensorData.yaw += mpu_sensorData.gyroZ_body * dt;

  // 计算运动加速度
  float pitch_rad = mpu_sensorData.pitch * PI / 180;
  float roll_rad = mpu_sensorData.roll * PI / 180;
  
  // 重力分量
  float gravity_x = sin(roll_rad) * 9.8;
  float gravity_y = -sin(pitch_rad) * 9.8;
  float gravity_z = cos(pitch_rad) * cos(roll_rad) * 9.8;

  // 运动加速度（局部坐标系）
  float motion_accX = mpu_sensorData.accX_body - gravity_x;
  float motion_accY = mpu_sensorData.accY_body - gravity_y;
  float motion_accZ = mpu_sensorData.accZ_body - gravity_z;

  // 运动加速度（全局坐标系）
  mpu_sensorData.global_accX = motion_accX * cos(roll_rad) + motion_accZ * sin(roll_rad);
  mpu_sensorData.global_accY = motion_accY * cos(pitch_rad) + motion_accZ * sin(pitch_rad);
  mpu_sensorData.global_accX = alpha * mpu_sensorData.global_accX + (1 - alpha) * last_global_accX;
  mpu_sensorData.global_accY = alpha * mpu_sensorData.global_accY + (1 - alpha) * last_global_accY;
  last_global_accX = mpu_sensorData.global_accX;
  last_global_accY = mpu_sensorData.global_accX;
  // 更新速度和位置（使用全局加速度）
  mpu_sensorData.velocityX += mpu_sensorData.global_accX * dt;
  mpu_sensorData.velocityY += mpu_sensorData.global_accY * dt;
  mpu_sensorData.posX += mpu_sensorData.velocityX * dt;
  mpu_sensorData.posY += mpu_sensorData.velocityY * dt;
    
  // 如果检测到小车停止，将速度归零
  if (fabs(mpu_sensorData.global_accX) < 0.1 && fabs(mpu_sensorData.global_accY) < 0.1) { // 阈值根据实际情况调整
        mpu_sensorData.velocityX = 0;
        mpu_sensorData.velocityY = 0;
  }

        // 更新历史值
  prevTime = currentTime;
  prevEncoderLeft = currentEncoderLeft;
  prevEncoderRight = currentEncoderRight;



  float fusedYaw = a * mpu_sensorData.yaw + (1 - a) * encoder_sensorData.yaw;

  float targetSpeed = 70;  // 基础PWM值，可以根据调试结果选择合适值
  
  // 1. 角度PID计算
  float angleError = -fusedYaw;  // 假设期望角度为0
  static float angleIntegral = 0, lastAngleError = 0;
  
  angleIntegral += angleError;
  float angleDerivative = angleError - lastAngleError;
  lastAngleError = angleError;
  
  float angleCorrection = pidParams.kp * angleError + 
                         pidParams.ki * angleIntegral + 
                         pidParams.kd * angleDerivative;

  // 2. 转速差补偿
  float speedError = (encoder_sensorData.speedLeft - encoder_sensorData.speedRight) / 2 / PI / 0.024;
  float speedCorrection = pidParams.speed_kp * speedError;
  
  // 3. 综合补偿（动态部分）
  motorComp.dynamicAdj = angleCorrection + speedCorrection;
  
  // 4. 应用补偿（静态+动态）
  int leftSpeed = targetSpeed * motorComp.left - motorComp.dynamicAdj;
  int rightSpeed = targetSpeed * motorComp.right + motorComp.dynamicAdj;
  leftSpeed = abs(leftSpeed);
  rightSpeed = abs(rightSpeed);
  // 限幅保护
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  moveStraight(leftSpeed,rightSpeed); 
  delay(10); // 控制周期（单位：ms）
  }

  // 到达目标距离后停止
  stop();
  delay(150);
}


//控制小车旋转任意角度，建议自变量在正负0到90度以内，默认为右转
void RIGHT(float targetYaw){
controll_sensorData.displacement = 0.0;
encoderCountLeft = 0;
encoderCountRight = 0;
mpu_sensorData.yaw = 0.0;
float fusedYaw = 0.0;
float angleError = targetYaw - fusedYaw;
  while ( fabs(angleError) > 10.0f) {
  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0; // 时间间隔
  prevTime = currentTime;
  float alpha = 0.2, a = 0.90; // 滤波系数（0~1，越小越平滑）
  float last_global_accX = 0;
  float last_global_accY = 0;
  int currentEncoderLeft = encoderCountLeft;
  int currentEncoderRight = encoderCountRight;

  // 计算左轮和右轮速度
  encoder_sensorData.speedLeft = (currentEncoderLeft - prevEncoderLeft) / 260.0 * 2 * PI * 0.024 / dt;
  encoder_sensorData.speedRight = (currentEncoderRight - prevEncoderRight) / 260.0 * 2 * PI * 0.024 / dt;

  // 计算位移
  encoder_sensorData.displacementLeft = currentEncoderLeft / 260.0 * 2 * PI * 0.024;
  encoder_sensorData.displacementRight = currentEncoderRight / 260.0 * 2 * PI * 0.024;
  controll_sensorData.displacement = (encoder_sensorData.displacementLeft + encoder_sensorData.displacementRight) / 2.0;
  // 更新方向和位置
  encoder_sensorData.linearSpeed = (encoder_sensorData.speedLeft + encoder_sensorData.speedRight) / 2.0;
  float angularSpeed = (encoder_sensorData.speedRight - encoder_sensorData.speedLeft) / 0.13;
  encoder_sensorData.yaw += angularSpeed * dt;
  encoder_sensorData.posX += encoder_sensorData.linearSpeed * cos(encoder_sensorData.yaw) * dt;
  encoder_sensorData.posY += encoder_sensorData.linearSpeed * sin(encoder_sensorData.yaw) * dt;
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // 加速度数据
  mpu_sensorData.accX_body = accel.acceleration.x - accX_offset;
  mpu_sensorData.accY_body = accel.acceleration.y - accY_offset;
  mpu_sensorData.accZ_body = accel.acceleration.z - accZ_offset;

  // 角速度数据
  mpu_sensorData.gyroX_body = gyro.gyro.x * 180 / PI - gyroX_offset * 180 / PI ; // 转换为度/秒
  mpu_sensorData.gyroY_body = gyro.gyro.y * 180 / PI - gyroY_offset * 180 / PI ;
  mpu_sensorData.gyroZ_body = gyro.gyro.z * 180 / PI - gyroZ_offset * 180 / PI ;

  // 应用卡尔曼滤波
  mpu_sensorData.accX_body = kalman_update(&kf_x, mpu_sensorData.accX_body);
  mpu_sensorData.accY_body = kalman_update(&kf_y, mpu_sensorData.accY_body);
  mpu_sensorData.accZ_body = kalman_update(&kf_z, mpu_sensorData.accZ_body);
  //mpu_sensorData.gyroX_body = kalman_update(&kf_x, mpu_sensorData.gyroX_body);
  //mpu_sensorData.gyroY_body = kalman_update(&kf_y, mpu_sensorData.gyroY_body);
  //mpu_sensorData.gyroZ_body = kalman_update(&kf_z, mpu_sensorData.gyroZ_body);

  // 计算加速度倾角
  float accPitch = atan2(mpu_sensorData.accY_body, mpu_sensorData.accZ_body) * 180 / PI;
  float accRoll = atan2(-mpu_sensorData.accX_body, sqrt(mpu_sensorData.accY_body * mpu_sensorData.accY_body + mpu_sensorData.accZ_body * mpu_sensorData.accZ_body)) * 180 / PI;

  // 应用卡尔曼滤波
  mpu_sensorData.pitch = kalmanFilter(accPitch, mpu_sensorData.gyroX_body, dt, kalmanPitch);
  mpu_sensorData.roll = kalmanFilter(accRoll, mpu_sensorData.gyroY_body, dt, kalmanRoll);
  mpu_sensorData.yaw += mpu_sensorData.gyroZ_body * dt;

  // 计算运动加速度
  float pitch_rad = mpu_sensorData.pitch * PI / 180;
  float roll_rad = mpu_sensorData.roll * PI / 180;
  
  // 重力分量
  float gravity_x = sin(roll_rad) * 9.8;
  float gravity_y = -sin(pitch_rad) * 9.8;
  float gravity_z = cos(pitch_rad) * cos(roll_rad) * 9.8;

  // 运动加速度（局部坐标系）
  float motion_accX = mpu_sensorData.accX_body - gravity_x;
  float motion_accY = mpu_sensorData.accY_body - gravity_y;
  float motion_accZ = mpu_sensorData.accZ_body - gravity_z;

  // 运动加速度（全局坐标系）
  mpu_sensorData.global_accX = motion_accX * cos(roll_rad) + motion_accZ * sin(roll_rad);
  mpu_sensorData.global_accY = motion_accY * cos(pitch_rad) + motion_accZ * sin(pitch_rad);
  mpu_sensorData.global_accX = alpha * mpu_sensorData.global_accX + (1 - alpha) * last_global_accX;
  mpu_sensorData.global_accY = alpha * mpu_sensorData.global_accY + (1 - alpha) * last_global_accY;
  last_global_accX = mpu_sensorData.global_accX;
  last_global_accY = mpu_sensorData.global_accX;
  // 更新速度和位置（使用全局加速度）
  mpu_sensorData.velocityX += mpu_sensorData.global_accX * dt;
  mpu_sensorData.velocityY += mpu_sensorData.global_accY * dt;
  mpu_sensorData.posX += mpu_sensorData.velocityX * dt;
  mpu_sensorData.posY += mpu_sensorData.velocityY * dt;
    
  // 如果检测到小车停止，将速度归零
  if (fabs(mpu_sensorData.global_accX) < 0.1 && fabs(mpu_sensorData.global_accY) < 0.1) { // 阈值根据实际情况调整
        mpu_sensorData.velocityX = 0;
        mpu_sensorData.velocityY = 0;
  }

        // 更新历史值
  prevTime = currentTime;
  prevEncoderLeft = currentEncoderLeft;
  prevEncoderRight = currentEncoderRight;


  fusedYaw = a * mpu_sensorData.yaw + (1 - a) * encoder_sensorData.yaw;

  // 转弯模式控制
  angleError = targetYaw - fusedYaw;
  
  // 处理角度跨越±180°的情况
  if (angleError > 180) angleError -= 360;
  if (angleError < -180) angleError += 360;

  // PD控制（去掉积分项避免超调）
  static float lastAngleError = 0;
  float angleOutput = 0.3 * angleError + 
                     5 * (angleError - lastAngleError);
  lastAngleError = angleError;

  // 设置电机速度（左右轮反向）
  int baseTurnSpeed = 60; // 基础转弯速度（根据实际调整）

 // if (fabs(angleError) < BRAKE_ANGLE) {
    
  // 进入减速区间：速度随角度差线性减小
   // baseTurnSpeed = map(fabs(angleError), 0, BRAKE_ANGLE, 40, 80);
  
//}
  int leftSpeed = baseTurnSpeed - angleOutput;
  int rightSpeed = -baseTurnSpeed - angleOutput;
  leftSpeed = abs(leftSpeed);
  rightSpeed = abs(rightSpeed);
  // 限幅保护
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // 执行转弯（需要修改moveStraight为moveTurn）
  moveRight(leftSpeed, rightSpeed);
   delay(10);
  }

  // 到达目标距离后停止
  stop();
   delay(150);
}

