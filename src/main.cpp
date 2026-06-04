// ============================================================
//  micro-ROS Teensy 4.0 — Super Smooth Closed Loop (PID + FF)
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

// ============================================================
// MOTOR PINS
// ============================================================
const int PWM_FL_A = 8,  PWM_FL_B = 9;
const int PWM_RL_A = 4,  PWM_RL_B = 5;
const int PWM_FR_A = 7,  PWM_FR_B = 6;
const int PWM_RR_A = 3,  PWM_RR_B = 2;

// ============================================================
// ENCODER PINS
// ============================================================
const int ENC_FL_A = 14, ENC_FL_B = 15;
const int ENC_RL_A = 16, ENC_RL_B = 17;
const int ENC_FR_A = 12, ENC_FR_B = 13;
const int ENC_RR_A = 10, ENC_RR_B = 11;

// ============================================================
// MPU6050
// ============================================================
#define MPU6050_ADDR   0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_GYRO_CFG   0x1B
#define REG_ACCEL_CFG  0x1C
#define REG_ACCEL_XOUT 0x3B
const float GYRO_SCALE  = 131.0f;
const float ACCEL_SCALE = 16384.0f;
const float DEG2RAD     = M_PI / 180.0f;
const float G_TO_MS2    = 9.80665f;
struct ImuData { float ax, ay, az, gx, gy, gz; };
bool  imu_ok      = false;
float gyro_bias_z = 0.0f;

// ============================================================
// WHEEL PARAMS
// ============================================================
const int   TICKS_PER_REV = 2848;
const float RAD_PER_TICK  = (2.0f * M_PI) / (TICKS_PER_REV * 2.0f);
const float MAX_WHEEL_VEL = 4.8f;
const int   MIN_PWM       = 35; // ความฝืดเริ่มต้นของมอเตอร์ (Deadband)

const uint32_t CMD_TIMEOUT_MS = 1000;
const float    LOOP_DT        = 0.02f;

// ============================================================
// PID PARAMETERS (แยกซ้าย-ขวา)
// ============================================================
const float KP_L = 0.5f, KI_L = 1.2f, KD_L = 0.02f;
const float KP_R = 0.4f, KI_R = 0.8f, KD_R = 0.01f;

// [UPGRADE 1 & 2] โครงสร้าง PID แบบ Smooth ลดอาการกระตุกและเสียงคราง
struct PID {
  float kp, ki, kd;
  float integral, prev_current, filtered_deriv;
  
  float compute(float target, float current, float dt) {
    float err = target - current;
    
    // I-Term: สะสมค่าชดเชย (พร้อม Anti-windup)
    integral += err * dt;
    integral = constrain(integral, -1.0f, 1.0f); 

    // D-Term: Derivative on Measurement (ป้องกัน Derivative Kick เวลารับเป้าหมายใหม่)
    float deriv = -(current - prev_current) / dt;
    prev_current = current;

    // เติม Low-pass filter ให้ D-Term ลด Noise จากเอนโค้ดเดอร์
    filtered_deriv = 0.5f * deriv + 0.5f * filtered_deriv;

    return (kp * err) + (ki * integral) + (kd * filtered_deriv);
  }
  
  void reset() { 
    integral = 0.0f; 
    prev_current = 0.0f; 
    filtered_deriv = 0.0f; 
  }
};

PID wheel_pids[4];

// ============================================================
// WHEEL NAMES
// ============================================================
const char* WHEEL_NAMES[4] = {
  "front_left_wheel_joint",  // 0: FL (Left)
  "front_right_wheel_joint", // 1: FR (Right)
  "rear_left_wheel_joint",   // 2: RL (Left)
  "rear_right_wheel_joint"   // 3: RR (Right)
};

// ============================================================
// TIMING
// ============================================================
const uint32_t PING_PERIOD_MS = 1000;
const uint32_t SYNC_PERIOD_MS = 5000;
uint32_t last_ping_ms = 0, last_sync_ms = 0, last_cmd_ms = 0;
const int PING_FAIL_LIMIT = 5;
int ping_fail_count = 0;

// ============================================================
// ENCODER
// ============================================================
volatile long ticksFL=0, ticksFR=0, ticksRL=0, ticksRR=0;
long prevFL=0, prevFR=0, prevRL=0, prevRR=0;

// ============================================================
// COMMAND + FILTER
// ============================================================
volatile float target_vel[4] = {0, 0, 0, 0};
float filtered_vel[4]        = {0, 0, 0, 0};
const float VEL_ALPHA        = 0.20f;
float held_duty[4]           = {0, 0, 0, 0};

// ============================================================
// micro-ROS
// ============================================================
rcl_subscription_t cmd_sub;
rcl_publisher_t    state_pub, imu_pub;
rcl_timer_t        timer;
sensor_msgs__msg__JointState cmd_msg, state_msg;
sensor_msgs__msg__Imu        imu_msg;
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

typedef enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } State;
State state = WAITING_AGENT;

#define RCCHECK(fn)     { rcl_ret_t rc = fn; if(rc != RCL_RET_OK) return false; }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// ============================================================
// MPU6050
// ============================================================
bool mpu_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool mpu_init() {
  Wire.begin(); Wire.setClock(400000); delay(100);
  if (!mpu_write(REG_PWR_MGMT_1, 0x00)) return false;
  delay(100);
  mpu_write(REG_GYRO_CFG, 0x00);
  mpu_write(REG_ACCEL_CFG, 0x00);
  delay(100);
  return true;
}
ImuData readIMU() {
  ImuData d = {0};
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) return d;
  if (Wire.requestFrom(MPU6050_ADDR, 14) != 14) return d;
  auto rd16 = []() -> int16_t {
    return (int16_t)((Wire.read() << 8) | Wire.read());
  };
  d.ax = rd16() / ACCEL_SCALE * G_TO_MS2;
  d.ay = rd16() / ACCEL_SCALE * G_TO_MS2;
  d.az = rd16() / ACCEL_SCALE * G_TO_MS2;
  rd16();
  d.gx = rd16() / GYRO_SCALE * DEG2RAD;
  d.gy = rd16() / GYRO_SCALE * DEG2RAD;
  d.gz = rd16() / GYRO_SCALE * DEG2RAD;
  return d;
}
void calibrateGyro() {
  float sum = 0.0f;
  for (int i = 0; i < 200; i++) { sum += readIMU().gz; delay(2); }
  gyro_bias_z = sum / 200;
}

// ============================================================
// ENCODER ISR
// ============================================================
void isrFL_A() { bool a=digitalRead(ENC_FL_A),b=digitalRead(ENC_FL_B); ticksFL+=(a==b)?-1:1; }
void isrFR_A() { bool a=digitalRead(ENC_FR_A),b=digitalRead(ENC_FR_B); ticksFR+=(a==b)? 1:-1; }
void isrRL_A() { bool a=digitalRead(ENC_RL_A),b=digitalRead(ENC_RL_B); ticksRL+=(a==b)?-1:1; }
void isrRR_A() { bool a=digitalRead(ENC_RR_A),b=digitalRead(ENC_RR_B); ticksRR+=(a==b)? 1:-1; }
void isrFL_B() { bool a=digitalRead(ENC_FL_A),b=digitalRead(ENC_FL_B); ticksFL+=(a!=b)?-1:1; }
void isrFR_B() { bool a=digitalRead(ENC_FR_A),b=digitalRead(ENC_FR_B); ticksFR+=(a!=b)? 1:-1; }
void isrRL_B() { bool a=digitalRead(ENC_RL_A),b=digitalRead(ENC_RL_B); ticksRL+=(a!=b)?-1:1; }
void isrRR_B() { bool a=digitalRead(ENC_RR_A),b=digitalRead(ENC_RR_B); ticksRR+=(a!=b)? 1:-1; }

// ============================================================
// MOTOR
// ============================================================
void setMotor(int pinA, int pinB, float duty) {
  duty = constrain(duty, -1.0f, 1.0f);
  float abs_duty = fabsf(duty);
  int pwm = 0;

  // [UPGRADE 3] Smooth Deadband Compensation (ลบขั้นบันไดออก)
  // แมพเปอร์เซ็นต์คำสั่ง 0-1 ให้เข้ากับช่วงที่มอเตอร์ขยับจริง (MIN_PWM ถึง 255) อย่างนุ่มนวล
  if (abs_duty > 0.01f) {
    float min_duty_ratio = (float)MIN_PWM / 255.0f;
    float mapped_duty = min_duty_ratio + abs_duty * (1.0f - min_duty_ratio);
    pwm = (int)(mapped_duty * 255.0f);
    if (pwm > 255) pwm = 255;
  }

  if      (duty >  0.01f) { analogWrite(pinA, pwm); analogWrite(pinB, 0);   }
  else if (duty < -0.01f) { analogWrite(pinA, 0);   analogWrite(pinB, pwm); }
  else                    { analogWrite(pinA, 0);   analogWrite(pinB, 0);   }
}

void stopAll() {
  setMotor(PWM_FL_A, PWM_FL_B, 0);
  setMotor(PWM_FR_A, PWM_FR_B, 0);
  setMotor(PWM_RL_A, PWM_RL_B, 0);
  setMotor(PWM_RR_A, PWM_RR_B, 0);
}

// ============================================================
// CMD CALLBACK
// ============================================================
void cmd_callback(const void *msgin) {
  const sensor_msgs__msg__JointState *m =
    (const sensor_msgs__msg__JointState *)msgin;
  size_t n = (m->velocity.size < 4) ? m->velocity.size : 4;
  for (size_t i = 0; i < n; i++)
    target_vel[i] = (float)m->velocity.data[i];
  last_cmd_ms = millis();
}

// ============================================================
// TIMER — 50 Hz  Closed-loop (PID + FF)
// ============================================================
void timer_callback(rcl_timer_t *, int64_t) {

  noInterrupts();
  long fl=ticksFL, fr=ticksFR, rl=ticksRL, rr=ticksRR;
  interrupts();

  bool alive = (millis() - last_cmd_ms) < CMD_TIMEOUT_MS;

  long cur[4]  = {fl, fr, rl, rr};
  long prev[4] = {prevFL, prevFR, prevRL, prevRR};
  for (int i = 0; i < 4; i++) {
    float raw = (cur[i] - prev[i]) * RAD_PER_TICK / LOOP_DT;
    filtered_vel[i] = VEL_ALPHA * raw + (1.0f - VEL_ALPHA) * filtered_vel[i];
    state_msg.position.data[i] = cur[i] * RAD_PER_TICK;
    state_msg.velocity.data[i] = filtered_vel[i];
  }
  prevFL=fl; prevFR=fr; prevRL=rl; prevRR=rr;

  // ============================================================
  // CLOSED-LOOP CONTROL (Feed-Forward + PID)
  // ============================================================
  float duty[4];
  for (int i = 0; i < 4; i++) {
    if (alive) {
      if (fabsf(target_vel[i]) < 0.01f) {
        wheel_pids[i].reset();
        duty[i] = 0.0f;
        held_duty[i] = 0.0f;
      } else {
        float ff = target_vel[i] / MAX_WHEEL_VEL;
        float pid_out = wheel_pids[i].compute(target_vel[i], filtered_vel[i], LOOP_DT);
        
        duty[i] = constrain(ff + pid_out, -1.0f, 1.0f);
        held_duty[i] = duty[i];
      }
    } else {
      // fade ลงช้าๆ รีเซ็ต PID แทนหยุดทันที ป้องกันกระตุกจาก watchdog
      wheel_pids[i].reset();
      held_duty[i] *= 0.85f;
      if (fabsf(held_duty[i]) < 0.01f) held_duty[i] = 0.0f;
      duty[i] = held_duty[i];
    }
  }

  setMotor(PWM_FL_A, PWM_FL_B, duty[0]);
  setMotor(PWM_FR_A, PWM_FR_B, duty[1]);
  setMotor(PWM_RL_A, PWM_RL_B, duty[2]);
  setMotor(PWM_RR_A, PWM_RR_B, duty[3]);

  if (state_msg.effort.size >= 4)
    for (int i = 0; i < 4; i++) state_msg.effort.data[i] = duty[i];

  // timestamp + publish
  bool synced = rmw_uros_epoch_synchronized();
  int32_t sec = 0; uint32_t nsec = 0;
  if (synced) {
    int64_t ns = rmw_uros_epoch_nanos();
    sec  = (int32_t)(ns / 1000000000LL);
    nsec = (uint32_t)(ns % 1000000000LL);
  }
  state_msg.header.stamp.sec     = sec;
  state_msg.header.stamp.nanosec = nsec;
  RCSOFTCHECK(rcl_publish(&state_pub, &state_msg, NULL));

  if (imu_ok) {
    ImuData imu = readIMU();
    imu.gz -= gyro_bias_z;
    imu_msg.header.stamp.sec     = sec;
    imu_msg.header.stamp.nanosec = nsec;
    imu_msg.orientation_covariance[0] = -1.0;
    imu_msg.angular_velocity.x = imu.gx;
    imu_msg.angular_velocity.y = imu.gy;
    imu_msg.angular_velocity.z = imu.gz;
    imu_msg.angular_velocity_covariance[0] = 0.0025;
    imu_msg.angular_velocity_covariance[4] = 0.0025;
    imu_msg.angular_velocity_covariance[8] = 0.0025;
    imu_msg.linear_acceleration.x = imu.ax;
    imu_msg.linear_acceleration.y = imu.ay;
    imu_msg.linear_acceleration.z = imu.az;
    imu_msg.linear_acceleration_covariance[0] = 0.09;
    imu_msg.linear_acceleration_covariance[4] = 0.09;
    imu_msg.linear_acceleration_covariance[8] = 0.09;
    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
  }
}

// ============================================================
// CREATE / DESTROY ENTITIES
// ============================================================
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_node", "", &support));
  rmw_uros_sync_session(1000);
  last_sync_ms = millis();

  RCCHECK(rclc_subscription_init_default(
    &cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "wheel_commands"));
  RCCHECK(rclc_publisher_init_default(
    &state_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "wheel_states"));
  RCCHECK(rclc_publisher_init_default(
    &imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw"));

  micro_ros_utilities_memory_conf_t conf = {};
  conf.max_string_capacity              = 32;
  conf.max_ros2_type_sequence_capacity  = 4;
  conf.max_basic_type_sequence_capacity = 4;
  micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), &cmd_msg, conf);
  micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), &state_msg, conf);

  state_msg.name.size = state_msg.position.size =
  state_msg.velocity.size = state_msg.effort.size = 4;
  for (int i = 0; i < 4; i++)
    state_msg.name.data[i] =
      micro_ros_string_utilities_set(state_msg.name.data[i], WHEEL_NAMES[i]);

  static char imu_frame[] = "imu_link";
  imu_msg.header.frame_id.data = imu_frame;
  imu_msg.header.frame_id.size = strlen(imu_frame);

  for (int i = 0; i < 4; i++) {
    filtered_vel[i] = 0.0f;
    held_duty[i]    = 0.0f;
    wheel_pids[i].reset();
  }

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_sub, &cmd_msg, &cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
  rcl_subscription_fini(&cmd_sub, &node);
  rcl_publisher_fini(&state_pub, &node);
  rcl_publisher_fini(&imu_pub, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(921600);
  set_microros_transports();

  int mpins[8] = {PWM_FL_A,PWM_FL_B,PWM_FR_A,PWM_FR_B,
                  PWM_RL_A,PWM_RL_B,PWM_RR_A,PWM_RR_B};
  for (int i=0;i<8;i++) pinMode(mpins[i], OUTPUT);

  int epins[8] = {ENC_FL_A,ENC_FL_B,ENC_FR_A,ENC_FR_B,
                  ENC_RL_A,ENC_RL_B,ENC_RR_A,ENC_RR_B};
  for (int i=0;i<8;i++) pinMode(epins[i], INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isrFL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_B), isrFL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isrFR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_B), isrFR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_A), isrRL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_B), isrRL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_A), isrRR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_B), isrRR_B, CHANGE);

  // กำหนด PID ให้ล้อแต่ละฝั่ง
  wheel_pids[0] = {KP_L, KI_L, KD_L, 0, 0, 0}; // 0: FL (ซ้าย)
  wheel_pids[1] = {KP_R, KI_R, KD_R, 0, 0, 0}; // 1: FR (ขวา)
  wheel_pids[2] = {KP_L, KI_L, KD_L, 0, 0, 0}; // 2: RL (ซ้าย)
  wheel_pids[3] = {KP_R, KI_R, KD_R, 0, 0, 0}; // 3: RR (ขวา)

  stopAll();
  delay(500);
  imu_ok = mpu_init();
  if (imu_ok) { delay(500); calibrateGyro(); }
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      if (rmw_uros_ping_agent(500, 1) == RMW_RET_OK)
        state = AGENT_AVAILABLE;
      break;
    case AGENT_AVAILABLE:
      if (create_entities()) {
        ping_fail_count = 0;
        last_ping_ms = millis();
        state = AGENT_CONNECTED;
      } else {
        destroy_entities();
        state = WAITING_AGENT;
      }
      break;
    case AGENT_CONNECTED: {
      uint32_t now = millis();
      if (now - last_ping_ms > PING_PERIOD_MS) {
        last_ping_ms = now;
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          if (++ping_fail_count >= PING_FAIL_LIMIT) {
            ping_fail_count = 0;
            state = AGENT_DISCONNECTED;
            break;
          }
        } else { ping_fail_count = 0; }
      }
      if (now - last_sync_ms > SYNC_PERIOD_MS) {
        last_sync_ms = now;
        rmw_uros_sync_session(100);
      }
      RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
      break;
    }
    case AGENT_DISCONNECTED:
      stopAll();
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}