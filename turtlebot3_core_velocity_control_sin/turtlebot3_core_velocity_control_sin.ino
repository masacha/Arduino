/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#include "turtlebot3_core_config.h"

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<std_msgs::Float64> cmd_velocity_left_sub("cmd_velocity_left", commandVelocityLeftCallback);
ros::Subscriber<std_msgs::Float64> cmd_velocity_right_sub("cmd_velocity_right", commandVelocityRightCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

std_msgs::Float64 angular_velocity_left;
ros::Publisher angular_velocity_left_pub("angular_velocity_left",&angular_velocity_left);

std_msgs::Float64 angular_velocity_right;
ros::Publisher angular_velocity_right_pub("angular_velocity_right",&angular_velocity_right);

std_msgs::Float64 angular_acceleration_left;
ros::Publisher angular_acceleration_left_pub("angular_acceleration_left",&angular_acceleration_left);

std_msgs::Float64 angular_acceleration_right;
ros::Publisher angular_acceleration_right_pub("angular_acceleration_right",&angular_acceleration_right);

std_msgs::Float64 motor_velocity_left;
ros::Publisher motor_velocity_left_pub("motor_velocity_left",&motor_velocity_left);

std_msgs::Float64 motor_velocity_right;
ros::Publisher motor_velocity_right_pub("motor_velocity_right",&motor_velocity_right);

std_msgs::Float64 current_left;
ros::Publisher current_left_pub("current_left",&current_left);

std_msgs::Float64 current_right;
ros::Publisher current_right_pub("current_right",&current_right);

std_msgs::Float64 disturbance_torque_left;
ros::Publisher disturbance_torque_left_pub("disturbance_torque_left",&disturbance_torque_left);

std_msgs::Float64 disturbance_torque_right;
ros::Publisher disturbance_torque_right_pub("disturbance_torque_right",&disturbance_torque_right);

std_msgs::Float64 reaction_torque_left;
ros::Publisher reaction_torque_left_pub("reaction_torque_left",&reaction_torque_left);

std_msgs::Float64 reaction_torque_right;
ros::Publisher reaction_torque_right_pub("reaction_torque_right",&reaction_torque_right);

std_msgs::Float64 reaction_force;
ros::Publisher reaction_force_pub("reaction_force",&reaction_force);

std_msgs::Float64 compensation_voltage_left;
ros::Publisher compensation_voltage_left_pub("compensation_voltage_left",&compensation_voltage_left);

std_msgs::Float64 compensation_voltage_right;
ros::Publisher compensation_voltage_right_pub("compensation_voltage_right",&compensation_voltage_right);

std_msgs::Float64 motor_voltage_left;
ros::Publisher motor_voltage_left_pub("motor_voltage_left",&motor_voltage_left);

std_msgs::Float64 motor_voltage_right;
ros::Publisher motor_voltage_right_pub("motor_voltage_right",&motor_voltage_right);

std_msgs::Float64 error_left;
ros::Publisher error_left_pub("error_left",&error_left);

std_msgs::Float64 error_right;
ros::Publisher error_right_pub("error_right",&error_right);

std_msgs::Float64 yaw;
ros::Publisher yaw_pub("yaw",&yaw);
/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped tfs_msg;
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tfbroadcaster;

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[4];

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;
bool init_encoder_[2]  = {false, false};
int32_t last_diff_tick_[2];
int32_t last_tick_[2];
double last_rad_[2];
double last_velocity_[2];
double last_velocity_filtered_[2];
double motor_velocity_[2];
int64_t goal_pwm_left  = 0;
int64_t goal_pwm_right = 0;
int64_t cmd_pwm_left  = 0;
int64_t cmd_pwm_right = 0;
double cmd_velocity_left  = 0;
double cmd_velocity_right = 0;
double current_[2];
double current_filtered_[2];
double motor_voltage_[2];
double compensation_voltage_[2];
double disturbance_torque_[2];
double disturbance_torque_tmp_[2];
double disturbance_torque_filtered_[2];
double reaction_torque_[2];
double reaction_torque_tmp_[2];
double reaction_torque_filtered_[2];
double mobile_robot_reaction_force;
double acc_tmp_[2];
double acc_filtered_[2];
double p_gain = 50.0;
double i_gain = 1000.0;
double d_gain = 2.5;
double pwm_gain = 1.0;
double control_output_left = 0.0;
double control_output_right = 0.0;
double velocity_error_left = 0.0;
double velocity_error_right = 0.0;
double pre_error_left = 0.0;
double pre_error_right = 0.0;
double integral_left = 0.0;
double integral_right = 0.0;
double derivative_left = 0.0;
double derivative_right = 0.0;
char keyboard;

/*******************************************************************************
* Declaration for IMU
*******************************************************************************/
cIMU imu;

/*******************************************************************************
* Declaration for test drive
*******************************************************************************/
bool start_move = false;
bool start_rotate = false;
int32_t last_left_encoder  = 0;
int32_t last_right_encoder = 0;
/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
char *joint_states_name[] = {"wheel_left_joint", "wheel_right_joint"};
float joint_states_pos[2] = {0.0, 0.0};
float joint_states_vel[2] = {0.0, 0.0};
float joint_states_eff[2] = {0.0, 0.0};

/*******************************************************************************
* Declaration for LED
*******************************************************************************/
#define LED_TXD         0
#define LED_RXD         1
#define LED_LOW_BATTERY 2
#define LED_ROS_CONNECT 3

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
#define BATTERY_POWER_OFF             0
#define BATTERY_POWER_STARTUP         1
#define BATTERY_POWER_NORMAL          2
#define BATTERY_POWER_CHECK           3
#define BATTERY_POWER_WARNNING        4

static bool    setup_end       = false;
static uint8_t battery_voltage = 0;
static float   battery_valtage_raw = 0;
static uint8_t battery_state   = BATTERY_POWER_OFF;

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(cmd_velocity_left_sub);
  nh.subscribe(cmd_velocity_right_sub);
  nh.advertise(sensor_state_pub);
  nh.advertise(imu_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(angular_velocity_left_pub);
  nh.advertise(angular_velocity_right_pub);
  nh.advertise(angular_acceleration_left_pub);
  nh.advertise(angular_acceleration_right_pub);
  nh.advertise(motor_velocity_left_pub);
  nh.advertise(motor_velocity_right_pub);
  nh.advertise(current_left_pub);
  nh.advertise(current_right_pub);
  nh.advertise(disturbance_torque_left_pub);
  nh.advertise(disturbance_torque_right_pub);
  nh.advertise(motor_voltage_left_pub);
  nh.advertise(motor_voltage_right_pub);
  nh.advertise(compensation_voltage_left_pub);
  nh.advertise(compensation_voltage_right_pub);
  nh.advertise(reaction_torque_left_pub);
  nh.advertise(reaction_torque_right_pub);
  nh.advertise(reaction_force_pub);
  nh.advertise(error_left_pub);
  nh.advertise(error_right_pub);
  nh.advertise(yaw_pub);
  
  tfbroadcaster.init(nh);

  nh.loginfo("Connected to OpenCR board!");

  // Setting for Dynamixel motors
  motor_driver.init();

  // Setting for IMU
  imu.begin();

  // Setting for SLAM and navigation (odometry, joint states, TF)
  odom_pose[0] = 0.0;
  odom_pose[1] = 0.0;
  odom_pose[2] = 0.0;

  last_velocity_filtered_[LEFT] = 0.0;
  last_velocity_filtered_[RIGHT] = 0.0;

  acc_tmp_[LEFT] = 0.0;
  acc_tmp_[RIGHT] = 0.0;

  acc_filtered_[LEFT] = 0.0;
  acc_filtered_[RIGHT] = 0.0;

  motor_velocity_[LEFT]=0.0;
  motor_velocity_[RIGHT]=0.0;

  current_filtered_[LEFT] = 0.0;
  current_filtered_[RIGHT] = 0.0;

  disturbance_torque_tmp_[LEFT] = 0.0;
  disturbance_torque_tmp_[RIGHT] = 0.0;
  
  disturbance_torque_filtered_[LEFT] = 0.0;
  disturbance_torque_filtered_[RIGHT] = 0.0;

  reaction_torque_[LEFT] = 0.0;
  reaction_torque_[RIGHT] = 0.0;

  reaction_torque_tmp_[LEFT] = 0.0;
  reaction_torque_tmp_[RIGHT] = 0.0;
  
  reaction_torque_filtered_[LEFT] = 0.0;
  reaction_torque_filtered_[RIGHT] = 0.0;

  mobile_robot_reaction_force = 0.0;

  compensation_voltage_[LEFT]=0.0;
  compensation_voltage_[RIGHT]=0.0;

  joint_states.header.frame_id = "base_footprint";
  joint_states.name            = joint_states_name;

  joint_states.name_length     = 2;
  joint_states.position_length = 2;
  joint_states.velocity_length = 2;
  joint_states.effort_length   = 2;

  prev_update_time = millis();

  pinMode(13, OUTPUT);

  SerialBT2.begin(57600);

  setup_end = true;

  char log_msg2[50];
  
//  if (motor_driver.lmao())
//  {
//    sprintf(log_msg2, "Changing Operating Mode : Success");
//    nh.loginfo(log_msg2);
//  }
//  else
//  {
//    sprintf(log_msg2, "Failed to Change Operating Mode");
//    nh.loginfo(log_msg2);
//  }
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  if ((millis()-tTime[0]) >= (1000 / CONTROL_MOTOR_PWM_PERIOD))
  {
    controlMotorPwm();
    tTime[0] = millis();
  }

  if ((millis()-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD))
  {
    publishSensorStateMsg();
    publishDriveInformation();
    tTime[2] = millis();
  }

  if ((millis()-tTime[3]) >= (1000 / IMU_PUBLISH_PERIOD))
  {
    publishImuMsg();
    tTime[3] = millis();
  }
  
  // Update the IMU unit
  imu.update();

  // Start Gyro Calibration after ROS connection
  updateGyroCali();

  // Show LED status
  showLedStatus();

  // Update Voltage
  updateVoltageCheck();

  char log_msg3[50];
  
  #ifdef DEBUG
  if (Serial.available())
  {
    keyboard = Serial.read();
    if (keyboard == 'u')
    {
      p_gain = p_gain + 1.0;
    }
    else if (keyboard == 'j')
    {
      p_gain = p_gain - 1.0;
    }
    else if (keyboard == 'o')
    {
      d_gain = d_gain + 0.1;
    }
    else if (keyboard == 'l')
    {
      d_gain = d_gain - 0.1;
    }
    else if (keyboard == 'i')
    {
      i_gain = i_gain + 0.1;
    }
    else if (keyboard == 'k')
    {
      i_gain = i_gain - 0.1;
    }
  }

  Serial.print(" P : ");
  Serial.print(p_gain);
  Serial.print(" I : ");
  Serial.print(i_gain);
  Serial.print(" D : ");
  Serial.print(d_gain);
  Serial.print(" output : ");
  Serial.print(control_output_left);
  Serial.print(control_output_right);
  #endif
  
  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();
}


/*******************************************************************************
* Callback function for cmd_velocity_msg
*******************************************************************************/
void commandVelocityLeftCallback(const std_msgs::Float64& cmd_velocity_msg)
{
  cmd_velocity_left  = cmd_velocity_msg.data;
}

void commandVelocityRightCallback(const std_msgs::Float64& cmd_velocity_msg)
{
  cmd_velocity_right  = cmd_velocity_msg.data;
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg.header.stamp    = nh.now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.angular_velocity.x = imu.SEN.gyroADC[0];
  imu_msg.angular_velocity.y = imu.SEN.gyroADC[1];
  imu_msg.angular_velocity.z = imu.SEN.gyroADC[2];
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration.x = imu.SEN.accADC[0];
  imu_msg.linear_acceleration.y = imu.SEN.accADC[1];
  imu_msg.linear_acceleration.z = imu.SEN.accADC[2];
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation.w = imu.quat[0];
  imu_msg.orientation.x = imu.quat[1];
  imu_msg.orientation.y = imu.quat[2];
  imu_msg.orientation.z = imu.quat[3];

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  imu_pub.publish(&imu_msg);

  tfs_msg.header.stamp    = nh.now();
  tfs_msg.header.frame_id = "base_link";
  tfs_msg.child_frame_id  = "imu_link";
  tfs_msg.transform.rotation.w = imu.quat[0];
  tfs_msg.transform.rotation.x = imu.quat[1];
  tfs_msg.transform.rotation.y = imu.quat[2];
  tfs_msg.transform.rotation.z = imu.quat[3];

  tfs_msg.transform.translation.x = -0.032;
  tfs_msg.transform.translation.y = 0.0;
  tfs_msg.transform.translation.z = 0.068;

  tfbroadcaster.sendTransform(tfs_msg);
}

/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{
  bool dxl_comm_result = false;

  int32_t current_tick;

  sensor_state_msg.battery = checkVoltage();

  dxl_comm_result = motor_driver.readEncoder(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);

  if (dxl_comm_result == true)
  {
    sensor_state_pub.publish(&sensor_state_msg);
  }
  else
  {
    return;
  }

  current_tick = sensor_state_msg.left_encoder;

  if (!init_encoder_[LEFT])
  {
    last_tick_[LEFT] = current_tick;
    init_encoder_[LEFT] = true;
  }

  last_diff_tick_[LEFT] = current_tick - last_tick_[LEFT];
  last_tick_[LEFT] = current_tick;
  last_rad_[LEFT] += TICK2RAD * (double)last_diff_tick_[LEFT];

  current_tick = sensor_state_msg.right_encoder;

  if (!init_encoder_[RIGHT])
  {
    last_tick_[RIGHT] = current_tick;
    init_encoder_[RIGHT] = true;
  }

  last_diff_tick_[RIGHT] = current_tick - last_tick_[RIGHT];
  last_tick_[RIGHT] = current_tick;
  last_rad_[RIGHT] += TICK2RAD * (double)last_diff_tick_[RIGHT];
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;
  prev_update_time = time_now;
  ros::Time stamp_now = nh.now();

  // odom
  updateOdometry((double)(step_time * 0.001));
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);
  yaw_pub.publish(&yaw);

  //velocity and acceleration
  angular_velocity_left_pub.publish(&angular_velocity_left);
  angular_velocity_right_pub.publish(&angular_velocity_right);
  angular_acceleration_left_pub.publish(&angular_acceleration_left);
  angular_acceleration_right_pub.publish(&angular_acceleration_right);
  motor_velocity_left_pub.publish(&motor_velocity_left);
  motor_velocity_right_pub.publish(&motor_velocity_right);

  //velocity controller
  controlVelocity(step_time*0.001);

  error_left_pub.publish(&error_left);
  error_right_pub.publish(&error_right);

  //current
  updateCurrent((double)(step_time * 0.001));
  current_left_pub.publish(&current_left);
  current_right_pub.publish(&current_right);

  //disturbance_torque
  updateDisturbanceTorque((double)(step_time * 0.001));
  disturbance_torque_left_pub.publish(&disturbance_torque_left);
  disturbance_torque_right_pub.publish(&disturbance_torque_right);
  compensation_voltage_left_pub.publish(&compensation_voltage_left);
  compensation_voltage_right_pub.publish(&compensation_voltage_right);

  motor_voltage_left_pub.publish(&motor_voltage_left);
  motor_voltage_right_pub.publish(&motor_voltage_right);

  //reaction_torque
  updateReactionTorque((double)(step_time * 0.001));
  reaction_torque_left_pub.publish(&reaction_torque_left);
  reaction_torque_right_pub.publish(&reaction_torque_right);
  updateReactionForce();
  reaction_force_pub.publish(&reaction_force);
  
  // joint_states
  updateJoint();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);

  // tf
  updateTF(odom_tf);
  tfbroadcaster.sendTransform(odom_tf);
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool updateOdometry(double diff_time)
{
  double odom_vel[3];

  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick_[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick_[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  delta_theta = atan2f(imu.quat[1]*imu.quat[2] + imu.quat[0]*imu.quat[3],
                       0.5f - imu.quat[2]*imu.quat[2] - imu.quat[3]*imu.quat[3]) - last_theta;

  v = delta_s / step_time;
  w = delta_theta / step_time;

  last_velocity_[LEFT]  = wheel_l / step_time;
  last_velocity_[RIGHT] = wheel_r / step_time;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneous velocity
  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;  

  yaw.data = odom_pose[2];
  
  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  // We should update the twist of the odometry
  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];

  last_theta = atan2f(imu.quat[1]*imu.quat[2] + imu.quat[0]*imu.quat[3],
                      0.5f - imu.quat[2]*imu.quat[2] - imu.quat[3]*imu.quat[3]);
  
  last_velocity_filtered_[LEFT] = GDIFF*step_time*last_velocity_[LEFT]+(1-GDIFF*step_time)*last_velocity_filtered_[LEFT];
  last_velocity_filtered_[RIGHT] = GDIFF*step_time*last_velocity_[RIGHT]+(1-GDIFF*step_time)*last_velocity_filtered_[RIGHT];

  motor_velocity_[LEFT] = last_velocity_filtered_[LEFT]*258.5; //velocity before gear reduction
  motor_velocity_[RIGHT] = last_velocity_filtered_[RIGHT]*258.5;

  angular_velocity_left.data = last_velocity_filtered_[LEFT];
  angular_velocity_right.data = last_velocity_filtered_[RIGHT];

  motor_velocity_left.data = motor_velocity_[LEFT];
  motor_velocity_right.data = motor_velocity_[RIGHT];

  acc_tmp_[LEFT] += acc_filtered_[LEFT]*step_time;
  acc_filtered_[LEFT] = GDIFF2 * (last_velocity_filtered_[LEFT] - acc_tmp_[LEFT]);

  acc_tmp_[RIGHT] += acc_filtered_[RIGHT]*step_time;
  acc_filtered_[RIGHT] = GDIFF2 * (last_velocity_filtered_[RIGHT] - acc_tmp_[RIGHT]);

  angular_acceleration_left.data = acc_filtered_[LEFT];
  angular_acceleration_right.data = acc_filtered_[RIGHT];
  
  
  return true;
}


bool updateCurrent(double diff_time)
{ 
  double step_time;

  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;
  
  current_[LEFT] = (motor_voltage_[LEFT]-K_EN*last_velocity_filtered_[LEFT])/R_N;
  current_[RIGHT] = (motor_voltage_[RIGHT]-K_EN*last_velocity_filtered_[RIGHT])/R_N;

  current_filtered_[LEFT] = G_FILTER*step_time*current_[LEFT] + (1-G_FILTER*step_time)*current_filtered_[LEFT];
  current_filtered_[RIGHT] = G_FILTER*step_time*current_[RIGHT] + (1-G_FILTER*step_time)*current_filtered_[RIGHT];

  current_left.data = current_filtered_[LEFT];
  current_right.data = current_filtered_[RIGHT];
  
  return true;
}

bool updateDisturbanceTorque(double diff_time)
{ 
  double step_time;

  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;
  
  disturbance_torque_[LEFT] = K_TN*current_filtered_[LEFT] + G_DOB*J_N*last_velocity_filtered_[LEFT];
  disturbance_torque_[RIGHT] = K_TN*current_filtered_[RIGHT] + G_DOB*J_N*last_velocity_filtered_[RIGHT];
  disturbance_torque_tmp_[LEFT] = G_DOB*step_time*disturbance_torque_[LEFT] + (1-G_DOB*step_time)*disturbance_torque_tmp_[LEFT];
  disturbance_torque_tmp_[RIGHT] = G_DOB*step_time*disturbance_torque_[RIGHT] + (1-G_DOB*step_time)*disturbance_torque_tmp_[RIGHT];
  disturbance_torque_filtered_[LEFT] = (disturbance_torque_tmp_[LEFT] - G_DOB*J_N*last_velocity_filtered_[LEFT]);
  disturbance_torque_filtered_[RIGHT] = (disturbance_torque_tmp_[RIGHT]- G_DOB*J_N*last_velocity_filtered_[RIGHT]);
  disturbance_torque_left.data = disturbance_torque_filtered_[LEFT];
  disturbance_torque_right.data = disturbance_torque_filtered_[RIGHT];

  compensation_voltage_left.data = disturbance_torque_filtered_[LEFT]*R_N/(K_TN);
  compensation_voltage_right.data = disturbance_torque_filtered_[RIGHT]*R_N/(K_TN);
  
  return true;
}

bool updateReactionTorque(double diff_time)
{ 
  double step_time;

  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

//  if (last_velocity_filtered_[LEFT]>VELOCITY_EPSILON){
//      reaction_torque_[LEFT] = K_TN*current_filtered_[LEFT]+G_DOB*J_N*last_velocity_filtered_[LEFT] - K_TN*F_plus_left/K_EN - D_plus_left*last_velocity_filtered_[LEFT]*K_TN/K_EN;
//  }
//  else if (last_velocity_filtered_[LEFT]<-VELOCITY_EPSILON){
//      reaction_torque_[LEFT] = K_TN*current_filtered_[LEFT]+G_DOB*J_N*last_velocity_filtered_[LEFT] - F_minus_left*K_TN/K_EN - D_minus_left*last_velocity_filtered_[LEFT]*K_TN/K_EN;
//  }
//  else {
  reaction_torque_[LEFT] = K_TN*current_filtered_[LEFT]+G_DOB*J_N*last_velocity_filtered_[LEFT];
//  }

//  if (last_velocity_filtered_[RIGHT]>VELOCITY_EPSILON){
//      reaction_torque_[RIGHT] = K_TN*current_filtered_[RIGHT]+G_DOB*J_N*last_velocity_filtered_[RIGHT] - F_plus_right*K_TN/K_EN - D_plus_right*last_velocity_filtered_[RIGHT]*K_TN/K_EN;
//  }
//  else if (last_velocity_filtered_[RIGHT]<-VELOCITY_EPSILON){
//      reaction_torque_[RIGHT] = K_TN*current_filtered_[RIGHT]+G_DOB*J_N*last_velocity_filtered_[RIGHT] - F_minus_right*K_TN/K_EN - D_minus_right*last_velocity_filtered_[RIGHT]*K_TN/K_EN;
//  }
//  else {
  reaction_torque_[RIGHT] = K_TN*current_filtered_[RIGHT]+G_DOB*J_N*last_velocity_filtered_[RIGHT];
//  }
  reaction_torque_tmp_[LEFT] = G_DOB*step_time*reaction_torque_[LEFT] + (1-G_DOB*step_time)*reaction_torque_tmp_[LEFT];
  reaction_torque_tmp_[RIGHT] = G_DOB*step_time*reaction_torque_[RIGHT] + (1-G_DOB*step_time)*reaction_torque_tmp_[RIGHT];
  reaction_torque_filtered_[LEFT] = (reaction_torque_tmp_[LEFT] - G_DOB*J_N*last_velocity_filtered_[LEFT]);
  reaction_torque_filtered_[RIGHT] = (reaction_torque_tmp_[RIGHT]- G_DOB*J_N*last_velocity_filtered_[RIGHT]);
  reaction_torque_left.data = reaction_torque_filtered_[LEFT];
  reaction_torque_right.data = reaction_torque_filtered_[RIGHT];
  
  return true;
}

void updateReactionForce(void)
{ 
  mobile_robot_reaction_force = (reaction_torque_filtered_[LEFT] + reaction_torque_filtered_[RIGHT])/WHEEL_RADIUS - M_R*WHEEL_RADIUS*(acc_filtered_[LEFT]+acc_filtered_[RIGHT])/2.0 ;
  reaction_force.data = mobile_robot_reaction_force;
}

/*******************************************************************************
* Calculate the joint states
*******************************************************************************/
void updateJoint(void)
{
  joint_states_pos[LEFT]  = last_rad_[LEFT];
  joint_states_pos[RIGHT] = last_rad_[RIGHT];

  joint_states_vel[LEFT]  = last_velocity_[LEFT];
  joint_states_vel[RIGHT] = last_velocity_[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
* Calculate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom.header.frame_id = "odom";
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

bool controlVelocity(double diff_time)
{
double step_time;

  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  static int16_t cnt = 0;           // timer counter
  double current_time = millis();
  const float pi = 3.1415;
  double omega;
  omega = 2*pi*0.0001;
  
  velocity_error_left = cmd_velocity_left*(1+sin(omega*current_time)) - last_velocity_filtered_[LEFT];
  velocity_error_right = cmd_velocity_right*(1+sin(omega*current_time)) - last_velocity_filtered_[RIGHT];
  integral_left   = integral_left + (velocity_error_left * step_time);
  integral_right   = integral_right + (velocity_error_right * step_time);
  derivative_left = (velocity_error_left - pre_error_left) / step_time;
  derivative_right = (velocity_error_right - pre_error_right) / step_time;

   if (cnt > 2000)
  {
    integral_left = 0.0;
    integral_right = 0.0;
    cnt = 0;
  }
  else
  {
    cnt++;
  }
  
  control_output_left = p_gain * velocity_error_left + i_gain * integral_left  + d_gain * derivative_left;
  control_output_right = p_gain * velocity_error_right + i_gain * integral_right  + d_gain * derivative_right;

  if (control_output_left >= PWM_LIMIT)
  {
    control_output_left = PWM_LIMIT;
  }
  else if (control_output_left <= (-1) * PWM_LIMIT)
  {
    control_output_left = (-1) * PWM_LIMIT;
  }
  if (control_output_right >= PWM_LIMIT)
  {
    control_output_right = PWM_LIMIT;
  }
  else if (control_output_right <= (-1) * PWM_LIMIT)
  {
    control_output_right = (-1) * PWM_LIMIT;
  }
  
  cmd_pwm_left = pwm_gain*control_output_left;
  cmd_pwm_right = pwm_gain*control_output_right;
  
  pre_error_left = velocity_error_left;
  pre_error_right = velocity_error_right;

  error_left.data = velocity_error_left;
  error_right.data = velocity_error_right;
  
  return true;  
}


/*******************************************************************************
* Control motor pwm
*******************************************************************************/
void controlMotorPwm(void)
{
  goal_pwm_left = cmd_pwm_left + compensation_voltage_[LEFT]*885/11.1 - K_EN*last_velocity_filtered_[LEFT];
  goal_pwm_right = cmd_pwm_right + compensation_voltage_[RIGHT]*885/11.1 - K_EN*last_velocity_filtered_[RIGHT];
     
  bool dxl_comm_result = false;

  if (goal_pwm_left >= PWM_LIMIT)
  {
    goal_pwm_left = PWM_LIMIT;
  }
  else if (goal_pwm_left <= (-1) * PWM_LIMIT)
  {
    goal_pwm_left = (-1) * PWM_LIMIT;
  }

  if (goal_pwm_right >= PWM_LIMIT)
  {
    goal_pwm_right = PWM_LIMIT;
  }
  else if (goal_pwm_right <= (-1) * PWM_LIMIT)
  {
    goal_pwm_right = (-1) * PWM_LIMIT;
  }

  motor_voltage_[LEFT] = goal_pwm_left * 11.1/885;
  motor_voltage_[RIGHT] = goal_pwm_right * 11.1/885;

  motor_voltage_left.data = motor_voltage_[LEFT];
  motor_voltage_right.data = motor_voltage_[RIGHT];
  
  dxl_comm_result = motor_driver.pwmControl((int64_t)goal_pwm_left, (int64_t)goal_pwm_right);
  if (dxl_comm_result == false)
    return;
}
/*******************************************************************************
* Check voltage
*******************************************************************************/
float checkVoltage(void)
{
  float vol_value;

  vol_value = getPowerInVoltage();

  return vol_value;
}

/*******************************************************************************
* Show LED status
*******************************************************************************/
void showLedStatus(void)
{
  static uint32_t t_time = millis();

  if ((millis()-t_time) >= 500)
  {
    t_time = millis();
    digitalWrite(13, !digitalRead(13));
  }

  if (getPowerInVoltage() < 11.1)
  {
    setLedOn(LED_LOW_BATTERY);
  }
  else
  {
    setLedOff(LED_LOW_BATTERY);
  }

  if (nh.connected())
  {
    setLedOn(LED_ROS_CONNECT);
  }
  else
  {
    setLedOff(LED_ROS_CONNECT);
  }

  updateRxTxLed();
}

void updateRxTxLed(void)
{
  static uint32_t rx_led_update_time;
  static uint32_t tx_led_update_time;
  static uint32_t rx_cnt;
  static uint32_t tx_cnt;

  if ((millis()-tx_led_update_time) > 50)
  {
    tx_led_update_time = millis();

    if (tx_cnt != Serial.getTxCnt())
    {
      setLedToggle(LED_TXD);
    }
    else
    {
      setLedOff(LED_TXD);
    }

    tx_cnt = Serial.getTxCnt();
  }

  if ((millis()-rx_led_update_time) > 50)
  {
    rx_led_update_time = millis();

    if (rx_cnt != Serial.getRxCnt())
    {
      setLedToggle(LED_RXD);
    }
    else
    {
      setLedOff(LED_RXD);
    }

    rx_cnt = Serial.getRxCnt();
  }
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(void)
{
  static bool gyro_cali = false;
  uint32_t pre_time;
  uint32_t t_time;

  char log_msg[50];

  if (nh.connected())
  {
    if (gyro_cali == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      imu.SEN.gyro_cali_start();

      t_time   = millis();
      pre_time = millis();
      while(!imu.SEN.gyro_cali_get_done())
      {
        imu.update();

        if (millis()-pre_time > 5000)
        {
          break;
        }
        if (millis()-t_time > 100)
        {
          t_time = millis();
          setLedToggle(LED_ROS_CONNECT);
        }
      }
      gyro_cali = true;

      sprintf(log_msg, "Calibrattion End");
      nh.loginfo(log_msg);
    }
  }
  else
  {
    gyro_cali = false;
  }
}

/*******************************************************************************
* updateVoltageCheck
*******************************************************************************/
void updateVoltageCheck(void)
{
  static bool startup = false;
  static int vol_index = 0;
  static int prev_state = 0;
  static int alram_state = 0;
  static int check_index = 0;

  int i;
  float vol_sum;
  float vol_value;

  static uint32_t process_time[8] = {0,};
  static float    vol_value_tbl[10] = {0,};

  float voltage_ref       = 11.0 + 0.0;
  float voltage_ref_warn  = 11.0 + 0.0;


  if (startup == false)
  {
    startup = true;
    for (i=0; i<8; i++)
    {
      process_time[i] = millis();
    }
  }

  if (millis()-process_time[0] > 100)
  {
    process_time[0] = millis();

    vol_value_tbl[vol_index] = getPowerInVoltage();

    vol_index++;
    vol_index %= 10;

    vol_sum = 0;
    for(i=0; i<10; i++)
    {
        vol_sum += vol_value_tbl[i];
    }
    vol_value = vol_sum/10;
    battery_valtage_raw = vol_value;

    //Serial.println(vol_value);

    battery_voltage = vol_value;
  }


  if(millis()-process_time[1] > 1000)
  {
    process_time[1] = millis();

    //Serial.println(battery_state);

    switch(battery_state)
    {
      case BATTERY_POWER_OFF:
        if (setup_end == true)
        {
          alram_state = 0;
          if(battery_valtage_raw > 5.0)
          {
            check_index   = 0;
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_STARTUP;
          }
          else
          {
            noTone(BDPIN_BUZZER);
          }
        }
        break;

      case BATTERY_POWER_STARTUP:
        if(battery_valtage_raw > voltage_ref)
        {
          check_index   = 0;
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
          setPowerOn();
        }

        if(check_index < 5)
        {
          check_index++;
        }
        else
        {
          if (battery_valtage_raw > 5.0)
          {
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_CHECK;
          }
          else
          {
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_OFF;
          }
        }
        break;

      case BATTERY_POWER_NORMAL:
        alram_state = 0;
        if(battery_valtage_raw < voltage_ref)
        {
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_CHECK;
          check_index   = 0;
        }
        break;

      case BATTERY_POWER_CHECK:
        if(check_index < 5)
        {
          check_index++;
        }
        else
        {
          if(battery_valtage_raw < voltage_ref_warn)
          {
            setPowerOff();
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_WARNNING;
          }
        }
        if(battery_valtage_raw >= voltage_ref)
        {
          setPowerOn();
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
        }
        break;

      case BATTERY_POWER_WARNNING:
        alram_state ^= 1;
        if(alram_state)
        {
          tone(BDPIN_BUZZER, 1000, 500);
        }

        if(battery_valtage_raw > voltage_ref)
        {
          setPowerOn();
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
        }
        else
        {
          setPowerOff();
        }

        if(battery_valtage_raw < 5.0)
        {
          setPowerOff();
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_OFF;
        }
        break;

      default:
        break;
    }
  }
}

void setPowerOn()
{
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
}

void setPowerOff()
{
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}
