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

#ifndef TURTLEBOT3_CORE_CONFIG_H_
#define TURTLEBOT3_CORE_CONFIG_H_

#include <math.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>

#include <IMU.h>

#include "turtlebot3_motor_driver.h"

#define CONTROL_MOTOR_PWM_PERIOD       2000   //hz
#define IMU_PUBLISH_PERIOD               200  //hz
#define SENSOR_STATE_PUBLISH_PERIOD      2000   //hz (initially 30)
#define DRIVE_INFORMATION_PUBLISH_PERIOD 2000   //hz (initially 30) au 19 juin

#define M_R                              1.0             // kg
#define J_R                              0.0032
#define WHEEL_RADIUS                     0.033           // meter
#define WHEEL_SEPARATION                 0.160           // meter (BURGER : 0.160, WAFFLE : 0.287)
#define TURNING_RADIUS                   0.080           // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.105           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw
#define LEFT                             0
#define RIGHT                            1

#define VELOCITY_CONSTANT_VALUE          1263.632956882  // V = r * w = r * RPM * 0.10472
                                                         //   = 0.033 * 0.229 * Goal RPM * 0.10472
                                                         // Goal RPM = V * 1263.632956882

#define PWM_LIMIT                       885

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree

#define WAIT_FOR_BUTTON_PRESS            0
#define WAIT_SECOND                      1
#define CHECK_BUTTON_RELEASED            2

#define K_EN                             0.43
#define K_TN                             0.43
#define R_N                              8.7
#define J_N                              0.01  

#define GDIFF                            5.0
#define GDIFF2                           5.0
#define G_FILTER                         10.0
#define G_SENSOR                         1.0
#define G_DOB                            5.0 //5 le 19 juin     
#define G_ROBOT                          5.0             

#define F_plus_left                     0.019//0.0913729125
#define D_plus_left                     0.058//0.3092316078
#define F_minus_left                    -0.01//-0.0392929464
#define D_minus_left                    0.063//0.3157354114
#define F_plus_right                    0.00034//0.0913729125
#define D_plus_right                    0.06//0.3092316078
#define F_minus_right                   -0.016//-0.0392929464
#define D_minus_right                   0.06//0.3157354114
#define DISTURBANCE_EPSILON             0.1

#define F_R                             0.0
#define D_R                             0.0
#define F_ROTATION                      0.0
#define D_ROTATION                      0.0

#define K_P                              0.5
#define K_I                              50.0
#define K_D                              0.0

// Callback function prototypes
void commandVelocityLeftCallback(const std_msgs::Float64& cmd_velocity_msg);
void commandVelocityRightCallback(const std_msgs::Float64& cmd_velocity_msg);

// Function prototypes
void publishImuMsg(void);
void publishSensorStateMsg(void);
void publishDriveInformation(void);
void publishSensorValue(void);
bool updateOdometry(double diff_time);
bool updateSensorValue(double diff_time);
bool updateDisturbanceTorque(double diff_time);
bool updateReactionTorque(double diff_time);
bool updateReactionForce(double diff_time);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void receiveRemoteControlData(void);
void controlMotorPwm(void);
bool controlAcceleration(double diff_time);
uint8_t getButtonPress(void);
void testDrive(void);
void checkPushButtonState(void);
float checkVoltage(void);
void showLedStatus(void);
void updateRxTxLed(void);

#endif // TURTLEBOT3_CORE_CONFIG_H_
