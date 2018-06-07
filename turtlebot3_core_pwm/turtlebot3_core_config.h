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

#define CONTROL_MOTOR_PWM_PERIOD       30   //hz
#define IMU_PUBLISH_PERIOD               200  //hz
#define SENSOR_STATE_PUBLISH_PERIOD      30   //hz (initially 30)
#define DRIVE_INFORMATION_PUBLISH_PERIOD 30   //hz (initially 30)
#define CURRENT_PERIOD                   200

#define M_R                              1.0             // kg
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

#define GDIFF                            10.0
#define GDIFF2                           5.0
#define G_FILTER                         10.0
#define G_DOB                            5.0
#define K_EN                             1.7//0.006553
#define K_TN                             0.30//0.8727//0.19
#define R_N                              9.54545//8.3
#define J_N                              0.01//0.087//0.0003 //0.000001175 //0.00001235
#define TAU_CST                          0.12                   

#define F_plus_left                     0.014274731
#define D_plus_left                     -0.0204030112
#define F_minus_left                    -0.0119349983
#define D_minus_left                    -0.0235699064
#define F_plus_right                    0.0267315332
#define D_plus_right                    -0.031700967
#define F_minus_right                   -0.0304814575
#define D_minus_right                   -0.0278212661
#define VELOCITY_EPSILON                 0.1

// Callback function prototypes
void commandPwmLeftCallback(const std_msgs::Int64& cmd_pwm_msg);
void commandPwmRightCallback(const std_msgs::Int64& cmd_pwm_msg);

// Function prototypes
void publishImuMsg(void);
void publishSensorStateMsg(void);
void publishDriveInformation(void);
bool updateOdometry(double diff_time);
bool updateCurrent(double diff_time);
bool updateDisturbanceTorque(double diff_time);
bool updateReactionTorque(double diff_time);
void updateReactionForce(void);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void receiveRemoteControlData(void);
void controlMotorPwm(void);
uint8_t getButtonPress(void);
void testDrive(void);
void checkPushButtonState(void);
float checkVoltage(void);
void showLedStatus(void);
void updateRxTxLed(void);

#endif // TURTLEBOT3_CORE_CONFIG_H_
