/* 
 *  ESP32 based controller for Quadruped robot through  directional ESP-NOW
 *      - Full body Kinematics and open-loop gait controller -
 *  
 *  David Gonzalez 2021
 *  
 */

//##########################_ROBOT PARAMETERS_##########################
typedef struct Robot{
  const float femur_length = 154;          // Length of the femur (knee actuator) [mm]
  const float tibia_length = 165;          // Length of the tibia [mm]
  const float center_to_hip = 215;         // Distance from center of the robot to center of shoulder's axis of rotation [mm]
  const float center_to_shoulder = 80;     // Distance from center of the robot to center of hip's axis of ratiation [mm]
  const float foot_hip_offset = 75;        // Offset from hip axis of rotation to foot [mm]
  const float kneeRR = 11.1111;            // Reduction ratio of the knee actuators
  const float hipRR = 9.25;                // Reduction ratio of the hip actuators
  const float shoulderRR = 9.0;            // Reduction ratio of the shoulder actuators
} Robot;

Robot quad;


//##########################_GAIT PARAMETERS_##########################
typedef struct Gait_parameters{
  float height = 240;                      // Height of the Robot [mm]
  float foot_pos_offset_x = 0;             // Offset in the Y axis, move the feet closer or further [mm]
  float foot_pos_offset_y = 0;             // Offset in the X axis, move the feet closer or further [mm]
  float step_length_x = 50;                // Length of the step in the X axis [mm]
  float step_length_y = 30;                // Length of the step in the Y axis [mm]
  float step_length_z = 80;                // Lenght the foot is picked up on the swing phase of the gait [mm]
  float yaw_angle = 6;                     // Degrees to turn in yaw during gait [degrees]
  float pitch_angle = 7;                   // Max pitch angle during gait [degrees]
  float step_period = 100;                 // Time each phase of the gait takes [ms]
} Gait_parameters;

Gait_parameters gait;


//##########################_INVERSE KINEMATICS DEMO PARAMETERS_##########################
typedef struct IK_parameters{
  const float max_x_movement = 80;         // Max torso movement in the X axis [mm]
  const float max_y_movement = 50;         // Max torso movement in the Y axis [mm]
} IK_parameters;

IK_parameters ik;
