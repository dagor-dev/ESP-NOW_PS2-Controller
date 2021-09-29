//MY MAC ADDRESS: 24:6F:28:51:ED:A4

#include "PsxLib.h"
#include <Ramp.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi_internal.h> 

Psx ps2x; // create PS2 Controller instance

/*
 * Leg structure
 * 
 * Contains the angular positions af the 3 actuators of the legs, each corresponding to an ID (0-3)
 * 
 */

typedef struct Leg{
  int id;
  float theta;          // Angular position of the knee [rads]
  float phi;            // Angular position of the shoulder [rads]
  float gamma;          // Angular position of the hip [rads]
} Leg;

Leg legs[] = { {.id = 0},
               {.id = 1},
               {.id = 2},
               {.id = 3}};


/*
 * CartesianCoordinates structure
 * 
 * Contains the (x,y,z) coordinates of the feet, each corresponding to an ID (0-3)
 * 
 */
typedef struct CartesianCoordinates{
  float id;
  float z;
  float x;
  float y;
} CartesianCoordinates;

CartesianCoordinates coordinates[] = { {.id = 0},
                                       {.id = 1},
                                       {.id = 2},
                                       {.id = 3}};


class Interpolation {  
public:
    rampFloat myRamp;
    bool interpolationFlag = false;
    float savedValue;    

    float go(float input, int duration) {

      if (input != savedValue) {   // check for new data
          interpolationFlag = false;
      }
      savedValue = input;          // bookmark the old value  
    
      if (interpolationFlag == 0) {                                        // only do it once until the flag is reset
          myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
          interpolationFlag = true;
      }
    
      float output = myRamp.update();               
      return output;
    }
};    // end of class

Interpolation interpFRX;        // interpolation objects front right leg
Interpolation interpFRY;
Interpolation interpFRZ;
Interpolation interpFRS;

Interpolation interpFLX;        // interpolation objects front left leg
Interpolation interpFLY;
Interpolation interpFLZ;
Interpolation interpFLS;

Interpolation interpBRX;        // interpolation objects back right leg
Interpolation interpBRY;
Interpolation interpBRZ;
Interpolation interpBRS;

Interpolation interpBLX;        // interpolation objects back left leg
Interpolation interpBLY;
Interpolation interpBLZ;
Interpolation interpBLS;

//########_Torque testing_########
bool tt = false;
//uint8_t TT1[] = {0x98, 0xCD, 0xAC, 0xB6, 0x37, 0x58};     //QM5006 9:1
//uint8_t TT1[] = {0xA4, 0xE5, 0x7C, 0x4F, 0x7D, 0xBC};        //QM5006 11.11:1
uint8_t TT1[] = {0xA8, 0x03, 0x2A, 0xCE, 0xA9, 0xD8};        //5010
bool torqueTest = false;

//########_Jumping Leg_########
bool jl = false;
uint8_t HT1[] = {0xA8, 0x03, 0x2A, 0xCE, 0xAA, 0x48};       //Jump hip Actuator
//uint8_t KT1[] = {0xAC, 0x67, 0xB2, 0x59, 0xAB, 0xB8}; 
//uint8_t KT1[] = {0x98, 0xCD, 0xAC, 0xB6, 0x37, 0x58};
uint8_t KT1[] = {0xA4, 0xE5, 0x7C, 0x4F, 0x7D, 0xBC};

//########_Front Right Leg_########
bool frl = true;                                           // ID -> 0
uint8_t H1[] = {0x98, 0xCD, 0xAC, 0xB6, 0x36, 0xD0};       // Hip
uint8_t S1[] = {0xA4, 0xE5, 0x7C, 0x4F, 0x7E, 0xC4};       // Shoulder
uint8_t K1[] = {0xA8, 0x03, 0x2A, 0xCE, 0xA9, 0xD8};       // Knee
//uint8_t K1[] = {0x98, 0xCD, 0xAC, 0xB6, 0x37, 0x58};     //QM5006 9:1

//########_Front Left Leg_########
bool fll = true;                                            // ID -> 1
uint8_t H2[] = {0xA4, 0xE5, 0x7C, 0x4F, 0x7F, 0x34};
uint8_t S2[] = {0x98, 0xCD, 0xAC, 0xAD, 0x25, 0x28};
uint8_t K2[] = {0xA4, 0xE5, 0x7C, 0x4F, 0x7D, 0xBC};        //QM5006 11.11:1      

//########_Back Right Leg_########
bool brl = true;
uint8_t H3[] = {0xA4, 0xE5, 0x7C, 0x4F, 0x7E, 0x80};
uint8_t S3[] = {0xA4, 0xE5, 0x7C, 0x4F, 0x93, 0xB4};
uint8_t K3[] = {0xA4, 0xE5, 0x7C, 0x4F, 0x7E, 0x44};

//########_Back Left Leg_########
bool bll = true;
uint8_t H4[] = {0xA4, 0xE5, 0x7C, 0x4F, 0x7F, 0x98};
uint8_t S4[] = {0xA4, 0xE5, 0x7C, 0x4F, 0x7F, 0x04};
uint8_t K4[] = {0xA4, 0xE5, 0x7C, 0x4F, 0x7D, 0x30};

//#########_STATE MACHINE_########
int stateMachine = 0;
int sp = 0;
bool jumping = false;

//#####_TIME MANAGEMENT_#####
unsigned long runTime, prevT = 0, timeDif; 
unsigned long jumpPeriod, jumpInterval = 225;
unsigned long kinematicsPeriod, kinematicsFreq = 8;
unsigned long walkingPeriod, walkingFreq = 4;

//######_FUNCTION DECLARATION_######
void holdInverseKinematics(float constantZ, float constantX, float constantY, float initZ = gait.height, float initX = gait.foot_pos_offset_x, float initY = gait.foot_pos_offset_y);
void moveInverseKinematics(float constantZ, float constantX, float constantY);
void walking(int stepPeriod = gait.step_period, float constantX = gait.step_length_x, float constantY = gait.step_length_y, float constantYaw = gait.yaw_angle, float constantPitch = gait.pitch_angle);
