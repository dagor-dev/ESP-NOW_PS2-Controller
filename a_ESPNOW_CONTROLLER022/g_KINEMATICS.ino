

float heightRestriction(float heightRes){
  if (heightRes > quad.femur_length + quad.tibia_length - 10) heightRes = quad.femur_length + quad.tibia_length - 18;
  else if (heightRes <= 18) heightRes = 18;
  return heightRes;
}

/*Z Inverse Kinematics*/
void z(struct Leg *leg, float leg_length){
  
  float thetaZ = ( (HALF_PI) - acos( (sq(quad.femur_length) + sq(leg_length) - sq(quad.tibia_length) ) / ( 2 * quad.femur_length * leg_length ) ) ) * quad.hipRR;
  
  float phiZ = ( acos( (sq(quad.femur_length) + sq(quad.tibia_length) - sq(leg_length)) / ( 2 * quad.femur_length * quad.tibia_length ) ) ) * quad.kneeRR;

  if ( !isnan(thetaZ) ){
    float *theta = &(*leg).theta;
    //Decide the sign of the calculated angle
    // it depends on how the actuator is physically wired and what direction it thinks is positive 
    if( (*leg).id == 0 || (*leg).id == 3 ) *theta -= thetaZ;
    else if( (*leg).id == 1 || (*leg).id == 2 ) *theta += thetaZ;  
  } // Don't update calculated if it's NAN

  if ( !isnan(phiZ) ){
    float *phi = &(*leg).phi;
    if( (*leg).id == 0 || (*leg).id == 3 ) *phi = phiZ;
    else if( (*leg).id == 1 || (*leg).id == 2 ) *phi = -phiZ;
  } // Don't update calculated if it's NAN
}

/*X Inverse Kinematics*/
float x(struct Leg *leg, float posZ, float posX){ 
  
  float extraTheta = ( atan( posX / posZ ) ); 

  float thetaX = extraTheta * quad.hipRR;

  float newLegLength = ( posZ / (cos(extraTheta)) );

  newLegLength = heightRestriction(abs(newLegLength));

  if ( !isnan(thetaX) ){
    float *theta = &(*leg).theta;
    if( (*leg).id == 0 || (*leg).id == 3 ) *theta = thetaX;
    else if( (*leg).id == 1 || (*leg).id == 2 ) *theta = -thetaX;
  } // Don't update calculated if it's NAN

  return newLegLength;
}

/*Y Inverse Kinematics*/
float y(struct Leg *leg, float posZ, float posY){
  
  float distY = quad.foot_hip_offset + posY;
  
  float gammaP = atan( distY / posZ );
  if (isnan(gammaP)) gammaP = HALF_PI;

  float hipHyp = distY / sin( gammaP );

  float lambda = asin ( quad.foot_hip_offset / hipHyp );

  float gammaY = ( ( - lambda ) + gammaP  ) * quad.shoulderRR;

  float newNewLegLength = quad.foot_hip_offset/tan(lambda);

  newNewLegLength = heightRestriction(abs(newNewLegLength));

  
  if ( !isnan(gammaY) ){
    float *gamma = &(*leg).gamma;
    *gamma = gammaY;
  } // Don't update calculated if it's NAN
  
  return newNewLegLength;
}


void inverseKinematics(struct Leg *leg, float pos_z, float pos_x, float pos_y){
    
  z(leg, x(leg, y(leg, pos_z, pos_y), pos_x));
  
}

// Send calculated angles to actuators
void sendCalculatedAngles(){
  // -> front right leg
  if (frl){
    sendData(0, "M", legs[0].phi, "M", legs[0].theta, "M", legs[1].gamma);      // Fix this when you update the ACT FW
  }

  // -> back left leg
  if (bll){
    sendData(3, "M", legs[3].phi, "M", legs[3].theta, "M", legs[3].gamma);
  }
  
  // -> front left leg
  if (fll){
    sendData(1, "M", legs[1].phi, "M", -legs[1].theta, "M", legs[0].gamma);
  }
  
  // -> back right leg
  if (brl){
    sendData(2, "M", legs[2].phi, "M", legs[2].theta, "M", legs[2].gamma);
  }
  
  // -> Jump leg
  if (jl){
    sendData(0, "M", legs[0].phi, "M", legs[0].theta, "M", legs[0].gamma);
  }
}

void moveInverseKinematics(float constantZ, float constantX, float constantY){
  static float positionZ, positionX, positionY;
  // L1 BUTTON PRESS -> Read left analog stick and command 1 actuator
  if( ps2x.button(PSB_L1) || ps2x.button(PSB_R1) ){
    float a0 = ps2x.analog(0);                  // Y input
    float adderY = -((a0-128)/128);             // Retain position
    positionY += adderY * constantY;            // Multiply change by a constant

    float a2 = ps2x.analog(2);                  // X input
    float adderX = ((a2-128)/128);
    positionX += adderX * constantX; 

    float a3 = ps2x.analog(3);                  // Z input
    float adderZ = -((a3-128)/128);
    positionZ += adderZ * constantZ;

    // Check for restrictions
    positionZ = heightRestriction(positionZ);

    //Calculate IK
    inverseKinematics(&legs[0], positionZ, positionX, positionY);
    inverseKinematics(&legs[1], positionZ, positionX, -positionY);
    inverseKinematics(&legs[2], positionZ, -positionX, positionY);
    inverseKinematics(&legs[3], positionZ, -positionX, -positionY);
    
    /*Serial.print(positionZ);
    Serial.print(",  ");
    Serial.print(positionX);
    Serial.print(",  ");
    Serial.print(positionY);
    Serial.print(",  ");
    Serial.print(legs[0].gamma);
    Serial.print(",  ");
    Serial.print(legs[0].theta);
    Serial.print(",  ");
    Serial.println(legs[0].phi);*/

    //sendCalculatedAngles();
    
  }
  
}

void yaw(struct CartesianCoordinates *coord, float yawAngle, float positionX, float positionY){
  
  float posX, posY;
  
  if( (*coord).id == 0){
    posY = positionY + (quad.center_to_shoulder + quad.foot_hip_offset);
    posX = positionX + quad.center_to_hip;
  }
  else if( (*coord).id == 1){
    posY = positionY - (quad.center_to_shoulder + quad.foot_hip_offset);
    posX = positionX - quad.center_to_hip;
  }
  else if( (*coord).id == 2){
    posY = positionY - (quad.center_to_shoulder + quad.foot_hip_offset);
    posX = positionX - quad.center_to_hip;
  }
  else if( (*coord).id == 3){
    posY = positionY + (quad.center_to_shoulder + quad.foot_hip_offset);
    posX = positionX + quad.center_to_hip;
  }

  float currentFootAngle = atan(posY/posX);
  float radius = posY/ (sin(currentFootAngle) );

  float totalYawAngle = currentFootAngle + yawAngle;

  float newPosX = radius * cos(totalYawAngle);
  float newPosY = radius * sin(totalYawAngle);

  if( (*coord).id == 0){
    (*coord).y = newPosY - (quad.center_to_shoulder + quad.foot_hip_offset);
    (*coord).x = newPosX - quad.center_to_hip;
  }
  else if( (*coord).id == 1){
    (*coord).y = newPosY + (quad.center_to_shoulder + quad.foot_hip_offset);
    (*coord).x = newPosX + quad.center_to_hip;
  }
  else if( (*coord).id == 2){
    (*coord).y = newPosY + (quad.center_to_shoulder + quad.foot_hip_offset);
    (*coord).x = newPosX + quad.center_to_hip;
  }
  else if( (*coord).id == 3){
    (*coord).y = newPosY - (quad.center_to_shoulder + quad.foot_hip_offset);
    (*coord).x = newPosX - quad.center_to_hip;
  }
  
}

void pitch(struct CartesianCoordinates *coord, float pitchAngle, float positionZ, float positionX){
  
  if ((*coord).id == 2 || (*coord).id == 3) pitchAngle = -pitchAngle;

  float difZ = positionZ - ( quad.center_to_hip * sin(pitchAngle) );
  float difX = quad.center_to_hip - ( quad.center_to_hip * cos(pitchAngle) ) + positionX;

  float auxAngle = atan( difX / difZ );
  float virtualLeg = difZ / cos(auxAngle);

  float auxAngleTotal = auxAngle + pitchAngle;
  
  (*coord).z = cos(auxAngleTotal) * virtualLeg;

  (*coord).x = -sin(auxAngleTotal) * virtualLeg;
  
}

void roll(struct CartesianCoordinates *coord, float rollAngle, float positionZ, float positionY){
  
  if ((*coord).id == 1 || (*coord).id == 3) rollAngle = -rollAngle;

  float difZ = positionZ - ( quad.center_to_shoulder * sin(rollAngle) );
  float difY = quad.center_to_shoulder - ( quad.center_to_shoulder * cos(rollAngle) ) + quad.foot_hip_offset + positionY;

  float auxAngle = atan( difY / difZ );
  float virtualLeg = ( difZ / cos(auxAngle) );

  float auxAngleTotal = (auxAngle + rollAngle);
  
  (*coord).z = cos(auxAngleTotal) * virtualLeg;

  (*coord).y = (sin(auxAngleTotal) * virtualLeg) - quad.foot_hip_offset;

}

float holdPositionZ( float constantZ ){
  //static float positionZ = gait.height;
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  pressedPeriod += timeDif;
  if (pressedPeriod >= 10){
    pressedPeriod = 0;
    pressed = false;
  }
  
  if( ps2x.button(PSB_PAD_UP) && !pressed ){
    gait.height += constantZ;
    pressed = true;
  }
  else if( ps2x.button(PSB_PAD_DOWN) && !pressed ){
    gait.height -= constantZ;
    pressed = true;
  }

  gait.height = heightRestriction(gait.height);

  return gait.height;
}

void holdFeetOffsetY( float offsetY ){
  //static float positionZ = gait.height;
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  pressedPeriod += timeDif;
  if (pressedPeriod >= 10){
    pressedPeriod = 0;
    pressed = false;
  }
  
  if( ps2x.button(PSB_PAD_LEFT) && !pressed ){
    gait.foot_pos_offset_y -= offsetY;
    pressed = true;
  }
  else if( ps2x.button(PSB_PAD_RIGHT) && !pressed ){
    gait.foot_pos_offset_y += offsetY;
    pressed = true;
  }
}

float holdYawAngle(float initYaw, float constantYaw){
  static float yawAngle = initYaw;
  static bool pressed = true;
  static float pressedPeriod = 0;
  
  pressedPeriod += timeDif;
  if (pressedPeriod >= 10){
    pressedPeriod = 0;
    pressed = false;
  }
  
  if( ps2x.button(PSB_PAD_LEFT) && !pressed ){
    yawAngle += constantYaw;
    pressed = true;
  }
  else if( ps2x.button(PSB_PAD_RIGHT) && !pressed ){
    yawAngle -= constantYaw;
    pressed = true;
  }

  return yawAngle;
}

void holdInverseKinematics(float constantZ, float constantX, float constantY, float initZ, float initX, float initY){
  static float positionZ = initZ;
  static float positionX = initX; 
  static float positionY;
  static float pitchAngle;
  static float rollAngle;
  static float yawAngle;

  if( ps2x.button(PSB_L1) || ps2x.button(PSB_R1) ){
    
    // Z input
    positionZ = holdPositionZ(constantZ);
    
    // X input
    float a2 = ps2x.analog(2);
    float newY = -((a2-128)/128) * constantY;                 
    positionY = 0.98*positionY + 0.02*newY;
  
    // Y input
    float a3 = ps2x.analog(3);
    float newX = ((a3-128)/128) * constantX;
    positionX = initX + 0.98*positionX + 0.02*newX;
  
    // Yaw input
    yawAngle = holdYawAngle(0, 0.006);
  
    // Pitch input
    float a1 = ps2x.analog(1);
    float newPitch = ((a1-128)/128) * PI/10;
    pitchAngle = 0.96*pitchAngle + 0.04*newPitch; 
  
    // Roll input
    float a0 = ps2x.analog(0);
    float newRoll = ((a0-128)/128) * PI/5;
    rollAngle = 0.96*rollAngle + 0.04*newRoll;
  
    yaw(&coordinates[0], yawAngle,  positionX,  positionY - gait.foot_pos_offset_y);
    yaw(&coordinates[1], yawAngle,  positionX, -positionY - gait.foot_pos_offset_y);
    yaw(&coordinates[2], yawAngle, -positionX,  positionY - gait.foot_pos_offset_y);
    yaw(&coordinates[3], yawAngle, -positionX, -positionY - gait.foot_pos_offset_y);
  
    pitch(&coordinates[0], pitchAngle, positionZ, coordinates[0].x);
    pitch(&coordinates[1], pitchAngle, positionZ, coordinates[1].x);
    pitch(&coordinates[2], pitchAngle, positionZ, coordinates[2].x);
    pitch(&coordinates[3], pitchAngle, positionZ, coordinates[3].x);
  
    roll(&coordinates[0], rollAngle, coordinates[0].z, coordinates[0].y);
    roll(&coordinates[1], rollAngle, coordinates[1].z, coordinates[1].y);
    roll(&coordinates[2], rollAngle, coordinates[2].z, coordinates[2].y);
    roll(&coordinates[3], rollAngle, coordinates[3].z, coordinates[3].y);
  
    //Calculate IK
    inverseKinematics(&legs[0], coordinates[0].z, coordinates[0].x, coordinates[0].y);
    inverseKinematics(&legs[1], coordinates[1].z, coordinates[1].x, coordinates[1].y);
    inverseKinematics(&legs[2], coordinates[2].z, coordinates[2].x, coordinates[2].y);
    inverseKinematics(&legs[3], coordinates[3].z, coordinates[3].x, coordinates[3].y);
  
    /*Serial.print(yawAngle);
    Serial.print(",  ");
    Serial.print(pitchAngle);
    Serial.print(",  ");
    Serial.print(rollAngle);
    Serial.print(",  ");
    Serial.print(positionZ);
    Serial.print(",  ");
    Serial.print(positionX);
    Serial.print(",  ");
    Serial.println(positionY);*/
    
    /*
    Serial.print(coordinates[0].x);
    Serial.print(",  ");
    Serial.print(coordinates[1].x);
    Serial.print(",  ");
    Serial.print(coordinates[0].y);
    Serial.print(",  ");
    Serial.println(coordinates[1].y);
    */
    /*
    Serial.print(legs[0].gamma);
    Serial.print(",  ");
    Serial.print(legs[0].theta);
    Serial.print(",  ");
    Serial.println(legs[0].phi);*/
  
    sendCalculatedAngles(); 
  }
}

void gaitKinematics(int leg, float positionX, float positionY, float positionZ, float yawAngle, float pitchAngle, float rollAngle, float dur){

  static float mult = 1.0;
  static float multZ = 0.8;

  if (leg == 0) {        // front right
    positionZ = interpFRZ.go(positionZ,dur*multZ);
    positionX = interpFRX.go(positionX,dur*mult);
    positionY = interpFRY.go(positionY,dur*mult);
    yawAngle  = interpFRS.go(yawAngle, dur*mult);
  }
  
  else if (leg == 1) {    // front left
    positionZ = interpFLZ.go(positionZ,dur*multZ);
    positionX = interpFLX.go(positionX,dur*mult);
    positionY = interpFLY.go(positionY,dur*mult);
    yawAngle  = interpFLS.go(yawAngle, dur*mult);            
  }

  else if (leg == 2) {   // back right
    positionZ = interpBRZ.go(positionZ,dur*multZ);
    positionX = interpBRX.go(positionX,dur*mult);
    positionY = interpBRY.go(positionY,dur*mult);
    yawAngle  = interpBRS.go(yawAngle, dur*mult);
  }

  else if (leg == 3) {    // back left
    positionZ = interpBLZ.go(positionZ,dur*multZ);
    positionX = interpBLX.go(positionX,dur*mult);
    positionY = interpBLY.go(positionY,dur*mult);
    yawAngle  = interpBLS.go(yawAngle, dur*mult);
  }

  
  /*
  if (leg == 3) {
    Serial.print(positionX);
    Serial.print(",  ");
    Serial.print(positionY);
    Serial.print(",  ");
    Serial.print(positionZ);
    Serial.print(",  ");
    Serial.print(yawAngle, 5);
    Serial.print(",  ");
    Serial.println(pitchAngle, 5);
  }
  */
  

  yawAngle = yawAngle*PI/180;   //Passing the yawAngle to this function in Degrees because small floats don't work well with the interpolation.

  pitchAngle = pitchAngle*PI/180;

  //Calculate rotational axes
  yaw(&coordinates[leg], yawAngle,  positionX,  positionY);
  pitch(&coordinates[leg], pitchAngle, positionZ, coordinates[leg].x);
  roll(&coordinates[leg], rollAngle, coordinates[leg].z, coordinates[leg].y);

  //Calculate IK
  inverseKinematics(&legs[leg], coordinates[leg].z, coordinates[leg].x, coordinates[leg].y);
    
}
