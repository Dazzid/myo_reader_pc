#include "Arm.h"

Arm::Arm(){
}

void Arm::setup(){
    material.setDiffuseColor(ofFloatColor(1., 1., 1., 1.0));
    handMaterial.setDiffuseColor(ofFloatColor(1.0, 0., 0., 1.0));
    
	myArm.setParent(geometry);
	extension.setParent(myArm);
    myoSphere.setParent(myArm);
    
    myoSphere.set(15,50);
	
    extension.set(10,120,20,4);
    extension.setOrientation(ofQuaternion(1, 0, 0, 1));

    myArm.setHeight(50.0);
    myArm.setWidth(50.0);
    myArm.setDepth(50.0);
    myArm.setScale(1);
    
    extension.move(0, 0, -80);
    myoSphere.move(0,0,-150);
	geometry.move(ofGetWidth()/2, ofGetHeight()/2, 0);
    //geometry.setScale(1.0);
    
    for (int i = 0; i < 5; i++){
        pPosition.push_back(ofVec3f(0,0,0));
    }
}

ofVec3f Arm::getPointVector(){
    return myoSphere.getGlobalPosition();
}

float Arm::getPointVelocity(){
    return mVelocity;
}
//----------------------------------------------------------------------
void Arm::update(){

    pPosition.push_back(myoSphere.getGlobalPosition());
    if (pPosition.size() > 5) pPosition.pop_front();
    
    result = pPosition[0]-pPosition[4];
    mVelocity = result.length();
    if(result.x > 0.5) direction = 1;
    else if(result.x < -0.5)direction = -1;
    else direction = 0;
    
    easyVelocity += (myoVelocity - easyVelocity) * easing;
    geometry.setGlobalPosition(easyVelocity.y*-50.0+ofGetWidth()/2, easyVelocity.z*50.0+ofGetHeight()/2, easyVelocity.x*-50.0);
}
//----------------------------------------------------------------------
void Arm::draw(){
    material.begin();
    myArm.draw();
    extension.draw();
    material.end();
    
    handMaterial.begin();
    myoSphere.draw();
    ofDrawArrow(pPosition[4], pPosition[0], 5.0);
    handMaterial.end();
}
//----------------------------------------------------------------------
void Arm::steerYaw(float dir){
	geometry.rotate(dir, 0, 1, 0); // the rotation happens on the y axis
	/*cout << "----------------" << endl;
	cout << "myArm: " << myArm.getGlobalOrientation() << endl;
	cout << "geometry: " << geometry.getGlobalOrientation() << endl;
	cout << "result: " << geometry.getGlobalOrientation() * geometry.getGlobalOrientation() << endl;
     */
}
//----------------------------------------------------------------------
void Arm::steerPitch(float dir) {
	geometry.rotate(dir, 1, 0, 0); // the rotation happens on the y axis
}
//----------------------------------------------------------------------
void Arm::steerRoll(float dir) {
	geometry.rotate(dir, 0, 0, 1); // the rotation happens on the y axis
}
//----------------------------------------------------------------------
ofVec3f Arm::computeZeroRollVector (ofVec3f forward){
    ofVec3f antigravity = ofVec3f(0,1,0);
    ofVec3f m = myArm.getZAxis().cross (antigravity);
    ofVec3f roll = m.cross (myArm.getZAxis());
    return roll.normalize();
}
//----------------------------------------------------------------------
float Arm::rollFromZero (ofVec3f zeroRoll, ofVec3f forward, ofVec3f up){
    // The cosine of the angle between the up vector and the zero roll vector. Since both are
    // orthogonal to the forward vector, this tells us how far the Myo has been turned around the
    // forward axis relative to the zero roll vector, but we need to determine separately whether the
    // Myo has been rolled clockwise or counterclockwise.
    float cosine = up.dot(zeroRoll);
    
    // To determine the sign of the roll, we take the cross product of the up vector and the zero
    // roll vector. This cross product will either be the same or opposite direction as the forward
    // vector depending on whether up is clockwise or counter-clockwise from zero roll.
    // Thus the sign of the dot product of forward and it yields the sign of our roll value.
    ofVec3f cp = up.cross (zeroRoll);
    float directionCosine = forward.dot (cp);
    float sign = directionCosine < 0.0f ? 1.0f : -1.0f;
    
    // Return the angle of roll (in degrees) from the cosine and the sign.
    return sign * ofRadToDeg(acos(cosine));
}
//----------------------------------------------------------------------
float Arm::normalizeAngle (float angle){
    if (angle > 180.0f) {
        return angle - 360.0f;
    }
    if (angle < -180.0f) {
        return angle + 360.0f;
    }
    return angle;
}
//----------------------------------------------------------------------
void Arm::reset(ofQuaternion inRotation){
    ofQuaternion result = inRotation.inverse();
    geometry.setOrientation(result);
    
   /*
    //trying to replicate Unity3D C# version on C++. Not complete yet
    _antiYaw.makeRotate(ofVec3f((myArm.getZAxis()).x, 0, (myArm.getZAxis()).z),ofVec3f (0, 0, 1));
    cout<<_antiYaw<<endl;
    ofVec3f referenceZeroRoll = computeZeroRollVector (myArm.getZAxis());
    _referenceRoll = rollFromZero (referenceZeroRoll, myArm.getZAxis(), myArm.getUpDir());
    
    ofVec3f zeroRoll = computeZeroRollVector (myArm.getZAxis());
    
    float roll = rollFromZero (zeroRoll, myArm.getZAxis(), myArm.getUpDir());
    float relativeRoll = normalizeAngle (roll - _referenceRoll);
    
    ofQuaternion antiRoll(relativeRoll, myArm.getZAxis());
    geometry.setOrientation(_antiYaw * antiRoll * inRotation.inverse());
    */
    
}
//----------------------------------------------------------------------
void Arm::setRotation(ofQuaternion inRotation){	
	myArm.setOrientation(ofQuaternion(inRotation.x(), inRotation.y(), inRotation.z(), inRotation.w()));
}
