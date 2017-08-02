#pragma once

#include "ofMain.h"
#include <deque>  

class Arm{
public:
    Arm();
    void setup();
    void draw();
    void update();
    void steerYaw(float dir);
	void steerPitch(float dir);
	void steerRoll(float dir);
	void reset(ofQuaternion inRotation);
	void setRotation(ofQuaternion inRotation);
    void setMyoVelocity(ofVec3f inVelocity){myoVelocity = inVelocity;}
    ofVec3f computeZeroRollVector (ofVec3f forward);
    float rollFromZero (ofVec3f zeroRoll, ofVec3f forward, ofVec3f up);
    float normalizeAngle (float angle);
    ofVec3f getPointVector();
    float getPointVelocity();
    int getDirection(){return direction;}
    ofVec3f getVectorDirection(){return result;}
    
private:
    float acceleration;
    float vel;
    ofBoxPrimitive geometry;
	ofBoxPrimitive myArm;
	ofCylinderPrimitive extension;
    ofSpherePrimitive myoSphere;
    ofMaterial material;
    ofMaterial handMaterial;
    
    ofVec3f pVelocity;
    ofVec3f myoVelocity;
    ofVec3f easyVelocity;
    ofVec3f result;
    ofVec3f easing = ofVec3f(0.2,0.2,0.2);
    
    deque<ofVec3f> pPosition;
    float mVelocity;

    ofVec3f pDirection;
    ofQuaternion _antiYaw;
    ofQuaternion antiRoll;
    float _referenceRoll = 0.0f;
    int direction = 1;
};
