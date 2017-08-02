//
//  DataCollector.hpp
//  of_myo
//
//  Created by David Dalmazzo on 09/07/17.
//
//

#ifndef DataCollector_hpp
#define DataCollector_hpp

#include <stdio.h>
#include "ofMain.h"
#include "myo/myo.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>

#define EMG_SENSOR 8

class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
    : onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
    {
    }
    
    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        // We've lost a Myo.
        // Let's clean up some leftover state.
        roll_w = 0;
        pitch_w = 0;
        yaw_w = 0;
        myoQuaternion = ofQuaternion(0, 0, 0, 0);
        myoEuler = ofVec3f(0,0,0);
        onArm = false;
        isUnlocked = false;
    }
    
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg){
        for (int i = 0; i < EMG_SENSOR; i++) {
            emgSamples[i] = emg[i];
        }
    }
    
    void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro) {
        myoGyro = ofVec3f(gyro.x(), gyro.y(), gyro.z());
    }
    
    void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel) {
        myoAccelerometer = ofVec3f(accel.x(), accel.y(), accel.z());
    }
    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;
        
        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                          1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
        
        // Convert the floating point angles in radians to a scale from 0 to 180.
        roll_w = static_cast<int>((roll + PI) / (PI * 2.0f) * 180);
        pitch_w = static_cast<int>((pitch + PI / 2.0f) / PI * 180);
        yaw_w = static_cast<int>((yaw + PI) / (PI * 2.0f) * 180);
        
        //myoQuaternion= ofQuaternion(quat.x()*-1.0, quat.y(), quat.z()*-1.0, quat.w());
        myoQuaternion = ofQuaternion(quat.y()*-1.0, quat.z(), quat.x()*-1.0, quat.w());
        myoEuler = ofVec3f(yaw_w, pitch_w, roll_w);
    }
    
    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;
        
        if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
            // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
            // Myo becoming locked.
            myo->unlock(myo::Myo::unlockHold);
            
            // Notify the Myo that the pose has resulted in an action, in this case changing
            // the text on the screen. The Myo will vibrate.
            myo->notifyUserAction();
        }
        else {
            // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
            // are being performed, but lock after inactivity.
            myo->unlock(myo::Myo::unlockTimed);
        }
    }
    
    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                   myo::WarmupState warmupState)
    {
        onArm = true;
        whichArm = arm;
    }
    
    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
    }
    
    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = true;
    }
    
    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = false;
    }
    
    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.
    ofQuaternion getQuaternion() {
        return myoQuaternion;
    }
    
    ofVec3f getEuler() {
        return myoEuler;
    }
    
    ofVec3f getGyroscope() {
        return myoGyro;
    }
    
    ofVec3f getAccelerometer() {
        return myoAccelerometer;
    }
    
    int getEMG(int in) {
        return emgSamples[in];
    }
    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
        // Clear the current line
        std::cout << '\r';
        
        // Print out the EMG data.
        for (size_t i = 0; i < emgSamples.size(); i++) {
            std::ostringstream oss;
            oss << static_cast<int>(emgSamples[i]);
            std::string emgString = oss.str();
            
            std::cout << '[' << emgString << std::string(4 - emgString.size(), ' ') << ']';
        }
        std::cout <<"\n"<<"----------------------------------------------"<<"\n";
        
        // Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
        std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
        << '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
        << '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';
        
        if (onArm) {
            // Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.
            
            // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
            // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
            // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
            std::string poseString = currentPose.toString();
            
            std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
            << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
            << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
        }
        else {
            // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
            std::cout << '[' << std::string(4, ' ') << ']' << "[?]" << '[' << std::string(8, ' ') << ']';
        }
        
        std::cout << std::flush;
    }
    
    // These values are set by onArmSync() and onArmUnsync() above.
    bool onArm;
    myo::Arm whichArm;
    
    // This is set by onUnlocked() and onLocked() above.
    bool isUnlocked;
    
    // These values are set by onOrientationData() and onPose() above.
    int roll_w, pitch_w, yaw_w;
    ofQuaternion myoQuaternion;
    ofVec3f myoEuler;
    ofVec3f myoGyro;
    ofVec3f myoAccelerometer;
    myo::Pose currentPose;
    vector<int> emgSamples{0,0,0,0,0,0,0,0};
};


#endif /* DataCollector_hpp */
