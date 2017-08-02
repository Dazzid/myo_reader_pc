#include "ofApp.h"
#include <iostream>
#include <fstream>
#include <string>

#define EMG_SENSORS 8
#define SENSOR_WIDTH 400
#define SENSOR_HEIGHT 200
#define ROUND_CORNERS 4
#define Background_Sensors 50

//--------------------------------------------------------------
void ofApp::setup() {
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    
    osc.setup(HOST, PORT);
    
    myFont.load("avenir.ttf", 8, true);
    titles.load("avenir.ttf", 12, true);
    info.load("avenir.ttf", 10, true);
    
    // turn on smooth lighting //
    ofSetSmoothLighting(true);
    ofEnableDepthTest();
    ofEnableAlphaBlending();
    center.set(ofGetWidth()*.5, ofGetHeight()*.5, 0);
    
    pointLight.setAmbientColor(ofColor(130.f));
    pointLight.setSpecularColor(ofColor(130.f));
    pointLight.setPosition(center.x, center.y, 300);
    
    lightColor.setBrightness(255.f);
    lightColor.setSaturation(0.f);
    pointLight.setDiffuseColor(lightColor);
    
    showFps = false;
    saveData = false;
    sendOSCData = false;
    
    arm.setup();
    //ofEnableDepthTest();
    
    user_path = ofFilePath::getPathForDirectory(ofFilePath::getUserHomeDir());
    dir_path =  user_path + "\\" + "Documents" + "\\" + "Myo_Recordings" + "\\";
    system(("mkdir "+ dir_path).c_str());

    //myo
    try {
        std::cout << "Attempting to find a Myo..." << std::endl;
        myo::Myo* myo = hub.waitForMyo(10000);
        
        // If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
        if (!myo) {
            throw std::runtime_error("Unable to findm Myo");
        }
        
        // We've found a Myo
        std::cout << "Connected to a Myo armband!" << std::endl << std::endl;
        hub.addListener(&collector);
        myo->setStreamEmg(myo::Myo::streamEmgEnabled);
    }
    catch (const std::exception& e) {
        std::cout << "Check in 'Myo Armband Manager' if your Myo is connected" << std::endl << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        setup();
    }
    for (int i = 0; i < 8; i++) {
        deque<float> values;
        sensors.push_back(values);
    }
    arm.reset(mQuat);
}
//--------------------------------------------------------------
template <>
void ofApp::addMessage(string address, ofVec3f data) {
    ofxOscMessage msg;
    msg.setAddress(address);
    msg.addFloatArg(data.x);
    msg.addFloatArg(data.y);
    msg.addFloatArg(data.z);
    bundle.addMessage(msg);
}

template <>
void ofApp::addMessage(string address, ofQuaternion data) {
    ofxOscMessage msg;
    msg.setAddress(address);
    msg.addFloatArg(data.w());
    msg.addFloatArg(data.x());
    msg.addFloatArg(data.y());
    msg.addFloatArg(data.z());
    bundle.addMessage(msg);
}

template <>
void ofApp::addMessage(string address, float data) {
    ofxOscMessage msg;
    msg.setAddress(address);
    msg.addFloatArg(data);
    bundle.addMessage(msg);
}

template <>
void ofApp::addMessage(string address, int data) {
    ofxOscMessage msg;
    msg.setAddress(address);
    msg.addIntArg(data);
    bundle.addMessage(msg);
}

void ofApp::addMessage(string address, int64_t time_0, int time_1, int frameNumber, int v_1, int v_2, int v_3) {
    ofxOscMessage msg;
    msg.setAddress(address);
    msg.addInt64Arg(time_0);
    msg.addIntArg(time_1);
    msg.addIntArg(frameNumber);
    msg.addIntArg(v_1);
    msg.addIntArg(v_2);
    msg.addIntArg(v_3);
    bundle.addMessage(msg);
}

void ofApp::addMessage(string address, int emg_0, int emg_1, int emg_2, int emg_3, int emg_4, int emg_5, int emg_6, int emg_7){
    ofxOscMessage msg;
    msg.setAddress(address);
    msg.addIntArg(emg_0);
    msg.addIntArg(emg_1);
    msg.addIntArg(emg_2);
    msg.addIntArg(emg_3);
    msg.addIntArg(emg_4);
    msg.addIntArg(emg_5);
    msg.addIntArg(emg_6);
    msg.addIntArg(emg_7);
    bundle.addMessage(msg);
}

void ofApp::sendBundle() {
    osc.sendBundle(bundle);
}

void ofApp::clearBundle() {
    bundle.clear();
}
//--------------------------------------------------------------
void ofApp::saveCSV(bool save_in){
    if(save_in){
        string date = ofToString(ofGetHours())+ofToString(ofGetMinutes())+ofToString(ofGetSeconds())+"_"+ofToString(ofGetDay())+ofToString(ofGetMonth())+ofToString(ofGetYear());
        string createFile = dir_path  + "myoSensorsData_" + date + ".csv";
        
        lastTiming = ofGetElapsedTimeMillis();
        myfile.open (createFile);
        myfile << "timer, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, euler_x, euler_y, euler_z, quat_w, quat_x, quat_y, quat_z, point_vector_x, point_vector_y, point_vector_z, point_direction_x, point_direction_y, point_direction_z, point_vector_velocity, emg_0, emg_1, emg_2, emg_3, emg_4, emg_5, emg_6, emg_7, event\n";
    }
    else {
        myfile.flush();
        myfile.close();
    }
}

//--------------------------------------------------------------
void ofApp::update() {
    
    //collector.print();
    hub.run(10);
    //collector.print();
    for (int i = 0; i < EMG_SENSORS; i++) {
        //cout<< collector.getEMG(i);
        float myValue = collector.getEMG(i) * 0.25;
        sensors.at(i).push_back(myValue);
        if (sensors.at(i).size() > SENSOR_WIDTH) {
            sensors.at(i).pop_front();
        }
    }
    
    gyroscope.push_back(collector.getGyroscope());
    if(gyroscope.size() > SENSOR_WIDTH) gyroscope.pop_front();
    
    accelerometer.push_back(collector.getAccelerometer());
    if(accelerometer.size() > SENSOR_WIDTH) accelerometer.pop_front();
    
    quaternion.push_back(collector.getQuaternion());
    if(quaternion.size() > SENSOR_WIDTH) quaternion.pop_front();
    
    mRotation = collector.getEuler();
    mQuat = collector.getQuaternion();
    arm.update();
    arm.setMyoVelocity(collector.getAccelerometer());
    //cout << collector.getQuaternion()<<" "<<collector.getVectorPos()<<endl;
    
    frameNumber ++;
    if(saveData){
        currentTiming = ofGetElapsedTimeMillis() - lastTiming;
        myfile<<ofToString(currentTiming) + "," +
        ofToString(collector.getAccelerometer().x) + "," +
        ofToString(collector.getAccelerometer().y) + "," +
        ofToString(collector.getAccelerometer().z) + "," +
        ofToString(collector.getGyroscope().x) + "," +
        ofToString(collector.getGyroscope().y) + "," +
        ofToString(collector.getGyroscope().z) + "," +
        ofToString(collector.getEuler().x) + "," +
        ofToString(collector.getEuler().y) + "," +
        ofToString(collector.getEuler().z) + "," +
        ofToString(collector.getQuaternion().w()) + "," +
        ofToString(collector.getQuaternion().x()) + "," +
        ofToString(collector.getQuaternion().y()) + "," +
        ofToString(collector.getQuaternion().z()) + "," +
        ofToString(arm.getPointVector().x)+ "," +
        ofToString(arm.getPointVector().y)+ "," +
        ofToString(arm.getPointVector().z)+ "," +
        ofToString(arm.getVectorDirection().x)+ "," +
        ofToString(arm.getVectorDirection().y)+ "," +
        ofToString(arm.getVectorDirection().z)+ "," +
        ofToString(arm.getPointVelocity())+ "," +
        ofToString(collector.getEMG(0)) + "," +
        ofToString(collector.getEMG(1)) + "," +
        ofToString(collector.getEMG(2)) + "," +
        ofToString(collector.getEMG(3)) + "," +
        ofToString(collector.getEMG(4)) + "," +
        ofToString(collector.getEMG(5)) + "," +
        ofToString(collector.getEMG(6)) + "," +
        ofToString(collector.getEMG(7)) + "," +
        ofToString(recording_event) + "\n";
    }
    
    if(sendOSCData){
        clearBundle();
        currentTiming = ofGetElapsedTimeMillis() - lastTiming;
        addMessage("/qtm/data", int64_t(currentTiming), int(currentTiming), frameNumber, 0, 0, 6);
        addMessage("/features/myo/timeMS",currentTiming);
        addMessage("/features/myo/accelerometer",collector.getAccelerometer());
        addMessage("/features/myo/gyroscope",collector.getGyroscope());
        addMessage("/features/myo/euler",collector.getEuler());
        addMessage("/features/myo/quaternion",collector.getQuaternion());
        addMessage("/features/myo/point/vector",arm.getPointVector());
        addMessage("/features/myo/point/velocity",arm.getPointVelocity());
        addMessage("/features/myo/point/direction",arm.getVectorDirection());
        addMessage("/features/myo/emg",collector.getEMG(0), collector.getEMG(1), collector.getEMG(2), collector.getEMG(3), collector.getEMG(4), collector.getEMG(5), collector.getEMG(6), collector.getEMG(7));
        addMessage("/features/myo/event",recording_event);
        sendBundle();
    }
}

//--------------------------------------------------------------
void ofApp::drawEmg(int inx, int iny){
    ofDisableDepthTest();
    ofPushMatrix();
    ofTranslate(0, 0, -1);
    ofSetColor(Background_Sensors);
    ofFill();
    ofDrawRectRounded(inx, iny, SENSOR_WIDTH, ofGetHeight()-15, ROUND_CORNERS);
    ofPopMatrix();
    float scale = ofGetHeight() / 8;
    
    ofSetColor(255);
    string myTitle = "EMG";
    titles.drawString(myTitle, inx+SENSOR_WIDTH/2-titles.stringWidth(myTitle)/2, iny+16);

    ofPushMatrix();
    ofTranslate(inx, (iny+scale/2)-3, 0);
    float range = ((ofGetHeight()-25)/8)/5;
    for (int i = 0; i < EMG_SENSORS; i++) {
        ofSetColor(255,80);
        if(i < EMG_SENSOR-1){
            for (int j = 0; j < SENSOR_HEIGHT; j += 3) {
                ofDrawLine(j*2, (i * scale) + scale / 2 - 5, j*2 + 2, (i * scale) + scale / 2 - 5);
            }
        }
        ofSetColor(255);
        myFont.drawString(" 1.0 -", 0, (i * scale) - range*2);
        myFont.drawString(" 0.5 -", 0, (i * scale) - range);
        myFont.drawString(" 0.0 -", 0, (i * scale));
        myFont.drawString("-0.5 -", 0, (i * scale) + range);
        myFont.drawString("-1.0 -", 0, (i * scale) + range*2);
        
         for (int c = 0; c<17; c++){
             myFont.drawString("-", 21, ((i * scale) + (c * range*0.25))-range*2);
         }
        
        ofBeginShape();
        ofSetColor(100, 200, 255);
        ofNoFill();
        for (int j = 0; j < sensors.at(i).size(); j++) {
            ofVertex(j, sensors.at(i).at(j) + (i * scale)-2);
        }
        ofEndShape(false);
    }
    ofPopMatrix();
    ofEnableDepthTest();
}
//--------------------------------------------------------------
void ofApp::drawGyroscope(int inx, int iny){
    ofDisableDepthTest();
    ofPushMatrix();
    ofTranslate(0, 0, 0);
    ofSetColor(Background_Sensors);
    ofFill();
    ofDrawRectRounded(inx, iny, SENSOR_WIDTH, SENSOR_HEIGHT, ROUND_CORNERS);
    ofPopMatrix();
    
    ofSetColor(255);
    string myTitle = "Gyroscope";
    titles.drawString(myTitle, inx+SENSOR_WIDTH/2-titles.stringWidth(myTitle)/2, iny+16);
    
    ofPushMatrix();
    ofTranslate(inx+4, iny+13, 0.0);
    string text = "multiply the values by 16 (rad/s)";
    info.drawString(text, SENSOR_WIDTH/2-info.stringWidth(text)/2, 180);
    float scale = 45.0;
    int c = 0;
    for (int i = 0; i<20; i++){
        myFont.drawString("-", 22, (i * scale*0.2));
        if(i%5 == 0 || i == 19) {
            myFont.drawString(textVales[c], 0, (c * scale));
            c++;
        }
    }
    ofPopMatrix();
    
    ofPushMatrix();
    ofTranslate(inx, iny+100, 0.0);
    ofBeginShape();
    ofNoFill();
    ofSetColor(0, 180, 255);
    for (int i = 0; i < gyroscope.size(); i++) {
        gyroscope.at(i).limit(1000.0);
        ofVertex(i, gyroscope.at(i).x/10.);
    }
    ofEndShape(false);
    ofPopMatrix();
    
    ofPushMatrix();
    ofTranslate(inx, iny+100, 0.0);
    ofBeginShape();
    ofNoFill();
    ofSetColor(255, 180, 0);
    for (int i = 0; i < gyroscope.size(); i++) {
        ofVertex(i, gyroscope.at(i).y/10.);
    }
    ofEndShape(false);
    ofPopMatrix();
    
    ofPushMatrix();
    ofTranslate(inx, iny+100, 0.0);
    ofBeginShape();
    ofNoFill();
    ofSetColor(55, 200, 0);
    for (int i = 0; i < gyroscope.size(); i++) {
        ofVertex(i, gyroscope.at(i).z/10.);
    }
    ofEndShape(false);
    ofPopMatrix();
    ofEnableDepthTest();
}

//--------------------------------------------------------------
void ofApp::drawAccelerometer(int inx, int iny){
    ofDisableDepthTest();
    ofPushMatrix();
    ofTranslate(0, 0, -1);
    ofSetColor(Background_Sensors);
    ofFill();
    ofDrawRectRounded(inx, iny, SENSOR_WIDTH, SENSOR_HEIGHT, ROUND_CORNERS);
    ofPopMatrix();
    
    ofSetColor(255);
    string myTitle = "Accelerometer";
    titles.drawString(myTitle, inx+200-titles.stringWidth(myTitle)/2, iny+16);
    
    ofPushMatrix();
    ofTranslate(inx+4, iny+13, 0.0);
    string text = "multiply the values by 8g -> 1g = 9.81 m/s²";
    info.drawString(text, SENSOR_WIDTH/2-info.stringWidth(text)/2, 180);
    float scale = 45.0;
    
    int c = 0;
    for (int i = 0; i<20; i++){
        myFont.drawString("-", 22, (i * scale*0.2));
        if(i%5 == 0 || i == 19) {
            myFont.drawString(textVales[c], 0, (c * scale));
            c++;
        }
    }
    ofPopMatrix();
    
    
    ofPushMatrix();
    ofTranslate(inx, iny+100, 0.0);
    
    ofBeginShape();
    ofNoFill();
    ofSetColor(0, 180, 255);
    for (int i = 0; i < accelerometer.size(); i++) {
        accelerometer.at(i).limit(5.0);
        ofVertex(i, accelerometer.at(i).x*20.);
    }
    ofEndShape(false);
    ofPopMatrix();
    
    ofPushMatrix();
    ofTranslate(inx, iny+100, 0.0);
    ofBeginShape();
    ofNoFill();
    ofSetColor(255, 180, 0);
    for (int i = 0; i < accelerometer.size(); i++) {
        ofVertex(i, accelerometer.at(i).y*20.);
    }
    ofEndShape(false);
    ofPopMatrix();
    
    ofPushMatrix();
    ofTranslate(inx, iny+100, 0.0);
    ofBeginShape();
    ofNoFill();
    ofSetColor(55, 200, 0);
    for (int i = 0; i < accelerometer.size(); i++) {
        ofVertex(i, accelerometer.at(i).z*20.);
    }
    ofEndShape(false);
    ofPopMatrix();
    ofEnableDepthTest();
}

//--------------------------------------------------------------
void ofApp::drawQuaternion(int inx, int iny){
    ofDisableDepthTest();
    ofPushMatrix();
    ofTranslate(0, 0, -1);
    ofSetColor(Background_Sensors);
    ofFill();
    ofDrawRectRounded(inx, iny, SENSOR_WIDTH, SENSOR_HEIGHT, ROUND_CORNERS);
    ofPopMatrix();
    
    ofSetColor(255);
    string myTitle = "Orientation (quaternion)";
    titles.drawString(myTitle, inx+SENSOR_WIDTH/2-titles.stringWidth(myTitle)/2, iny+16);
    
    ofPushMatrix();
    ofTranslate(inx+4, iny+13, 0.0);
    float scale = 45.0;
    int c = 0;
    for (int i = 0; i<20; i++){
        myFont.drawString("-", 22, (i * scale*0.2));
        if(i%5 == 0 || i == 19) {
            myFont.drawString(textVales[c], 0, (c * scale));
            c++;
        }
    }
    ofPopMatrix();
    
    ofPushMatrix();
    ofTranslate(inx, iny+100, 0.0);
    ofBeginShape();
    ofNoFill();
    ofSetColor(0, 180, 255);
    for (int i = 0; i < quaternion.size(); i++) {
        ofVertex(i, quaternion.at(i).x()*90.);
    }
    ofEndShape(false);
    ofPopMatrix();
    
    ofPushMatrix();
    ofTranslate(inx, iny+100, 0.0);
    ofBeginShape();
    ofNoFill();
    ofSetColor(255, 20, 0);
    for (int i = 0; i < quaternion.size(); i++) {
        ofVertex(i, quaternion.at(i).y()*90.);
    }
    ofEndShape(false);
    ofPopMatrix();
    
    ofPushMatrix();
    ofTranslate(inx, iny+100, 0.0);
    ofBeginShape();
    ofNoFill();
    ofSetColor(55, 200, 0);
    for (int i = 0; i < quaternion.size(); i++) {
        ofVertex(i, quaternion.at(i).z()*90.);
    }
    ofEndShape(false);
    ofPopMatrix();
    
    ofPushMatrix();
    ofTranslate(inx, iny+100, 0.0);
    ofBeginShape();
    ofNoFill();
    ofSetColor(255, 180, 0);
    for (int i = 0; i < quaternion.size(); i++) {
        ofVertex(i, quaternion.at(i).w()*90.);
    }
    ofEndShape(false);
    ofPopMatrix();
    ofEnableDepthTest();
}
//--------------------------------------------------------------
void ofApp::draw() {
    ofBackgroundGradient(ofColor(50), ofColor(0));
    if(!sendOSCData){
        drawEmg(10,10);
        drawGyroscope(ofGetWidth()-SENSOR_WIDTH-10,10);
        drawAccelerometer(ofGetWidth()-SENSOR_WIDTH-10,220);
        drawQuaternion(ofGetWidth()-SENSOR_WIDTH-10,430);
    }
    
    pointLight.enable();
    //cam.begin();
    arm.setRotation(mQuat);
    arm.draw();
    //cam.end();
    ofDisableLighting();
    pointLight.disable();
    
    if(sendOSCData){
        ofSetColor(255);
        string sending = "Sending on: " + ofToString(HOST) +" port: " + ofToString(PORT);
        string event = "Event: "+ ofToString(recording_event);
        titles.drawString(sending, ofGetWidth()/2-titles.stringWidth(sending)/2, ofGetHeight() - 45);
        titles.drawString(event, ofGetWidth()/2-titles.stringWidth(event)/2, ofGetHeight() - 25);
    }
    if(saveData){
        ofSetColor(255);
        ofDrawBitmapString("Fps: " + ofToString(ofGetFrameRate()), ofGetWidth()-120, 25);
        string record = "Recording data on Documents/Myo_Recordings";
        string event = "Event: "+ ofToString(recording_event);
        titles.drawString(record, ofGetWidth()/2-titles.stringWidth(record)/2, 25);
        titles.drawString(event, ofGetWidth()/2-titles.stringWidth(event)/2, 45);
    }
    if(showFps){
        ofSetColor(255);
        ofDrawBitmapString("Fps: " + ofToString(ofGetFrameRate()), ofGetWidth()-120, 25);
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
    
    switch (key) {
        case OF_KEY_UP:
            arm.steerPitch(-1);
            break;
        case OF_KEY_DOWN:
            arm.steerPitch(1);
            break;
        case OF_KEY_LEFT:
            arm.steerYaw(1);
            break;
        case OF_KEY_RIGHT:
            arm.steerYaw(-1);
            break;
        case 'z':
            arm.steerRoll(1);
            break;
        case 'x':
            arm.steerRoll(-1);
            break;
        case ' ':
            arm.reset(mQuat);
            break;
        case 'f':
            showFps =! showFps;
            break;
        case 's':
            saveData =! saveData;
            recording_event = 0;
            saveCSV(saveData);
            break;
        case 'e':
            recording_event++;
            break;
        case 'c':
            //TODO reconnect myo
            break;
        case 'o':
            sendOSCData =! sendOSCData;
            showFps = true;
            break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {
    //pointLight.setPosition(center.x+200, center.y, x);
    //pointLight.setAmbientColor(ofColor(x));
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {
    
}
