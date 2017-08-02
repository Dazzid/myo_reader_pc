#pragma once

#include "ofMain.h"
#include "Arm.h"
#include "DataCollector.h"
#include "ofxOsc.h"
#include <deque>  

#define HOST "localhost"
#define PORT 6666
#define EMG_SENSOR 8

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void drawEmg(int inx, int iny);
        void drawGyroscope(int inx, int iny);
        void drawAccelerometer(int inx, int iny);
        void drawQuaternion(int inx, int iny);
        void saveCSV(bool save_in);
        void clearBundle();
        template <class T>
        void addMessage(string address, T data);
        void addMessage(string address, int64_t time_0, int time_1, int frameNumber, int v_1, int v_2, int v_3);
        void addMessage(string address, int emg_0, int emg_1, int emg_2, int emg_3, int emg_4, int emg_5, int emg_6, int emg_7);
        void sendBundle();

		DataCollector collector;
		myo::Hub hub;
    
		vector<deque<float>> sensors;
        deque<ofVec3f> gyroscope;
        deque<ofVec3f> accelerometer;
        deque<ofQuaternion> quaternion;
    
        string textVales[5] = {" 1.0 -"," 0.5 -"," 0.0 -","-0.5 -","-1.0 -"};
    
		ofVec3f mRotation;
		ofVec3f mGyro;
		ofQuaternion mQuat;
		ofLight pointLight;
		ofVec3f center;
		ofColor lightColor;
		ofBoxPrimitive geometry;
        bool showFps;

		//ofEasyCam cam;
		Arm arm;
		ofMaterial roadMaterial;

		ofTrueTypeFont	myFont;
        ofTrueTypeFont	titles;
        ofTrueTypeFont	info;
    
        float currentTiming = 0;
        float lastTiming = 0;
        int frameNumber = 0;
        string user_path = "";
        string dir_path =  "";
    
        bool saveData;

        ofstream myfile;
        int recording_event = 0;
    
        //OSC
        bool sendOSCData;
        ofxOscSender osc;
        ofxOscBundle bundle;
};
