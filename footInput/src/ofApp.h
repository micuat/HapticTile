#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"

#define HOST "localhost"
#define PORT 12345

// Windows users:
// You MUST install the libfreenect kinect drivers in order to be able to use
// ofxKinect. Plug in the kinect and point your Windows Device Manager to the
// driver folder in:
//
//     ofxKinect/libs/libfreenect/platform/windows/inf
//
// This should install the Kinect camera, motor, & audio drivers.
//
// You CANNOT use this driver and the OpenNI driver with the same device. You
// will have to manually update the kinect device to use the libfreenect drivers
// and/or uninstall/reinstall it in Device Manager.
//
// No way around the Windows driver dance, sorry.

class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	
	ofxKinect kinect;
	
	ofxCv::ContourFinder contourFinder;

	bool bDrawPointCloud;
	
	int angle;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;

	vector<ofPoint> cornersRgb;
	vector<ofPoint> corners3f;
	vector<ofPoint> cornersUp3f;
	ofVec3f floorNormal, floorNormalUp;
	ofImage footThresholded;

	ofxOscSender sender;
};
