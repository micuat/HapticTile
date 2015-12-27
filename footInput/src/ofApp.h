#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"

#define HOST "localhost"
#define PORT 12345

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
    
	// used for viewing the point cloud
	ofEasyCam easyCam;

	vector<ofPoint> cornersRgb;
	vector<ofPoint> corners3f;
	vector<ofPoint> cornersUp3f;
	ofVec3f floorNormal, floorNormalUp;
	ofImage footThresholded, floorMask;

	ofxOscSender sender;

    cv::Mat homography;
};
