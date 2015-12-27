#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_NOTICE);
	
	kinect.setRegistration(true);
    
	kinect.init();
	
	kinect.open();		// opens first available kinect
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
	footThresholded.allocate(kinect.width / 2, kinect.height / 2, OF_IMAGE_GRAYSCALE);
	contourFinder.setMinAreaRadius(5);
	contourFinder.setMaxAreaRadius(100);
	contourFinder.setThreshold(15);
	// wait for half a frame before forgetting something
	contourFinder.getTracker().setPersistence(15);
	// an object can move up to 32 pixels per frame
	contourFinder.getTracker().setMaximumDistance(32);

	ofSetVerticalSync(true);
	ofSetFrameRate(60);
	
	// start from the front
	bDrawPointCloud = false;

	sender.setup(HOST, PORT);
}

//--------------------------------------------------------------
void ofApp::update() {
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		contourFinder.findContours(footThresholded);
		ofxCv::RectTracker& tracker = contourFinder.getTracker();
		for (int i = 0; i < contourFinder.size(); i++) {
			unsigned int label = contourFinder.getLabel(i);
			// only draw a line if this is not a new label
			if (tracker.existsPrevious(label)) {
				ofxOscMessage m;
				m.setAddress("/footInput/" + ofToString(ofGetElapsedTimef()));
				ofVec2f vel = ofxCv::toOf(tracker.getVelocity(i));
				m.addFloatArg(vel.x);
				m.addFloatArg(vel.y);
				sender.sendMessage(m);
			}
		}

	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	
    ofBackground(100, 100, 100);
    
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		drawPointCloud();
	} else {
		// draw from the live kinect
		kinect.drawDepth(480, 0, 640, 480);
		kinect.draw(0, 0, 640, 480);
		for (int i = 0; i < cornersRgb.size(); i++) {
			ofDrawCircle(cornersRgb.at(i), 5);
		}
	}
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
    
    reportStream << "fps: " << ofGetFrameRate() << endl;

	ofDrawBitmapString(reportStream.str(), 20, 652);
    
}

void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			footThresholded.setColor(x / 2, y / 2, 0);
			if (kinect.getDistanceAt(x, y) > 0) {
				ofPoint p = kinect.getWorldCoordinateAt(x, y);
				if (corners3f.size() == 4) {
					ofPoint mult = p * floorNormal;
					if (mult.x + mult.y + mult.z - 1 > 0) {
						continue;
					}
					mult = p * floorNormalUp;
					if (mult.x + mult.y + mult.z - 1 < 0) {
						continue;
					}
				}
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(p);
				footThresholded.setColor(x / 2, y / 2, 255);
			}
		}
	}
	footThresholded.update();
	ofPushMatrix();
	ofScale(2, 2);
	ofSetColor(255);
	footThresholded.draw(0, 0);
	contourFinder.draw();
	ofPopMatrix();
	easyCam.begin();
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	for (int i = 0; i < corners3f.size(); i++) {
		ofDrawSphere(corners3f.at(i), 10);
	}
	if (corners3f.size() == 4) {
		ofSetColor(200, 50);
		ofDrawTriangle(corners3f.at(0), corners3f.at(1), corners3f.at(2));
		ofDrawTriangle(corners3f.at(2), corners3f.at(3), corners3f.at(0));
	}
	ofDisableDepthTest();
	ofPopMatrix();
	easyCam.end();
}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
	if (!bDrawPointCloud) {
		if (kinect.getDistanceAt(x, y) > 0 && corners3f.size() < 4) {
			float distOffset = 30.f;
			float distUpOffset = 100.f;
			cornersRgb.push_back(ofPoint(x, y));
			corners3f.push_back(kinect.getWorldCoordinateAt(x, y) - ofPoint(0, 0, distOffset));
			cornersUp3f.push_back(kinect.getWorldCoordinateAt(x, y) - ofPoint(0, 0, distUpOffset + distOffset));

			if (corners3f.size() == 4) {
				cv::Mat o = cv::Mat_<double>::ones(4, 1);
				cv::Mat m = (cv::Mat_<double>(4, 3) << corners3f.at(0).x, corners3f.at(0).y, corners3f.at(0).z,
					corners3f.at(1).x, corners3f.at(1).y, corners3f.at(1).z,
					corners3f.at(2).x, corners3f.at(2).y, corners3f.at(2).z,
					corners3f.at(3).x, corners3f.at(3).y, corners3f.at(3).z);
				cv::Mat a = m.inv(cv::DECOMP_SVD) * o;
				floorNormal.x = a.at<double>(0);
				floorNormal.y = a.at<double>(1);
				floorNormal.z = a.at<double>(2);
				ofLogNotice() << floorNormal;

				m = (cv::Mat_<double>(4, 3) << cornersUp3f.at(0).x, cornersUp3f.at(0).y, cornersUp3f.at(0).z,
					cornersUp3f.at(1).x, cornersUp3f.at(1).y, cornersUp3f.at(1).z,
					cornersUp3f.at(2).x, cornersUp3f.at(2).y, cornersUp3f.at(2).z,
					cornersUp3f.at(3).x, cornersUp3f.at(3).y, cornersUp3f.at(3).z);
				a = m.inv(cv::DECOMP_SVD) * o;
				ofLogNotice() << a;
				floorNormalUp.x = a.at<double>(0);
				floorNormalUp.y = a.at<double>(1);
				floorNormalUp.z = a.at<double>(2);

			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}
