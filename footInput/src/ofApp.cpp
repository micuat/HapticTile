#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_NOTICE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
	footThresholded.allocate(kinect.width / 2, kinect.height / 2, OF_IMAGE_GRAYSCALE);
	contourFinder.setMinAreaRadius(10);
	contourFinder.setMaxAreaRadius(100);
	contourFinder.setThreshold(15);
	// wait for half a frame before forgetting something
	contourFinder.getTracker().setPersistence(15);
	// an object can move up to 32 pixels per frame
	contourFinder.getTracker().setMaximumDistance(32);

	ofSetVerticalSync(true);
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
}

//--------------------------------------------------------------
void ofApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		contourFinder.findContours(footThresholded);
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		drawPointCloud();
	} else {
		// draw from the live kinect
		kinect.drawDepth(480, 0, 640, 480);
		kinect.draw(0, 0, 640, 480);
		for (int i = 0; i < cornersRgb.size(); i++) {
			ofCircle(cornersRgb.at(i), 5);
		}
	}
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
        
    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		<< "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

    if(kinect.hasCamTiltControl()) {
    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
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
	ofxCv::RectTracker& tracker = contourFinder.getTracker();
	for (int i = 0; i < contourFinder.size(); i++) {
		unsigned int label = contourFinder.getLabel(i);
		// only draw a line if this is not a new label
		if (tracker.existsPrevious(label)) {
			// use the label to pick a random color
			ofSeedRandom(label << 24);
			ofSetColor(ofColor::fromHsb(ofRandom(255), 255, 255));
			// get the tracked object (cv::Rect) at current and previous position
			const cv::Rect& previous = tracker.getPrevious(label);
			const cv::Rect& current = tracker.getCurrent(label);
			// get the centers of the rectangles
			ofVec2f previousPosition(previous.x + previous.width / 2, previous.y + previous.height / 2);
			ofVec2f currentPosition(current.x + current.width / 2, current.y + current.height / 2);
			ofLine(previousPosition, currentPosition);
		}
	}
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
		ofTriangle(corners3f.at(0), corners3f.at(1), corners3f.at(2));
		ofTriangle(corners3f.at(2), corners3f.at(3), corners3f.at(0));
//		ofTriangle(cornersUp3f.at(0), cornersUp3f.at(1), cornersUp3f.at(2));
//		ofTriangle(cornersUp3f.at(2), cornersUp3f.at(3), cornersUp3f.at(0));
	}
	ofDisableDepthTest();
	ofPopMatrix();
	easyCam.end();
}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;

		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
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
			float distUpOffset = 50.f;
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
