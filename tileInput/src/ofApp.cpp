#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetVerticalSync(true);

	bSendSerialMessage = false;
	ofBackground(255);
	ofSetLogLevel(OF_LOG_VERBOSE);

	font.loadFont("DIN.otf", 64);

	serialThread.startThread(true, false);
    
    sender.setup("localhost", 14924);
    
    ofSetFrameRate(30);
}

//--------------------------------------------------------------
void ofApp::update(){
    vector<int> values(4, 0);
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ofVec2f v;
            int val = serialThread.getSensorValue(i, j, v);
            values.at(i) += val;
        }
    }
    
    ofxOscMessage message;
    message.setAddress("/niw/client/VtoF");
    message.addFloatArg(values.at(0) * 100);
    message.addFloatArg(values.at(1) * 100);
    message.addFloatArg(values.at(2) * 100);
    message.addFloatArg(values.at(3) * 100);
    message.addFloatArg(0.0f);
    message.addFloatArg(0.0f);
    sender.sendMessage(message);
    ofLogError() << values.at(0) << " "  << values.at(1) << " "  << values.at(2) << " "  << values.at(3);
}

//--------------------------------------------------------------
void ofApp::draw(){
    return;
    
	ofSetColor(80);
	ofBackground(255);
	ofFill();
	ofTranslate(200, 400);
	ofScale(1, -1);
    
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			ofVec2f v;
			int val = serialThread.getSensorValue(i, j, v);
			ofCircle(v * 100, val / 10.0);
		}
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	bSendSerialMessage = true;
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}

