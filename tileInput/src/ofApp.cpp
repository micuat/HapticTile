#include "ofApp.h"
#include "ofxPubSubOsc.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetVerticalSync(true);

	ofBackground(255);
	ofSetLogLevel(OF_LOG_VERBOSE);

    fsrThread = ofPtr<SerialThread>(new SerialThread("/dev/cu.usbserial-A6004b3G"));
    contactThread = ofPtr<SerialThread>(new SerialThread("/dev/cu.usbmodemfd1311"));
    fsrThread->startThread(true);
    contactThread->startThread(true);
    
    fsrTiles.resize(6, 0);
    fsrRaw.resize(16, 0);
    ofxPublishOsc("localhost", 14924, "/niw/client/VtoF", fsrTiles, false);
    ofxPublishOsc("localhost", 14925, "/niw/client/raw", fsrRaw, false);
    
    sender.setup("localhost", 14924);
    
    hapticPresets.resize(4, None);
    
    ofSetFrameRate(30);
}

//--------------------------------------------------------------
void ofApp::update(){
    
    for (int i = 0; i < 4; i++) {
        fsrTiles.at(i) = 0;
        for (int j = 0; j < 4; j++) {
            ofVec2f v;
            int val = fsrThread->getSensorValue(i, j, v);
            fsrTiles.at(i) += val * 100; // scaling
            fsrRaw.at(i * 4 + j) = val * 100; // scaling
        }
    }
    
    for (int i = 0; i < 4; i++) {
        stringstream ss;
        ss << static_cast<char>('a' + i) << ' ';
        vector<int> vals;
        for (int j = 0; j < 4; j++) {
            ofVec2f v;
            int val = contactThread->getSensorValue(i, j, v);
            vals.push_back(val);
            ss << val << ' ';
        }
        //ofLogError() << ss.str();
        
        ofxOscMessage message;
        message.setAddress("/niw/preset");
        message.addIntArg(i + 1);
        if(hapticPresets[i] != Ice && vals.at(0) == 0 && vals.at(1) == 0 && vals.at(2) == 1 && vals.at(3) == 0)
        {
            hapticPresets[i] = Ice;
            message.addStringArg("ice");
            sender.sendMessage(message);
            system(("say " + ofToString(static_cast<char>('a' + i)) + " ice").c_str());
        }
        else if(hapticPresets[i] != Sand && vals.at(0) == 1 && vals.at(1) == 0 && vals.at(2) == 0 && vals.at(3) == 0)
        {
            hapticPresets[i] = Sand;
            message.addStringArg("sand");
            sender.sendMessage(message);
            system(("say " + ofToString(static_cast<char>('a' + i)) + " sand").c_str());
        }
//        else if(hapticPresets[i] != Snow && vals.at(0) == 0 && vals.at(1) == 1 && vals.at(2) == 0 && vals.at(3) == 0)
//        {
//            hapticPresets[i] = Snow;
//            message.addStringArg("snow");
//            sender.sendMessage(message);
//            system(("say " + ofToString(static_cast<char>('a' + i)) + " snow").c_str());
//        }
//        else if(hapticPresets[i] != Water && vals.at(0) == 1 && vals.at(1) == 1 && vals.at(2) == 0 && vals.at(3) == 0)
//        {
//            hapticPresets[i] = Water;
//            message.addStringArg("water");
//            sender.sendMessage(message);
//            system(("say " + ofToString(static_cast<char>('a' + i)) + " water").c_str());
//        }
        else if(hapticPresets[i] != Can && vals.at(0) == 0 && vals.at(1) == 1 && vals.at(2) == 0 && vals.at(3) == 0)
        {
            hapticPresets[i] = Can;
            message.addStringArg("can");
            sender.sendMessage(message);
            system(("say " + ofToString(static_cast<char>('a' + i)) + " can").c_str());
        }
    }
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
			int val = fsrThread->getSensorValue(i, j, v);
			ofDrawCircle(v * 100, val / 10.0);
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

