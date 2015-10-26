#include "ofApp.h"
#include "ofxPubSubOsc.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetVerticalSync(true);
    ofSetFrameRate(30);

	ofBackground(255);
	ofSetLogLevel(OF_LOG_VERBOSE);

    ofSerial serial;
    auto serialList = serial.getDeviceList();
    
    fsrThread = ofPtr<SerialThread>(new SerialThread("/dev/cu.usbserial-A6004b3G"));
    fsrThread->startThread(true);
    for(int i = 0; i < serialList.size(); i++)
    {
        auto name = serialList.at(i).getDeviceName();
        if(name.compare(0, 11, "cu.usbmodem") == 0) // cu.usbmodem...
        {
            contactThread = ofPtr<SerialThread>(new SerialThread(name));
            contactThread->startThread(true);
            break;
        }
    }
    
    fsrTiles.resize(6, 0);
    fsrRaw.resize(16, 0);
    ofxPublishOsc("localhost", 14924, "/niw/client/VtoF", fsrTiles, false);
    ofxPublishOsc("192.168.0.3", 14925, "/niw/client/raw", fsrRaw, false);
    
    sender.setup("localhost", 14924);
    
    hapticPresets.resize(4, None);
    
    gui = ofPtr<ofxDatGui>(new ofxDatGui( ofxDatGuiAnchor::TOP_RIGHT ));
    gui->addFRM();
}

//--------------------------------------------------------------
void ofApp::update(){
    
    for (int i = 0; i < 4; i++) {
        fsrTiles.at(i) = 0;
        for (int j = 0; j < 4; j++) {
            ofVec2f v;
            int val = fsrThread->getSensorValue(i, j, v);
            fsrTiles.at(i) += ofClamp(val * 100 - 1000, 0, 100000) * 0.33f; // scaling
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
        if(hapticPresets[i] != Ice && vals.at(0) == 0 && vals.at(1) == 1 && vals.at(2) == 0 && vals.at(3) == 0)
        {
            hapticPresets[i] = Ice;
            message.addStringArg("ice");
            sender.sendMessage(message);
            system(("say " + ofToString(static_cast<char>('a' + i)) + " ice").c_str());
        }
        else if(hapticPresets[i] != Sand && vals.at(0) == 0 && vals.at(1) == 0 && vals.at(2) == 0 && vals.at(3) == 1)
        {
            hapticPresets[i] = Sand;
            message.addStringArg("sand");
            sender.sendMessage(message);
            system(("say " + ofToString(static_cast<char>('a' + i)) + " sand").c_str());
        }
        else if(hapticPresets[i] != Water && vals.at(0) == 0 && vals.at(1) == 0 && vals.at(2) == 1 && vals.at(3) == 0)
        {
            hapticPresets[i] = Water;
            message.addStringArg("water");
            sender.sendMessage(message);
            system(("say " + ofToString(static_cast<char>('a' + i)) + " water").c_str());
        }
        else if(hapticPresets[i] != Can && vals.at(0) == 1 && vals.at(1) == 0 && vals.at(2) == 0 && vals.at(3) == 0)
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
    ofPushStyle();
    ofPushMatrix();
    
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
    
    ofPopMatrix();
    ofPopStyle();
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

