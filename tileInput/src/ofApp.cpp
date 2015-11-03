#include "ofApp.h"
#include "ofxPubSubOsc.h"

#define GUI_S(A) guiSliders.at(#A)->getValue()
#define GUI_SADD(A, MIN, MAX, DEFAULT) guiSliders.insert(pair<string, ofxDatGuiSlider*>(#A, gui->addSlider(#A, MIN, MAX, DEFAULT)));

ofxOscSender sender, senderRemote;

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
    //ofxPublishOsc("192.168.0.3", 14925, "/niw/client/raw", fsrRaw, false);
    
    ofxSubscribeOsc(14926, "/niw/game/status", gameStatus);
    
    sender.setup("localhost", 14924);
    senderRemote.setup("192.168.0.3", 14925);
    
    hapticPresets.resize(4, None);
    
    gui = ofPtr<ofxDatGui>(new ofxDatGui( ofxDatGuiAnchor::TOP_RIGHT ));
    gui->addFRM();
    GUI_SADD(closerThreshold, 0, 50000, 20000);
    GUI_SADD(spawnThreshold, 0, 50000, 15000);
    GUI_SADD(farThreshold, 0, 100000, 60000);
    GUI_SADD(gauge, 0, 1, 0);
    GUI_SADD(fsr actual, 0, 100000, 0);
    GUI_SADD(fsr filtered, 0, 100000, 0);
    GUI_SADD(delayBeforeAdd, 0, 20, 10);
    GUI_SADD(delayAfterRemove, 0, 2, 0.5f);
    GUI_SADD(state, 0, 10, 0);
    guiGaugeMode = gui->addToggle("Enable Adaptive", true);
    
    kalmanPosition.init(1e-4, 10);
    kalmanForce.init(1e-4, 1);
    
    slot == 0;
    fsrBias.resize(16, 0);
}

void centerOfPressure(vector<int>& fsrRaw, ofVec2f& p, int& total)
{
    p.x = ((4 + fsrRaw[3 * 4 + 2] + fsrRaw[3 * 4 + 3] + fsrRaw[1 * 4 + 0] + fsrRaw[1 * 4 + 1] + 2 * (fsrRaw[1 * 4 + 2] + fsrRaw[1 * 4 + 3])) / (float)(8 + total));
    p.y = ((4 + fsrRaw[3 * 4 + 0] + fsrRaw[3 * 4 + 3] + fsrRaw[1 * 4 + 0] + fsrRaw[1 * 4 + 3]) / (float)(8 + total));

}

//--------------------------------------------------------------
void ofApp::update(){
    int fsrFrontTotal = 0;
    int frontTiles[] = {1, 3}; // 0..3
    
    int slotCount;
    bool doInitialize = ofGetKeyPressed(' ');
    for (int i = 0; i < 4; i++) {
        fsrTiles.at(i) = 0;
        for (int j = 0; j < 4; j++) {
            ofVec2f v;
            int val = fsrThread->getSensorValue(i, j, v);
            if(gameStatus != "start")
                fsrTiles.at(i) += ofClamp(val * 100, 0, 100000) * 0.5f; // scaling
            
            if(doInitialize)
                fsrBias.at(i * 4 + j) = val * 100;
            fsrRaw.at(i * 4 + j) = val * 100 - fsrBias.at(i * 4 + j); // scaling
            
            if(i == frontTiles[0] || i == frontTiles[1])
            {
                if(slotCount == slot)
                    guiSliders.at("fsr actual")->setValue(fsrRaw.at(i * 4 + j));
                slotCount++;
                fsrFrontTotal += fsrRaw.at(i * 4 + j);
            }
        }
    }
    
    ofVec2f contactPositionRaw;
    centerOfPressure(fsrRaw, contactPositionRaw, fsrFrontTotal);
    
    switch(footTracker.state)
    {
        case FootTracker::WaitForAdd:
            if(fsrFrontTotal > GUI_S(closerThreshold))
            {
                footTracker.count++;
                footTracker.time = ofGetElapsedTimef();
            }
            else if(ofGetElapsedTimef() - footTracker.time > 1.0f)
            {
                footTracker.count = 0;
            }
            if(footTracker.count > GUI_S(delayBeforeAdd))
            {
                footTracker.state = FootTracker::Update;
                footTracker.time = 0;
                footTracker.count = 0;
                for(int i = 0; i < 50; i++)
                {
                    kalmanPosition.update(contactPositionRaw);
                }
                {
                    ofxOscMessage message;
                    message.setAddress("/niw/client/aggregator/floorcontact");
                    message.addStringArg("add");
                    message.addIntArg(0);
                    message.addFloatArg(contactPosition.x);
                    message.addFloatArg(contactPosition.y);
                    message.addFloatArg(0.0f);
                    senderRemote.sendMessage(message);
                }
            }
            break;
        case FootTracker::Update:
            if(fsrFrontTotal < GUI_S(closerThreshold))
            {
                footTracker.state = FootTracker::Idle;
                footTracker.time = 0;
                footTracker.count = 0;
                guiSliders.at("gauge")->setValue(0);
                {
                    ofxOscMessage message;
                    message.setAddress("/niw/client/aggregator/floorcontact");
                    message.addStringArg("remove");
                    message.addIntArg(0);
                    message.addFloatArg(contactPosition.x);
                    message.addFloatArg(contactPosition.y);
                    message.addFloatArg(footTracker.gauge);
                    senderRemote.sendMessage(message);
                }
            }
            else
            {
                kalmanPosition.update(contactPositionRaw);
                contactPosition = kalmanPosition.getEstimation();
                kalmanForce.update(ofVec2f(fsrFrontTotal));
                float fsrFrontTotalSmoothed = kalmanForce.getEstimation().x;
                guiSliders.at("fsr filtered")->setValue(fsrFrontTotalSmoothed);
                
                footTracker.count++;
                footTracker.time += ofGetLastFrameTime();
                if(guiGaugeMode->getEnabled())
                {
                    footTracker.gauge = ofMap(fsrFrontTotalSmoothed, GUI_S(closerThreshold), GUI_S(farThreshold), 0, 1, true);
                }
                else
                {
                    footTracker.gauge = ofMap(cosf(footTracker.time * M_PI * 0.125f), 1, -1, 0, 1);
                }
                guiSliders.at("gauge")->setValue(footTracker.gauge);
                
                if(footTracker.count % 8 == 0)
                {
                    float gauge = ofMap(footTracker.gauge, 0, 1, 0.25, 1);
                    ofxOscMessage message;
                    message.setAddress("/niw/crumpleparams");
                    message.addIntArg(contactPosition.x < 1.0f ? 4 : 2);
                    message.addFloatArg(0.5 * gauge * gauge);
                    message.addFloatArg(69.6);
                    message.addFloatArg(14.2);
                    message.addFloatArg(132.2);
                    //message.addFloatArg(ofMap(footTracker.gauge, 0, 1, 100, 200));
                    message.addFloatArg(141.);
                    message.addFloatArg(152.);
                    message.addFloatArg(10.);
                    message.addFloatArg(0.718);
                    message.addFloatArg(0.435);
                    message.addFloatArg(0.643);
                    message.addFloatArg(0.843);
                    message.addFloatArg(4.736);
                    message.addFloatArg(203.);
                    sender.sendMessage(message);
                }
                if(footTracker.count % 8 == 1)
                {
                    ofxOscMessage message;
                    message.setAddress("/niw/direct");
                    message.addIntArg(contactPosition.x < 1.0f ? 4 : 2);
                    sender.sendMessage(message);
                }
                {
                    ofxOscMessage message;
                    message.setAddress("/niw/client/aggregator/floorcontact");
                    message.addStringArg("update");
                    message.addIntArg(0);
                    message.addFloatArg(contactPosition.x);
                    message.addFloatArg(contactPosition.y);
                    message.addFloatArg(footTracker.gauge);
                    senderRemote.sendMessage(message);
                }
            }
            break;
        case FootTracker::Idle:
            if(ofGetElapsedTimef() - footTracker.time > GUI_S(delayAfterRemove)/* && fsrFrontTotal < GUI_S(spawnThreshold)*/)
            {
                footTracker.state = FootTracker::WaitForAdd;
                footTracker.count = 0;
                guiSliders.at("gauge")->setValue(0);
            }
            break;
    }
    
    guiSliders.at("state")->setValue((int)footTracker.state);
    
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
            //system(("say " + ofToString(static_cast<char>('a' + i)) + " ice").c_str());
        }
        else if(hapticPresets[i] != Sand && vals.at(0) == 0 && vals.at(1) == 0 && vals.at(2) == 0 && vals.at(3) == 1)
        {
            hapticPresets[i] = Sand;
            message.addStringArg("sand");
            sender.sendMessage(message);
            //system(("say " + ofToString(static_cast<char>('a' + i)) + " sand").c_str());
        }
        else if(hapticPresets[i] != Water && vals.at(0) == 0 && vals.at(1) == 0 && vals.at(2) == 1 && vals.at(3) == 0)
        {
            hapticPresets[i] = Water;
            message.addStringArg("water");
            sender.sendMessage(message);
            //system(("say " + ofToString(static_cast<char>('a' + i)) + " water").c_str());
        }
        else if(hapticPresets[i] != Can && vals.at(0) == 1 && vals.at(1) == 0 && vals.at(2) == 0 && vals.at(3) == 0)
        {
            hapticPresets[i] = Can;
            message.addStringArg("can");
            sender.sendMessage(message);
            //system(("say " + ofToString(static_cast<char>('a' + i)) + " can").c_str());
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
    
    ofPushMatrix();
    
	ofScale(1, -1);
    ofRotate(90);
    ofTranslate(-ofGetWidth() / 2, -ofGetHeight() / 2);
    
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			ofVec2f v;
			int val = fsrThread->getSensorValue(i, j, v);
			ofDrawCircle(v * 100, val / 10.0);
		}
	}
    
    ofPopMatrix();
    
    ofTranslate(ofGetWidth() / 2, ofGetHeight() / 2);
    if(footTracker.state == FootTracker::Update)
        ofSetColor(ofFloatColor::paleVioletRed);
    ofDrawCircle(contactPosition * 100, 100);
    
    ofPopMatrix();
    ofPopStyle();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if('1' <= key && key <= '8')
        slot = key - '1';
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

