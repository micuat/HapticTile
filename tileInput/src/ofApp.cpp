#include "ofApp.h"
#include "ofxPubSubOsc.h"

#define GUI_S(A) guiSliders.at(#A)->getValue()
#define GUI_SADD(G, A, MIN, MAX, DEFAULT) guiSliders.insert(pair<string, ofxDatGuiSlider*>(#A, G->addSlider(#A, MIN, MAX, DEFAULT)));

ofxOscSender sender, senderRemote, senderSurface, senderPhone;

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
    gameStatus = "none";
    sender.setup("localhost", 14924);
    senderRemote.setup("192.168.0.3", 14925);
    senderSurface.setup("142.157.102.21", 55002);
    senderPhone.setup("142.157.174.19", 55002);
    
    hapticPresets.resize(4, None);
    
    gui = ofPtr<ofxDatGui>(new ofxDatGui( ofxDatGuiAnchor::TOP_RIGHT ));
    gui->addFRM();
    
    guiZeroInitialize = gui->addButton("Zero Initialize");
    gui->buttonEventCallback = [&](ofxDatGuiButtonEvent e) {
        if(e.target == guiZeroInitialize)
        {
            doZeroInitialize = true;
        }
    };
    doZeroInitialize = false;
    
    presetOptions.push_back("kids");
    presetOptions.push_back("adults");
    presetOptions.push_back("xl");
    guiPresets = gui->addDropdown("Preset", presetOptions);
    curPreset = 1;
    gui->dropdownEventCallback = [&](ofxDatGuiDropdownEvent e) {
        if(e.target == guiPresets)
        {
            switch (e.child) {
                case 0:
                    ofLogError() << "0";
                    curPreset = 0;
                    guiSliders.at("closerThreshold")->setValue(5000);
                    guiSliders.at("spawnThreshold")->setValue(3000);
                    guiSliders.at("farThreshold")->setValue(40000);
                    break;
                case 1:
                    ofLogError() << "1";
                    curPreset = 1;
                    guiSliders.at("closerThreshold")->setValue(10000);
                    guiSliders.at("spawnThreshold")->setValue(5000);
                    guiSliders.at("farThreshold")->setValue(50000);
                    break;
                case 2:
                    ofLogError() << "2";
                    curPreset = 2;
                    guiSliders.at("closerThreshold")->setValue(15000);
                    guiSliders.at("spawnThreshold")->setValue(10000);
                    guiSliders.at("farThreshold")->setValue(60000);
                    break;
                    
                default:
                    break;
            }
        }
    };
    
    auto guiDebug = gui->addFolder("debug");
    GUI_SADD(guiDebug, closerThreshold, 0, 50000, 10000);
    GUI_SADD(guiDebug, spawnThreshold, 0, 50000, 5000);
    GUI_SADD(guiDebug, farThreshold, 0, 100000, 50000);
    GUI_SADD(guiDebug, gauge, 0, 1, 0);
    GUI_SADD(guiDebug, fsr actual, 0, 100000, 0);
    GUI_SADD(guiDebug, fsr filtered, 0, 100000, 0);
    GUI_SADD(guiDebug, delayBeforeAdd, 0, 20, 10);
    GUI_SADD(guiDebug, delayAfterRemove, 0, 2, 0.5f);
    GUI_SADD(guiDebug, state, 0, 10, 0);
    guiGaugeMode = guiDebug->addToggle("Enable Adaptive", true);
    
    kalmanPosition.init(1e-4, 10);
    kalmanForce.init(1e-4, 1);
    
    slot == 0;
    fsrBias.resize(16, 0);
}

void centerOfPressure(vector<int>& fsrRaw, ofVec2f& p, int& total)
{
    float weightedx = 0;
    weightedx += fsrRaw[3 * 4 + 2] + fsrRaw[3 * 4 + 3] + fsrRaw[1 * 4 + 0] + fsrRaw[1 * 4 + 1];
    weightedx += 2 * (fsrRaw[1 * 4 + 2] + fsrRaw[1 * 4 + 3]);
    weightedx += fsrRaw[2 * 4 + 2] + fsrRaw[2 * 4 + 3] + fsrRaw[0 * 4 + 0] + fsrRaw[0 * 4 + 1];
    weightedx += 2 * (fsrRaw[0 * 4 + 2] + fsrRaw[0 * 4 + 3]);
    p.x = (4 + weightedx) / (float)(8 + total);
    
    float weightedy = 0;
    weightedy += fsrRaw[3 * 4 + 0] + fsrRaw[3 * 4 + 3] + fsrRaw[1 * 4 + 0] + fsrRaw[1 * 4 + 3];
    weightedy += fsrRaw[2 * 4 + 1] + fsrRaw[2 * 4 + 2] + fsrRaw[0 * 4 + 1] + fsrRaw[0 * 4 + 2];
    weightedy += 2 * (fsrRaw[2 * 4 + 0] + fsrRaw[2 * 4 + 3] + fsrRaw[0 * 4 + 0] + fsrRaw[0 * 4 + 3]);
    p.y = ((4 + weightedy) / (float)(8 + total));

}

//--------------------------------------------------------------
void ofApp::update(){
    int fsrFrontTotal = 0;
    int frontTiles[] = {1, 3}; // 0..3
    
    int slotCount;
    for (int i = 0; i < 4; i++) {
        fsrTiles.at(i) = 0;
        for (int j = 0; j < 4; j++) {
            ofVec2f v;
            int val = fsrThread->getSensorValue(i, j, v);
            if(gameStatus != "start")
                fsrTiles.at(i) += ofClamp(val * 100, 0, 100000) * 0.5f; // scaling
            
            if(doZeroInitialize)
                fsrBias.at(i * 4 + j) = val * 100;
            fsrRaw.at(i * 4 + j) = val * 100 - fsrBias.at(i * 4 + j); // scaling
            
            if(i == frontTiles[0] || i == frontTiles[1])
            {
                if(slotCount == slot)
                    guiSliders.at("fsr actual")->setValue(fsrRaw.at(i * 4 + j));
                slotCount++;
            }
            fsrFrontTotal += fsrRaw.at(i * 4 + j);
        }
    }
    doZeroInitialize = false;
    
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
                    message.addStringArg(presetOptions.at(curPreset));
                    ofLogError() << presetOptions.at(curPreset);
                    senderRemote.sendMessage(message);
                    senderSurface.sendMessage(message);
                    senderPhone.sendMessage(message);
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
                    message.addStringArg(presetOptions.at(curPreset));
                    senderRemote.sendMessage(message);
                    senderSurface.sendMessage(message);
                    senderPhone.sendMessage(message);
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
                
                if(gameStatus == "start" && footTracker.count % 8 == 0)
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
                if(gameStatus == "start" && footTracker.count % 8 == 1)
                {
                    ofxOscMessage message;
                    message.setAddress("/niw/direct");
                    message.addIntArg(contactPosition.x < 1.0f ? 4 : 2);
                    sender.sendMessage(message);
                }
                if(footTracker.count % 8 == 0)
                {
                    ofxOscMessage message;
                    message.setAddress("/niw/client/aggregator/floorcontact");
                    message.addStringArg("update");
                    message.addIntArg(0);
                    message.addFloatArg(contactPosition.x);
                    message.addFloatArg(contactPosition.y);
                    message.addFloatArg(footTracker.gauge);
                    message.addStringArg(presetOptions.at(curPreset));
                    senderRemote.sendMessage(message);
                    senderSurface.sendMessage(message);
                    senderPhone.sendMessage(message);
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
            message.addStringArg("snow");
            string tileName = ofToString(static_cast<char>('a' + i));
            if(tileName == "a") tileName = "ay";
            system(("say " + tileName + " snow").c_str());
        }
        else if(hapticPresets[i] != Sand && vals.at(0) == 0 && vals.at(1) == 0 && vals.at(2) == 0 && vals.at(3) == 1)
        {
            hapticPresets[i] = Sand;
            message.addStringArg("sand");
            string tileName = ofToString(static_cast<char>('a' + i));
            if(tileName == "a") tileName = "ay";
            system(("say " + tileName + " sand").c_str());
        }
        else if(hapticPresets[i] != Water && vals.at(0) == 0 && vals.at(1) == 0 && vals.at(2) == 1 && vals.at(3) == 0)
        {
            hapticPresets[i] = Water;
            message.addStringArg("ice");
            string tileName = ofToString(static_cast<char>('a' + i));
            if(tileName == "a") tileName = "ay";
            system(("say " + tileName + " ice").c_str());
        }
        else if(hapticPresets[i] != Can && vals.at(0) == 1 && vals.at(1) == 0 && vals.at(2) == 0 && vals.at(3) == 0)
        {
            hapticPresets[i] = Can;
            message.addStringArg("can");
            string tileName = ofToString(static_cast<char>('a' + i));
            if(tileName == "a") tileName = "ay";
            system(("say " + tileName + " can").c_str());
        }
        else
        {
            continue;
        }
        sender.sendMessage(message);
        senderSurface.sendMessage(message);
        senderPhone.sendMessage(message);
        
        message.clear();
        message.setAddress("/niw/preset/volume");
        message.addIntArg(i + 1);
        if(hapticPresets[i] == Can)
        {
            int canCount = 0;
            for(int j = 0; j < 4; j++)
            {
                if(hapticPresets[j] == Can) canCount++;
            }
            message.addFloatArg(ofMap(canCount, 0, 4, 0.5f, 1.5f));
        }
        else
        {
            message.addFloatArg(1.0f);
        }
        sender.sendMessage(message);
        
        ofSleepMillis(1000);
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

