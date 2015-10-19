#pragma once

#include "ofMain.h"
#include "ofxOsc.h"

class SerialThread : public ofThread {
    bool isInitialized;
    ofSerial	serial;
	vector<vector<int> > sensorVals;

	ofVec2f tileIdToCoord(int id) {
		ofVec2f v;
		switch (id) {
		case 0:
			v.x = 0.5f;
			v.y = 0.5f;
			break;
		case 1:
			v.x = 1.5f;
			v.y = 0.5f;
			break;
		case 2:
			v.x = 0.5f;
			v.y = 1.5f;
			break;
		case 3:
			v.x = 1.5f;
			v.y = 1.5f;
			break;
		}
		return v;
	}

	ofVec2f sensorIdToCoord(int id) {
		ofVec2f v;
		float d = 0.5f - 12.0f / 303.0f;
		switch (id) {
		case 0:
			v.x = -d;
			v.y = d;
			break;
		case 1:
			v.x = d;
			v.y = d;
			break;
		case 2:
			v.x = d;
			v.y = -d;
			break;
		case 3:
			v.x = -d;
			v.y = -d;
			break;
		}
		return v;
	}

	// the thread function
	void threadedFunction() {
        if (!isInitialized) {
            return;
        }

		while (isThreadRunning()) {
			int nRead = 0;  // a temp variable to keep count per read

			unsigned char bytesReturned[1];
			string resultString;

			memset(bytesReturned, 0, 1);

			bool bNewline = false;
			while ((1 > 0) && !bNewline){
				nRead = serial.readBytes(bytesReturned, 1);
				for (int i = 0; i < nRead; i++) {
					if (bytesReturned[i] != '\n') {
						resultString.append(ofToString(bytesReturned[i]));
					}
					else {
						bNewline = true;
						break;
					}
				}
			};

			//ofLogNotice() << resultString;
			vector<string> splitString = ofSplitString(resultString, " ", true);
			if (splitString.size() == 5) {
				int index = splitString.at(0).at(0) - 'a';
				if (ofInRange(index, 0.0, 3.0)) {
					for (int i = 0; i < 4; i++) {
						sensorVals.at(index).at(i) = ofToInt(splitString.at(i + 1));
					}
				}
				else {
					//ofLogNotice() << "lost (index error)";
				}
			}
			else {
				//ofLogNotice() << "lost (parse error)";
			}
            ofSleepMillis(5);
		}

	}

public:
	SerialThread(string device) {
		serial.listDevices();
		vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();

		// this should be set to whatever com port your serial device is connected to.
		// (ie, COM4 on a pc, /dev/tty.... on linux, /dev/tty... on a mac)
		// arduino users check in arduino app....
		int baud = 9600;
		isInitialized = serial.setup(); //open the first device
		//serial.setup("COM4", baud); // windows example
		//serial.setup("/dev/tty.usbserial-A4001JEC", baud); // mac osx example
		//serial.setup("/dev/ttyUSB0", baud); //linux example

		sensorVals.resize(4, vector<int>(4));
	}

	int getSensorValue(int i, int j) {
		return sensorVals.at(i).at(j);
	}

	int getSensorValue(int i, int j, ofVec2f& v) {
		v = tileIdToCoord(i) + sensorIdToCoord(j);
		return sensorVals.at(i).at(j);
	}
};

class ofApp : public ofBaseApp{

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofPtr<SerialThread> fsrThread, contactThread;
    
    enum HapticPreset {None, Ice, Snow, Can, Water, Sand};
    vector<HapticPreset> hapticPresets;
    
    vector<int> fsrRaw, fsrTiles;
    
    ofxOscSender sender;
};

