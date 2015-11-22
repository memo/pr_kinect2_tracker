#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxXmlSettings.h"
#include "ofxKinectForWindows2.h"
#include "ofxOsc.h"

#define VIDEO_WIDTH 1920
#define VIDEO_HEIGHT 1080
#define VIDEO_OFFSET_X 100
#define VIDEO_OFFSET_Y 100


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void getSkelData();

		void draw();
		void drawDepth();
		void drawColor();
		void drawSkeleton();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		bool			bFullscreen;
		bool			bShowDepth;

		ofxXmlSettings	xmlSettings;
		string			oscHostname;
		int				oscPort;

		ofxOscSender	oscSkelSender;

		ofxKFW2::Device kinect;
		vector<ofxKFW2::Data::Body> skeletons;


		
};
