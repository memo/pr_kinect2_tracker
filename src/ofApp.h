#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxXmlSettings.h"
#include "ofxKinectForWindows2.h"
#include "ofxOsc.h"

#define VIDEO_WIDTH 1920
#define VIDEO_HEIGHT 1080
#define VIDEO_OFFSET_X 10
#define VIDEO_OFFSET_Y 10
#define FRAMERATE 30


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void getSkelData();
		void bundleNewUsers();
		void bundleLostUsers();
		void bundleCalib();
		void bundleUserLoc();
		void bundleRestricted();
		void bundleHandStates();
		void bundleLean();
		void bundleJoints();

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
		
		string toString(const ofxOscMessage &m);

		// settings	(TODO: read from settings xml)
		bool						bFullscreen;
		bool						bShowDepth;
		bool						bDrawDebug;
		int							displayTextAlpha;

		string						oscHostname;
		int							oscPort;

		ofxOscSender				oscSkelSender;
		ofxOscBundle				oscBundle;
		map<string, JointType>		jointNames;
		string						handStates[5];

		ofxKFW2::Device				kinect;
		vector<ofxKFW2::Data::Body> trackedUsers;

		vector<ofxKFW2::Data::Body> skeletons;
		vector<ofxKFW2::Data::Body> trackedSkeletons;
		vector<ofxKFW2::Data::Body> returningUsers;
		vector<ofxKFW2::Data::Body> newUsers;
		vector<ofxKFW2::Data::Body> lostUsers;

		static bool sortSkelsFunc(ofxKFW2::Data::Body skel1, ofxKFW2::Data::Body skel2) {
			return (skel1.trackingId < skel2.trackingId); }
		
};