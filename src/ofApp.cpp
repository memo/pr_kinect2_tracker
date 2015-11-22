#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

	// basic initialization
	ofBackground(0);
	bFullscreen = true;
	bShowDepth = true;

	// get values from XML
	xmlSettings.loadFile("oscSettings.xml");
	oscHostname = xmlSettings.getValue("ip_address", "192.168.10.100");
	oscPort = xmlSettings.getValue("port", 8001);
	
	// initialize OSC sender
	oscSkelSender.setup(oscHostname, oscPort);

	// initialize Kinect2 and all its streams
	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();

	
}

//--------------------------------------------------------------
void ofApp::update(){

	// update Kinect2
	kinect.update();

	// need to process skeletal data for a variety of tasks later
	getSkelData();

}

//--------------------------------------------------------------
void ofApp::getSkelData()
{
	// get the vector of skeletons
	skeletons = kinect.getBodySource()->getBodies();
}

//--------------------------------------------------------------
void ofApp::draw(){

	// draw either the depth image or the color image
	if (bShowDepth) {
		drawDepth();
	}
	else {
		drawColor();
	}

	// overlay the skeletons and hand state bubbles on the video
	drawSkeleton();

}

//--------------------------------------------------------------
void ofApp::drawDepth() {
	// taken from EW's example
	kinect.getDepthSource()->draw(VIDEO_OFFSET_X, VIDEO_OFFSET_Y, VIDEO_WIDTH, VIDEO_HEIGHT);
}

//--------------------------------------------------------------
void ofApp::drawColor() {
	// taken from EW's example
	kinect.getColorSource()->draw(VIDEO_OFFSET_X, VIDEO_OFFSET_Y, VIDEO_WIDTH, VIDEO_HEIGHT);
}

//--------------------------------------------------------------
void ofApp::drawSkeleton() {
	// taken from EW's example
	kinect.getBodySource()->drawProjected(VIDEO_OFFSET_X, VIDEO_OFFSET_Y, VIDEO_WIDTH, VIDEO_HEIGHT);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

	switch (key)
	{
	// toggle fullscreen
	case 'f':
	case 'F':
		bFullscreen = !bFullscreen;
		ofSetFullscreen(bFullscreen);
		break;

	 //toggle between depth stream video and color stream video
	case 'c':
	case 'C':
		bShowDepth = !bShowDepth;
		break;

	}

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

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
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

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
