#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

	ofSetFrameRate(FRAMERATE);

	// basic initialization
	ofBackground(0);
	bPause = false;

	// sets window to the size of the screen and positions it in the
	// upper left-hand corner
	ofSetWindowShape(ofGetScreenWidth(), ofGetScreenHeight());
	ofSetWindowPosition(10, 40);

	// get display settings from XML
	loadDisplayXml();

	// get host config from XML and init OSC
	loadInitOsc();

	// initialize Kinect2 and all its streams
	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();



	// Joint names for OSC are different, so create a map
	jointNames["head"] = JointType_Head;
	jointNames["neck"] = JointType_Neck;
	jointNames["torso"] = JointType_SpineMid;
	jointNames["waist"] = JointType_SpineBase;

	jointNames["l_shoulder"] = JointType_ShoulderLeft;
	jointNames["l_elbow"] = JointType_ElbowLeft;
	jointNames["l_wrist"] = JointType_WristLeft;
	jointNames["l_hand"] = JointType_HandLeft;

	jointNames["r_shoulder"] = JointType_ShoulderRight;
	jointNames["r_elbow"] = JointType_ElbowRight;
	jointNames["r_wrist"] = JointType_WristRight;
	jointNames["r_hand"] = JointType_HandRight;

	jointNames["l_hip"] = JointType_HipLeft;
	jointNames["l_knee"] = JointType_KneeLeft;
	jointNames["l_ankle"] = JointType_AnkleLeft;
	jointNames["l_foot"] = JointType_FootLeft;

	jointNames["r_hip"] = JointType_HipRight;
	jointNames["r_knee"] = JointType_KneeRight;
	jointNames["r_ankle"] = JointType_AnkleRight;
	jointNames["r_foot"] = JointType_FootRight;

	jointNames["c_shoulder"] = JointType_SpineShoulder;
	jointNames["l_hand_tip"] = JointType_HandTipLeft;
	jointNames["l_thumb"] = JointType_ThumbLeft;
	jointNames["r_hand_tip"] = JointType_HandTipRight;
	jointNames["r_thumb"] = JointType_ThumbRight;


	// in order to send osc strings for the handstates
	handStates[HandState_Unknown] = "unknown";
	handStates[HandState_NotTracked] = "nottracked";
	handStates[HandState_Open] = "open";
	handStates[HandState_Closed] = "closed";
	handStates[HandState_Lasso] = "lasso";

	windowResized(ofGetWidth(), ofGetHeight());
}

//--------------------------------------------------------------
void ofApp::loadDisplayXml() {

	// load settings from settings.xml
	ofxXmlSettings settings;
	if (!settings.loadFile("settings.xml")) {
		ofLogNotice("failed to load settings.xml");
		ofLogNotice("setting bShowDepth to TRUE");
		ofLogNotice("setting bDrawDebug to TRUE");
		ofLogNotice("setting displayTextAlpha to 100");
		ofLogNotice("setting depthGain to 20");
		ofLogNotice("setting depthGain to TRUE");
	}

	// set boolean values
	settings.pushTag("display_config");
	bShowDepth = settings.getValue("bShowDepth", true);
	bDrawDebug = settings.getValue("bDrawDebug", true);
	displayTextAlpha = settings.getValue("displayTextAlpha", 100);
	depthGain = settings.getValue("depthGain", 20);
	bDepthInvert = settings.getValue("bDepthInvert", true);
	if (settings.getValue("bStartFullscreen", true)) {
		ofToggleFullscreen();
	}
}

//--------------------------------------------------------------
void ofApp::loadInitOsc() {

	// load settings from hostconfig.xml
	ofxXmlSettings oscXml;
	if (!oscXml.loadFile("hostconfig.xml")) {
		ofLogNotice("failed to load hostconfig.xml");
		ofLogNotice("setting ip_address to 192.168.10.100");
		ofLogNotice("setting port to 8001");
	}
	oscXml.pushTag("osc_config");
	oscHostname = oscXml.getValue("ip_address", "192.168.10.100");
	oscPort = oscXml.getValue("port", 8001);

	// initialize OSC sender
	bOscConnected = true;
	try {
		oscSkelSender.setup(oscHostname, oscPort);
	}
	catch (...) {
		ofLogError("UNABLE TO CONNECT TO NETWORK");
		bOscConnected = false;
	}
}

//--------------------------------------------------------------
void ofApp::update(){
	if (!bPause) {
		// update Kinect2
		kinect.update();

		// need to process skeletal data for a variety of tasks later
		getSkelData();
	}

	// create OSC data bundle
	oscBundle.clear();

	bundleNewUsers();
	bundleLostUsers();
	bundleCalib();
	bundleUserLoc();
	bundleRestricted();
	bundleHandStates();
	bundleLean();
	bundleJoints();

	// send the bundle
	if (bOscConnected)	oscSkelSender.sendBundle(oscBundle);
	
}

//--------------------------------------------------------------
void ofApp::getSkelData()
{
	// get the vector of skeletons
	skeletons = kinect.getBodySource()->getBodies();
	
	// clear all the holding vectors used for sorting
	trackedSkeletons.clear();
	returningUsers.clear();
	newUsers.clear();
	lostUsers.clear();

	// narrow the skeletons vector into a vector only for tracked
	// skeletons
	for (auto & body : skeletons) {
		if (!body.tracked) continue;

		trackedSkeletons.push_back(body);
	}

	// sort both the old vector of tracked skels and the new vector
	sort(trackedUsers.begin(), trackedUsers.end(), &ofApp::sortSkelsFunc);
	sort(trackedSkeletons.begin(), trackedSkeletons.end(), &ofApp::sortSkelsFunc);

	// find out which users are returning this frame
	set_intersection(trackedUsers.begin(), trackedUsers.end(),
		trackedSkeletons.begin(), trackedSkeletons.end(), 
		back_inserter(returningUsers), &ofApp::sortSkelsFunc);

	// find out which users are new this frame
	set_difference(trackedSkeletons.begin(), trackedSkeletons.end(),
		returningUsers.begin(), returningUsers.end(),
		back_inserter(newUsers), &ofApp::sortSkelsFunc);

	// find out which users were lost this frame
	set_difference(trackedUsers.begin(), trackedUsers.end(),
		returningUsers.begin(), returningUsers.end(),
		back_inserter(lostUsers), &ofApp::sortSkelsFunc);

	trackedUsers = trackedSkeletons;

}

//--------------------------------------------------------------
void ofApp::bundleNewUsers()
{
	// step through new users vector and create osc message for each
	// /new_user	userID
	for (auto & body : newUsers) {
		ofxOscMessage m;
		m.setAddress("/new_user");
		m.addIntArg(body.bodyId);

		oscBundle.addMessage(m);
	}
}

//--------------------------------------------------------------
void ofApp::bundleLostUsers()
{
	// step through lost users vector and create osc message for each
	// /lost_user	userID
	for (auto & body : lostUsers) {
		ofxOscMessage m;
		m.setAddress("/lost_user");
		m.addIntArg(body.bodyId);

		oscBundle.addMessage(m);
	}
}

//--------------------------------------------------------------
void ofApp::bundleCalib()
{
	// all new users are already calibrated in our system
	// /calib_success	userID
	for (auto & body : newUsers) {
		ofxOscMessage m;
		m.setAddress("/calib_success");
		m.addIntArg(body.bodyId);

		oscBundle.addMessage(m);
	}
}

//--------------------------------------------------------------
void ofApp::bundleUserLoc()
{
	// step through tracked users vector and add location message for each
	// q is not currently calculated, and is therefore faked
	// /user/userID		x y z (fake)q
	for (auto & body : trackedUsers) {
		ofxOscMessage m;
		m.setAddress("/user/" + ofToString(body.bodyId));
		m.addFloatArg(body.joints[JointType_SpineBase].getPosition().x);
		m.addFloatArg(body.joints[JointType_SpineBase].getPosition().y);
		m.addFloatArg(body.joints[JointType_SpineBase].getPosition().z);

		// fake q value
		m.addIntArg(0);


		oscBundle.addMessage(m);
	}
}

//--------------------------------------------------------------
void ofApp::bundleRestricted()
{
	// fake the restricted value for all users
	// /restricted/userID	(fake)is_restricted (fake)conf
	for (auto & body : trackedUsers) {
		ofxOscMessage m;
		m.setAddress("/restricted/" + ofToString(body.bodyId));
		m.addIntArg(0);

		// fake confidence value
		m.addFloatArg(1.0);

		oscBundle.addMessage(m);
	}

}

//--------------------------------------------------------------
void ofApp::bundleHandStates()
{
	// create hand state messages for each user in the tracked user vector
	// messages contain both state and confidence
	// /handstate/userID/(left/right)	handstate conf
	for (auto & body : trackedUsers) {
		ofxOscMessage m;
		m.setAddress("/handstate/" + ofToString(body.bodyId) + "/left");
		m.addStringArg(handStates[body.leftHandState]);
		m.addFloatArg(body.leftHandConfidence);

		ofxOscMessage n;
		n.setAddress("/handstate/" + ofToString(body.bodyId) + "/right");
		n.addStringArg(handStates[body.rightHandState]);
		n.addFloatArg(body.rightHandConfidence);

		
		oscBundle.addMessage(m);
		oscBundle.addMessage(n);
	}
}

//--------------------------------------------------------------
void ofApp::bundleLean()
{
	// fake the lean values and confidence value for all users
	// /lean/userID		(fake)lean_x (fake)lean_y (fake)conf
	for (auto & body : trackedUsers) {
		ofxOscMessage m;
		m.setAddress("/lean/" + ofToString(body.bodyId));
		m.addFloatArg(0.0);
		m.addFloatArg(0.0);

		//fake confidence value
		m.addFloatArg(1.0);

		oscBundle.addMessage(m);
	}
}

//--------------------------------------------------------------
void ofApp::bundleJoints()
{
	// create a message for every joint in every user
	// /skel/userID/jointname	x y z (fake)conf
	//							qx qy qz qw
	//							vx vy vz speed
	for (auto & body : trackedUsers) {
		
		auto velContainer = body.getJointVels(1.0/FRAMERATE);

		// step through all desired joints
		for (map<string, JointType>::iterator it = jointNames.begin(); it != jointNames.end(); it++) {
			ofxOscMessage m;
			m.setAddress("/skel/" + ofToString(body.bodyId) + "/" + it->first);
			auto & aJoint = body.joints[it->second];
			
			// add position to message
			m.addFloatArg(aJoint.getPosition().x);
			m.addFloatArg(aJoint.getPosition().y);
			m.addFloatArg(aJoint.getPosition().z);

			// fake confidence value
			m.addFloatArg(float(aJoint.getTrackingState())/2.0);

			// add rotation quaternion to message
			m.addFloatArg(aJoint.getOrientation().x());
			m.addFloatArg(aJoint.getOrientation().y());
			m.addFloatArg(aJoint.getOrientation().z());
			m.addFloatArg(aJoint.getOrientation().w());

			// add joint velocities to the message
			m.addFloatArg(velContainer[it->second].x);
			m.addFloatArg(velContainer[it->second].y);
			m.addFloatArg(velContainer[it->second].z);
			m.addFloatArg(velContainer[it->second].distance(ofVec3f(0, 0, 0)));

			// add message to the bundle
			oscBundle.addMessage(m);
		}
	}
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

	stringstream displayStream;
	displayStream << "version #" + ofToString(VERSION_NUM) << endl;
	displayStream << "fps: " + ofToString(ofGetFrameRate(), 2) << endl;

	if (bDrawDebug) {
		for (int i = 0; i < oscBundle.getMessageCount(); i++) {
			ofxOscMessage tempMessage = oscBundle.getMessageAt(i);
			displayStream << toString(tempMessage);
			displayStream << endl;
		}
		
	}
	ofPushStyle();
	ofSetColor(255, displayTextAlpha);
	ofDrawBitmapString(displayStream.str(), 20, 20);
	ofPopStyle();
}

//--------------------------------------------------------------
void ofApp::drawDepth() {
	// taken from EW's example
	depthPixelsCopy.setFromPixels(kinect.getDepthSource()->getPixels(), DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);
	auto tempPixelsIt = depthPixelsCopy.begin();
	for (int i = 0; tempPixelsIt + i < depthPixelsCopy.end(); i++) {
		if (tempPixelsIt[i] > 0) {	// if not undefined...
			tempPixelsIt[i] = depthGain * tempPixelsIt[i];
			if (bDepthInvert) tempPixelsIt[i] = USHRT_MAX - tempPixelsIt[i];	// invert (white is close, black is far)
		}
	}
	
	depthTexture.loadData(depthPixelsCopy);
	depthTexture.draw(OFFSET_X, OFFSET_Y, displayWidth, displayHeight);

}

//--------------------------------------------------------------
void ofApp::drawColor() {
	// taken from EW's example
	kinect.getColorSource()->draw(OFFSET_X, OFFSET_Y, displayWidth, displayHeight);
}

//--------------------------------------------------------------
void ofApp::drawSkeleton() {
	// taken from EW's example
	if (bShowDepth) {
		kinect.getBodySource()->drawProjected(OFFSET_X, OFFSET_Y, displayWidth, displayHeight, ofxKFW2::ProjectionCoordinates::DepthCamera);
	}
	else {
		kinect.getBodySource()->drawProjected(OFFSET_X, OFFSET_Y, displayWidth, displayHeight, ofxKFW2::ProjectionCoordinates::ColorCamera);
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

	switch (key)
	{
	// toggle fullscreen
	case 'f':
	case 'F':
		ofToggleFullscreen();
		break;

	 // toggle between depth stream video and color stream video
	case 'c':
	case 'C':
		bShowDepth = !bShowDepth;
		windowResized(ofGetWidth(), ofGetHeight());
		break;

	// for debugging
	case 'd':
	case 'D':
		bDrawDebug = !bDrawDebug;
		break;

	case 'p':
	case 'P':
		bPause = !bPause;
		break;

	case 'i':
	case 'I':
		bDepthInvert = !bDepthInvert;
		break;

	case 'l':
	case 'L':
		loadDisplayXml();
		break;

	case 'o':
	case 'O':
		loadInitOsc();
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
	// calculate the ratio of width to height for comparison
	// (taking into account the border)
	float ratio = (float(w) - 2.0*OFFSET_X)/(float(h) - 2.0*OFFSET_Y);

	if (bShowDepth) {
		// if the new window is taller than it is wide in relation to
		// the depth image format
		if (ratio < float(DEPTH_WIDTH) / float(DEPTH_HEIGHT)) {
			displayWidth = w - 2.0 * OFFSET_X;
			displayHeight = float(displayWidth) / DEPTH_WIDTH * DEPTH_HEIGHT;
		}
		// if the new window is wider than it is tall in relation to
		// the depth image format
		else {
			displayHeight = h - 2.0 * OFFSET_Y;
			displayWidth = float(displayHeight) / DEPTH_HEIGHT * DEPTH_WIDTH;
		}
	}
	else {
		// if the new window is taller than it is wide in relation to
		// the depth image format
		if (ratio < float(VIDEO_WIDTH) / float(VIDEO_HEIGHT)) {
			displayWidth = w - 2.0 * OFFSET_X;
			displayHeight = float(displayWidth) / VIDEO_WIDTH * VIDEO_HEIGHT;
		}
		// if the new window is wider than it is tall in relation to
		// the depth image format
		else {
			displayHeight = h - 2.0 * OFFSET_Y;
			displayWidth = float(displayHeight) / VIDEO_HEIGHT * VIDEO_WIDTH;
		}
	}
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

//--------------------------------------------------------------
string ofApp::toString(const ofxOscMessage &m) {
	ostringstream msg_string;

	msg_string << m.getAddress() << " ";
	//        msg_string << " | ";
	for (int i = 0; i < m.getNumArgs(); i++) {
		// get the argument type
		//msg_string << m.getArgTypeName(i);
		msg_string << "[" << i << "]";
		msg_string << ":";
		// display the argument - make sure we get the right type
		if (m.getArgType(i) == OFXOSC_TYPE_INT32) {
			msg_string << m.getArgAsInt32(i) << " ";
		}
		else if (m.getArgType(i) == OFXOSC_TYPE_FLOAT) {
			msg_string << m.getArgAsFloat(i) << " ";
		}
		else if (m.getArgType(i) == OFXOSC_TYPE_STRING) {
			msg_string << m.getArgAsString(i) << " ";
		}
		else {
			msg_string << "unknown ";
		}
	}
	return msg_string.str();
}