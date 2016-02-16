
#include "Receiver.h"
#include "ofxImGui.h"

namespace pr {

float Receiver::pos_smoothing = 0.5;
float Receiver::vel_smoothing = 0.96;
float Receiver::spring_strength = 0.02;
float Receiver::spring_damping = 0.05;
int Receiver::kill_frame_count = 10;      // kill person afer this many frames of not receiving


void Receiver::initOsc() {
    ofLogNotice() << "Receiver " << _index << " initing oscReceiver on port " << _port;
    if(!oscReceiver) oscReceiver = make_unique<ofxOscReceiver>();
    oscReceiver->setup(_port);
}



void Receiver::parseOsc() {
    if(!oscReceiver) initOsc();

    // parse incoming OSC
    while(oscReceiver->hasWaitingMessages()) {
        _isConnected = 5;   // arbitrary, 5 frames buffer on connected flag

		ofxOscMessage m;
		oscReceiver->getNextMessage(m);

		if (strstr(m.getAddress().c_str(), "/skel/")) {

			// should get something like splitAddress = ["", "skel", "0", "c_shoulder"]
			vector<string> splitAddress = ofSplitString(m.getAddress(), "/");

            // assume we're parsing person with id == user_id
            int user_id = ofToInt(splitAddress[2]);

			// if new user found and calibrated, add to map
			if (!persons[user_id]) {
				ofLogWarning() << "Receiver::parseOsc creating person " << user_id;
				persons[user_id] = make_shared<Person>();
			}

            // this is the person we're receiving info for
            Person::Ptr person = persons[user_id];

            // whether it's new user or existing user, update joint details


            // reset alive counter
            person->alive_counter = 0;

            // read from osc:
			string jointName = splitAddress[3];
			float confidence = m.getArgAsFloat(3);
			ofVec3f pos = ofVec3f(m.getArgAsFloat(0), m.getArgAsFloat(1), m.getArgAsFloat(2));
			ofQuaternion quat = ofQuaternion(m.getArgAsFloat(4), m.getArgAsFloat(5), m.getArgAsFloat(6), m.getArgAsFloat(7));
			ofVec3f euler = quat.getEuler();
			ofVec3f vel = ofVec3f(m.getArgAsFloat(8), m.getArgAsFloat(9), m.getArgAsFloat(10));
            //        float speed;  // DON"T READ SPEED FROM OSC


            // apply world transformation to pos, quat and vel
			ofVec4f tempPos = ofVec4f(pos.x, pos.y, pos.z, 1.0);
			pos = tempPos*node.getGlobalTransformMatrix();
			ofVec4f tempVel = ofVec4f(vel.x, vel.y, vel.z, 0.0);    // 0 for w because we don't want transformation
			vel = tempVel*node.getGlobalTransformMatrix();
			quat *= node.getGlobalOrientation();



            // write to joint
            person->joints[jointName].confidence = confidence;
            person->joints[jointName].pos.target = pos;
            person->joints[jointName].quat = quat;
            person->joints[jointName].euler = euler;

            // only use velocity if we're confident, otherwise zero
            person->joints[jointName].vel.target = (confidence > 0.5) ? vel : ofVec3f(0);
        }

		else if (strstr(m.getAddress().c_str(), "/lost_user")) {
			// if person is deleted (user_lost) remove from map
			int user_id = m.getArgAsInt(0);
			ofLogWarning() << "Receiver::parseOsc delete person " << user_id;
			persons.erase(user_id);
		}

		else if (strstr(m.getAddress().c_str(), "/floorplane")) {
			floorQuat = ofQuaternion(m.getArgAsFloat(0), m.getArgAsFloat(1), m.getArgAsFloat(2), m.getArgAsFloat(3));
			floorQuat = floorQuat*node.getOrientationQuat();
		}

        // DONT DO SMOOTHING, SPRINGYNESS ETC. HERE SHOULD BE AT FIXED FPS EVERY FRAME, WHETHER DATA COMES IN OR NOT
    }

}


void Receiver::update(vector<Person::Ptr>& persons_global) {
    // return if not _enabled
    if(!_enabled) {
        _isConnected = 0;
        _numPeople = 0;
        persons.clear();
        oscReceiver = NULL;
        return;
    }


    // clear flag
    if(_isConnected > 0) _isConnected--;

    // check for Osc messages and update
    parseOsc();


    // delete dead persons
    for(auto it = persons.cbegin(); it != persons.cend(); ) {
        if(!it->second || it->second->alive_counter >= kill_frame_count) {
            persons.erase(it++);
        } else {
            ++it;
        }
    }


    // do smoothing, springyness etc.
    // iterate all persons
    for(auto&& pkv : persons) {
        auto person = pkv.second;

        if(person) {
            // incremment alive counter
            person->alive_counter++;

            // set person color
            static ofColor colors[] = { ofColor::red, ofColor::green, ofColor::blue };
            person->color = colors[_index-1];

            // iterate all joints.
            // first pass: smoothings, euler, springyness
            for(auto&& jkv : person->joints) {
                JointInfo& joint = jkv.second;

                // smoothings
                joint.pos.update(pos_smoothing);
                joint.vel.update(vel_smoothing);

                // now set speed as magnitude of SMOOTHED velocity
                joint.speed = joint.vel.current.length();

                // get euler
                joint.euler = joint.quat.getEuler();

                // springyness
                ofVec3f diff = (joint.pos.current - joint.springy_pos);
                joint.springy_vel = joint.springy_vel * (1 - spring_damping) + diff * spring_strength;
                joint.springy_pos += joint.springy_vel;
            }


            // iterate all joints
            // second pass: do vectors to parent
            // (do this in separate pass to above to make sure all joints have been smoothed first)
            for(auto&& jkv : person->joints) {
                string joint_name = jkv.first;
                string parent_name = Person::joint_parents[joint_name];
                JointInfo& joint = jkv.second;
                joint.vec = person->joints[parent_name].pos.current - joint.pos.current;
            }


            // add person to global list
            persons_global.push_back(person);

        } else {
            ofLogError() << "Receiver::update person == NULL";
        }
    }

    // set _numPeople
    _numPeople = persons.size();
}



void Receiver::drawGui() {
    string str_index = ofToString(_index);
    if(ImGui::CollapsingHeader(("Receiver " + str_index).c_str(), NULL, true, true)) {
        ImGui::Checkbox(("Enabled " + str_index).c_str(), &_enabled);
        if(ImGui::InputInt(("port " + str_index).c_str(), &_port, 1, 100)) initOsc();
        if(ImGui::SliderFloat3(("pos " + str_index).c_str(), _pos.getPtr(), -5, 5)) updateMatrix();
        if(ImGui::SliderFloat3(("rot " + str_index).c_str(), _rot.getPtr(), -180, 180)) updateMatrix();

        stringstream str;
        str << "Connected: " << (isConnected() ? "YES" : "NO") << endl;
        str << "Num People: " << _numPeople;
        ImGui::Text(str.str().c_str());
    }
}

void Receiver::updateMatrix() {
    node.setPosition(_pos);
    node.setOrientation(_rot);  // order should be ok, but needs checking
    // now node.getGlobalTransformMatrix() contains transformation matrix for the joints
}


void Receiver::saveToXml(ofXml& xml) const {

	xml.setTo("//Settings/Receivers");
	xml.addChild("receiver");
	xml.setTo("receiver[" + ofToString(_index - 1) + "]");
	xml.addValue("port", ofToString(_port));

	xml.addChild("pos");
	xml.setTo("pos");
	xml.addValue("x", ofToString(_pos.x));
	xml.addValue("y", ofToString(_pos.y));
	xml.addValue("z", ofToString(_pos.z));

	xml.setToParent();
	xml.addChild("rot");
	xml.setTo("rot");
	xml.addValue("x", ofToString(_rot.x));
	xml.addValue("y", ofToString(_rot.y));
	xml.addValue("z", ofToString(_rot.z));
	
}

void Receiver::loadFromXml(ofXml& xml) {
    
	xml.setTo("//Settings/Receivers");
	pos_smoothing = xml.getFloatValue("pos_smoothing");
	vel_smoothing = xml.getFloatValue("vel_smoothing");
	spring_strength = xml.getFloatValue("spring_strength");
	spring_damping = xml.getFloatValue("spring_damping");
	kill_frame_count = xml.getIntValue("kill_frame_count");


	xml.setTo("receiver[" + ofToString(_index - 1) + "]");
	
	_pos = ofVec3f(xml.getFloatValue("pos/x"), xml.getFloatValue("pos/y"), xml.getFloatValue("pos/z"));
	_rot = ofVec3f(xml.getFloatValue("rot/x"), xml.getFloatValue("rot/y"), xml.getFloatValue("rot/z"));
	updateMatrix();

	if (xml.getIntValue("port") != _port) {
		_port = xml.getIntValue("port");
		initOsc();
	}

}




}
