
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
        _isConnected = true;

		ofxOscMessage m;
		oscReceiver->getNextMessage(m);
		if (strstr(m.getAddress().c_str(), "/skel/")) {
			// should get something like splitAddress = ["", "skel", "0", "c_shoulder"]
			vector<string> splitAddress = ofSplitString(m.getAddress(), "/");

            // assume we're parsing person with id == user_id
            int user_id = ofToInt(splitAddress[2]);

            bool do_delete = false;

            // if person is deleted (user_lost) remove from map
            if(do_delete) {
                ofLogWarning() << "Receiver::parseOsc delete person " << user_id;
                persons.erase(user_id);
            }

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



            // write to joint
            person->joints[jointName].confidence = confidence;
            person->joints[jointName].pos.target = pos;
            person->joints[jointName].quat = quat;
            person->joints[jointName].euler = euler;

            // only use velocity if we're confident, otherwise zero
            person->joints[jointName].vel.target = (confidence > 0.5) ? vel : ofVec3f(0);
        }

        // DONT DO SMOOTHING, SPRINGYNESS ETC. HERE SHOULD BE AT FIXED FPS EVERY FRAME, WHETHER DATA COMES IN OR NOT
    }

}


void Receiver::update(vector<Person::Ptr>& persons_global) {
    // return if not _enabled
    if(!_enabled) {
        _isConnected = false;
        _numPeople = 0;
        persons.clear();
        oscReceiver = NULL;
        return;
    }


    // clear flag
    _isConnected = false;

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
                JointInfo& joint = jkv.second;
                string parentJointName; // TODO: read from list?
                joint.vec = person->joints[parentJointName].pos.current - joint.pos.current;
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
    ImGui::CollapsingHeader(("Receiver " + ofToString(_index)).c_str(), NULL, true, true);
    ImGui::Checkbox("Enabled", &_enabled);
    if(ImGui::InputInt("port", &_port, 1, 100)) initOsc();
    if(ImGui::SliderFloat3("pos", _pos.getPtr(), -5, 5)) updateMatrix();
    if(ImGui::SliderFloat3("rot", _rot.getPtr(), -180, 180)) updateMatrix();

    stringstream str;
    str << "Connected: " << (_isConnected ? "YES" : "NO") << endl;
    str << "Num People: " << _numPeople;
    ImGui::Text(str.str().c_str());
}

void Receiver::updateMatrix() {
    node.setPosition(_pos);
    node.setOrientation(_rot);  // order should be ok, but needs checking
    // now node.getGlobalTransformMatrix() contains transformation matrix for the joints
}


void Receiver::saveToXml(ofXml& xml) const {
    // TODO: write to xml, under 'Receiver_index' section
    // - port
    // - pos
    // - rot
}

void Receiver::loadFromXml(ofXml& xml) {
    // TODO: ead from xml, under 'Receiver_index' section
    // - port
    // - pos
    // - rot
    // if port loaded is different to current -> initOsc();
}




}
