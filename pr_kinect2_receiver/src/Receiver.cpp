
#include "Receiver.h"
#include "ofxImGui.h"

namespace pr {

float Receiver::posSmoothing = 0.5;
float Receiver::velSmoothing = 0.96;
float Receiver::springStrength = 0.02;
float Receiver::springDamping = 0.05;
int Receiver::killFrameCount = 10;      // kill person afer this many frames of not receiving


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

        // assume we're parsing person with id == user_id
        int user_id = 0;

        bool do_delete = false;

        // if person is deleted (user_lost) remove from map
        if(do_delete) {
            ofLogWarning() << "Receiver::parseOsc delete person " << user_id;
            persons.erase(user_id);
        }

        // this is the person we're receiving info for
        Person::Ptr person = persons[user_id];

        // if new user found and calibrated, add to map
        if(!persons[user_id]) {
            ofLogWarning() << "Receiver::parseOsc creating person " << user_id;
            persons[user_id] = make_shared<Person>();
        }

        // whether it's new user or existing user, update joint details

        // reset alive counter
        person->aliveCounter = 0;

        // read from osc:
        string jointName;
        float confidence;
        ofVec3f pos;
        ofQuaternion quat;
        ofVec3f euler;
        ofVec3f vel;
        //        float speed;  // DON"T READ SPEED FROM OSC


        // apply world transformation to pos, quat and vel



        // write to joint
        person->joints[jointName].confidence = confidence;
        person->joints[jointName].pos.target = pos;
        person->joints[jointName].quat = quat;
        person->joints[jointName].euler = euler;

        // only use velocity if we're confident, otherwise zero
        person->joints[jointName].vel.target = (confidence > 0.5) ? vel : ofVec3f(0);

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
        if(!it->second || it->second->aliveCounter >= killFrameCount) {
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
            person->aliveCounter++;

            // iterate all joints.
            // first pass: smoothings, euler, springyness
            for(auto&& jkv : person->joints) {
                JointInfo& joint = jkv.second;

                // smoothings
                joint.pos.update(posSmoothing);
                joint.vel.update(velSmoothing);

                // now set speed as magniture of SMOOTHED velocity
                joint.speed = joint.vel.current.length();

                // get euler
                joint.euler = joint.quat.getEuler();

                // springyness
                ofVec3f diff = (joint.pos.current - joint.springy_pos);
                joint.springy_vel = joint.springy_vel * (1 - springDamping) + diff * springStrength;
                joint.springy_pos += joint.springy_vel;
            }


            // iterate all joints
            // second pass: do vectors to parent
            // (do this in separate pass to above to make sure all joints have been smoothed first)
            for(auto&& jkv : person->joints) {
                JointInfo& joint = jkv.second;
                string parentJointName; // read from list?
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


void Receiver::draw() {

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
    // write to xml, under 'Receiver_index' section
    // - port
    // - pos
    // - rot
}

void Receiver::loadFromXml(ofXml& xml) {
    // read from xml, under 'Receiver_index' section
    // - port
    // - pos
    // - rot
    // if port loaded is different to current -> initOsc();
}




}
