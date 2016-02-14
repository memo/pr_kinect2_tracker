/*
 Stores data for a single person
 (could also rip out the one from the Kinect2 addons)
 */

#pragma once

#include "ofMain.h"

namespace pr {

// helper class for smoothing parameters
template<typename T> struct Smoother {
    T current, target;
    void update(float smoothing) { current += (target - current) * (1 - smoothing); }

};


struct JointInfo {
    float confidence;       // 0..1 confidence rating
    Smoother<ofVec3f> pos;  // world position, with smoothing
    ofQuaternion quat;      // world orientation as quat
    ofVec3f euler;          // world orienation as euler
    Smoother<ofVec3f> vel;  // velocity vector, with smoothing
    float speed;            // speed, with smoothing
    ofVec3f vec;            // vector to parent
    ofVec3f springy_pos;    // positions with springy behaviour applied
    ofVec3f springy_vel;    // velocities with springy behaviour applied
};


struct Person {

	typedef shared_ptr<Person> Ptr;

    // keep alive for XXX frames
    int alive_counter = 0;

    // draw color of this person (gonna color persons from kinect 1 red, kinect 2 green, kinect 3 blue etc)
    ofColor color = ofColor::yellow;

    // joint information
    map<string, JointInfo> joints;

	// xml for loadin joint map info
	ofXml jointXml;

	Person() {
		// total number of tracked joints
		int numJoints = 25;

		jointXml.load("joints.xml");
		jointXml.setTo("//JointNames");

		for (int i = 0; i < numJoints; i++) {
			string tempJointName = jointXml.getValue<string>("joint" + ofToString(i));
			joints[tempJointName] = JointInfo();
		}
	};

    // other info? hand states, lean, restrictedness etc


    /*
    // diffs to another person. useful for detecting if two people from different kinects are the same person or not
    // ignore for now

    // L1 norm of diffs to another person
    float dist(Ptr other) {
        float s = 0;
        for(auto&& j : joints) s += (j.second.pos.current - other->joints[j.first].pos.current).length();
        return s;
    }


    // L2 norm of diffs to another person
    float dist2(Ptr other) {
        float s = 0;
        for(auto&& j : joints) s += (j.second.pos.current - other->joints[j.first].pos.current).lengthSquared();
        return sqrt(s);
    }
*/

    // used for sorting persons left to right using waist position
    static bool compare(Ptr a, Ptr b) { return a->joints["waist"].pos.current.x < b->joints["waist"].pos.current.x; }


    // draw person
    void draw(float joint_radius, bool show_target_pos, bool show_springy_pos, bool show_vel, float vel_mult) {

       // iterate joints
       for(auto&& jkv : joints) {
           JointInfo& joint = jkv.second;

           ofSetColor(color);

           // draw limb
           ofDrawLine(joint.pos.current, joint.pos.current + joint.vec);

           // draw target (non smoothed joint)
           if(show_target_pos) ofDrawSphere(joint.pos.target, joint_radius);

           // draw springy_pos
           if(show_springy_pos) ofDrawSphere(joint.springy_pos, joint_radius);

           //  draw joint (Faded if showing target or springypos)
           ofSetColor(color, show_target_pos || show_springy_pos ? 128 : 255);
           ofDrawSphere(joint.pos.current, joint_radius);


           // draw velocity vector
           if(show_vel) {
               ofSetColor(255);
               ofDrawArrow(joint.pos.current, joint.pos.current + joint.vel.current * vel_mult);
           }
       }
    }
};


}

