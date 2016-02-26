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
    float confidence = 0;       // 0..1 confidence rating
    Smoother<ofVec3f> pos;      // world position, with smoothing
    ofQuaternion quat;          // world orientation as quat
    ofVec3f euler;              // world orienation as euler
    Smoother<ofVec3f> vel;      // velocity vector, with smoothing
    float speed = 0;            // speed, with smoothing
    ofVec3f vec;                // vector to parent
    ofVec3f springy_pos;        // positions with springy behaviour applied
    ofVec3f springy_vel;        // velocities with springy behaviour applied
};


struct Person {
	typedef shared_ptr<Person> Ptr;
    
    // useful for debugging
    string name = "Unknown person";

    // keep alive for XXX frames
    int alive_counter = 0;
    
    // which receiver this came from
    int receiver_id = -1;

    // draw color of this person (gonna color persons from kinect 1 red, kinect 2 green, kinect 3 blue etc)
    ofColor color = ofColor::yellow;

    // joint information
    map<string, JointInfo> joints;
    
    // how 'on axis' skeleton is (1: center, 0: towards edge)
//    float on_axis_amount = 1.0f;
    
    
    Person(string name):name(name) {
        for(const auto& joint_name : joint_names) joints[joint_name] = JointInfo();
    }
    
    

    // static joint info
    static vector<string> joint_names;
    static map<string, string> joint_parents;
    
    // load joint parent information from xml
    static void init_joint_parents() {
        ofLogNotice() << "Person::init_joint_parents";
        
        joint_names.clear();
        joint_parents.clear();
        
        // xml for loadin joint map info
        ofXml joints_xml("joints.xml");
        int numJoints = 25; // read from XML
        for (int i = 0; i < numJoints; i++) {
            string joint_name = joints_xml.getValue<string>("JointNames/joint[" + ofToString(i) + "]");
            
            joint_names.push_back(joint_name);
            joint_parents[joint_name] = joints_xml.getValue<string>("JointParents/parent[" + ofToString(i) + "]");
        }
    }
    
    // other info? hand states, lean, restrictedness etc

    
    // diffs to another person. useful for detecting if two people from different kinects are the same person or not
    // ignore for now

    // mean diff of joints
    static float dist(const Ptr p1, const Ptr p2) {
        if(joint_names.empty() || !p1 || !p2) return std::numeric_limits<float>::max();
        float s = 0;
        for(auto&& j : joint_names) s += (p1->joints[j].pos.current - p2->joints[j].pos.current).length();
        s /= joint_names.size();
        return s;
    }


    // mean diff squared of joints
    static float dist2(const Ptr p1, const Ptr p2) {
        if(joint_names.empty() || !p1 || !p2) return std::numeric_limits<float>::max();
        float s = 0;
        for(auto&& j : joint_names) s += (p1->joints[j].pos.current - p2->joints[j].pos.current).lengthSquared();
        s /= joint_names.size();
        return s;
    }


    // used for sorting persons left to right using waist position
    static bool compare(Ptr a, Ptr b) { return a->joints["waist"].pos.current.x < b->joints["waist"].pos.current.x; }
    
    
    // calculate how aligned person is to this axis
    static float on_axis(Ptr p, const ofNode& node) {
        if(!p || p->joints.empty()) return 1;
        ofVec3f avg_pos(0, 0, 0);
        for(auto&& j : p->joints) avg_pos += j.second.pos.current;
        avg_pos /= p->joints.size();
        
        // node to avg_pos
        ofVec3f dir(node.getGlobalPosition() - avg_pos);
        dir.normalize();
        
        return dir.dot(node.getLookAtDir());
        
    }
    
    
    // draw person
    static void draw(Ptr p, float joint_radius, bool show_target_pos, bool show_springy_pos, bool show_vel, float vel_mult) {
        if(!p) return;
        
       // iterate joints
       for(auto&& jkv : p->joints) {
           JointInfo& joint = jkv.second;

           ofSetColor(p->color);

           // draw limb
           ofDrawLine(joint.pos.current, joint.pos.current + joint.vec);

           // draw target (non smoothed joint)
           if(show_target_pos) ofDrawSphere(joint.pos.target, joint_radius);

           // draw springy_pos
           if(show_springy_pos) ofDrawSphere(joint.springy_pos, joint_radius);

           //  draw joint (Faded if showing target or springypos)
           ofSetColor(p->color);//, show_target_pos || show_springy_pos ? 50 : 255);
           ofDrawSphere(joint.pos.current, joint_radius);


           // draw velocity vector
           if(show_vel) {
               ofSetColor(255);
               ofDrawArrow(joint.pos.current, joint.pos.current + joint.vel.current * vel_mult, 0.02f);
           }
       }
    }
};


}

