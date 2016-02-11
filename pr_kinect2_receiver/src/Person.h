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
    int aliveCounter = 0;

    // joint information
    map<string, JointInfo> joints;

    // other info? hand states, lean, restrictedness etc


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


    // used for sorting persons left to right usinng waist position
    static bool compare(Ptr a, Ptr b) { return a->joints["waist"].pos.current.x < b->joints["waist"].pos.current.x; }
};


}
