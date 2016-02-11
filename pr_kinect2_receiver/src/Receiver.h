/*

 Receives and manages data coming from a single tracker
 - transform into global space

 */

#pragma once

#include "ofxOscReceiver.h"
#include "Person.h"

namespace pr {

class Receiver {
public:
    typedef shared_ptr<Receiver> Ptr;

    static float posSmoothing;
    static float velSmoothing;
    static float springStrength;
    static float springDamping;
    static int killFrameCount;


    Receiver(int i) { _index = i; }

    // pass in global (i.e. containing all persons from all receivers) vector to update
    void update(vector<Person::Ptr>& persons_global);

    void draw();
    void drawGui();

    bool isConnected() const    { return _isConnected; }
    int numPeople() const       { return _numPeople; }

    void saveToXml(ofXml& xml) const;
    void loadFromXml(ofXml& xml);

protected:
    bool _enabled = true;
    int _index;         // 1, 2, 3 etc. (starting at 1, not 0)
    int _port = 0;      // port to listen on
    ofVec3f _pos;       // world position of sensor
    ofVec3f _rot;       // world orientation (degrees) of sensor

    bool _isConnected;  // is receiving any signal from Tracker
    int _numPeople;      // current number of people on that Tracker

    ofNode node;        // contains transformation matrix of kinect (for transformming joints)

    map<int, Person::Ptr> persons; // all current Persons, using a map<int> instead of vector, because the id's aren't nessecary sequential.

    // receives osc
    unique_ptr<ofxOscReceiver> oscReceiver;

    void initOsc();
    void parseOsc();
    void updateMatrix();
};

}
