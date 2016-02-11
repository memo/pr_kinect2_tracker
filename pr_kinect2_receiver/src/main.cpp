#include "ofMain.h"

#include "Person.h"
#include "Receiver.h"
#include "ofxImGui.h"
#include "OscSender.h"

#define kPersonAvg      0
#define kPersonLeft     1
#define kPersonRight    2
#define kPersonCount    3

#define kXmlFilename    "settings.xml"

class ofApp : public ofBaseApp {

    // the receivers
    vector<pr::Receiver::Ptr> receivers;

    // the persons
    vector<pr::Person::Ptr> persons_global_reduced; // list of final, condensed persons
    vector<pr::Person::Ptr> persons_global_all;     // list of all persons from all receivers

    // for gui;
    ofxImGui gui;

    // osc
    pr::OscSender oscSender;


    //--------------------------------------------------------------
    void setup() {
        ofBackground(0);
        ofSetVerticalSync(true);
        ofSetFrameRate(30);

        // init and allocate 3x receivers (because we have 3x kinects - this number can be hardcoded for now)
        receivers.resize(3);
        for(int i=0; i<receivers.size(); i++) receivers[i] = shared_ptr<pr::Receiver>(new pr::Receiver(i+1));


        // init number of final persons to 3 (completely coincidence that we have 3 receivers as well)
        persons_global_reduced.resize(kPersonCount);


        loadFromXml(kXmlFilename);
    }


    //--------------------------------------------------------------
    void loadFromXml(string filename) {

        // load xmml
        ofXml xml(filename);

        // tell receivers to fetch their settings from loaded xml
        for(auto&& receiver : receivers) {
            if(receiver) receiver->loadFromXml(xml);
            else ofLogError() << "App::loadFromXml receiver == NULL";
        }

        oscSender.loadFromXml(xml);

        // also load other settings
        //        float Receiver::posSmoothing = 0.5;
        //        float Receiver::velSmoothing = 0.96;
        //        float Receiver::springStrength = 0.02;
        //        float Receiver::springDamping = 0.05;
        //        int Receiver::killFrameCount = 10;      // kill person afer this many frames of not receiving


    }



    //--------------------------------------------------------------
    void saveToXml(string filename) {

        ofXml xml;

        // tell receivers to write the settings to xml to be saved
        for(auto&& receiver : receivers) {
            if(receiver) receiver->saveToXml(xml);
            else ofLogError() << "App::saveToXml receiver == NULL";
        }

        oscSender.saveToXml(xml);

        // also write other settings to xml
        //        float Receiver::posSmoothing = 0.5;
        //        float Receiver::velSmoothing = 0.96;
        //        float Receiver::springStrength = 0.02;
        //        float Receiver::springDamping = 0.05;
        //        int Receiver::killFrameCount = 10;      // kill person afer this many frames of not receiving

        //        hostIp;
        //        hostPort;

        // save xml
        xml.save(filename);
    }



    //--------------------------------------------------------------
    void update() {

        // clear all persons list
        persons_global_all.clear();

        // first pass, parse all waiting osc, process receivers and add to global list of persons
        for(auto&& receiver : receivers) {
            if(receiver) receiver->update(persons_global_all);
            else ofLogError() << "App::update receiver == NULL";
        }


        // sort global list
        std::sort(persons_global_all.begin(), persons_global_all.end(), pr::Person::compare);


        // calculate average person (allocate first if nessecary)
        if(!persons_global_reduced[kPersonAvg]) persons_global_reduced[kPersonAvg] = make_shared<pr::Person>();

        if(persons_global_reduced[kPersonAvg] && persons_global_reduced[kPersonLeft] && persons_global_reduced[kPersonRight]) {
            for(auto&& jkv : persons_global_reduced[kPersonAvg]->joints) {
                pr::JointInfo& joint = jkv.second;
                string jointName = jkv.first;
                joint.confidence    = (persons_global_reduced[kPersonLeft]->joints[jointName].confidence  + persons_global_reduced[kPersonRight]->joints[jointName].confidence)/2;
                joint.pos.current   = (persons_global_reduced[kPersonLeft]->joints[jointName].pos.current + persons_global_reduced[kPersonRight]->joints[jointName].pos.current)/2;
                joint.quat          = (persons_global_reduced[kPersonLeft]->joints[jointName].quat        + persons_global_reduced[kPersonRight]->joints[jointName].quat)/2;
                joint.euler         = (persons_global_reduced[kPersonLeft]->joints[jointName].euler       + persons_global_reduced[kPersonRight]->joints[jointName].euler)/2;
                joint.vel.current   = (persons_global_reduced[kPersonLeft]->joints[jointName].vel.current + persons_global_reduced[kPersonRight]->joints[jointName].vel.current)/2;
                joint.speed         = (persons_global_reduced[kPersonLeft]->joints[jointName].speed       + persons_global_reduced[kPersonRight]->joints[jointName].speed)/2;
                joint.vec           = (persons_global_reduced[kPersonLeft]->joints[jointName].vec         + persons_global_reduced[kPersonRight]->joints[jointName].vec)/2;
                joint.springy_pos   = (persons_global_reduced[kPersonLeft]->joints[jointName].springy_pos + persons_global_reduced[kPersonRight]->joints[jointName].springy_pos)/2;
                joint.springy_vel   = (persons_global_reduced[kPersonLeft]->joints[jointName].springy_vel + persons_global_reduced[kPersonRight]->joints[jointName].springy_vel)/2;
            }
        } else {
            ofLogError() << "App::update one of more persons == NULL";
        }

        sendOsc();
    }


    //--------------------------------------------------------------
    void sendOsc() {
        ofxOscBundle b;
        int i=0;
        for(auto&& person: persons_global_reduced) {
            if(person) {
                for(auto&& jkv : person->joints) {
                    pr::JointInfo& joint = jkv.second;
                    string jointName = jkv.first;

                    ofxOscMessage m;
                    m.setAddress("/Skel/" + ofToString(i) + "/" + jointName);
                    m.addFloatArg(joint.pos.current.x);
                    m.addFloatArg(joint.pos.current.y);
                    m.addFloatArg(joint.pos.current.z);

                    m.addFloatArg(joint.quat._v.x);
                    m.addFloatArg(joint.quat._v.y);
                    m.addFloatArg(joint.quat._v.z);
                    m.addFloatArg(joint.quat._v.w);

                    m.addFloatArg(joint.euler.x);
                    m.addFloatArg(joint.euler.y);
                    m.addFloatArg(joint.euler.z);

                    m.addFloatArg(joint.vel.current.x);
                    m.addFloatArg(joint.vel.current.y);
                    m.addFloatArg(joint.vel.current.z);

                    m.addFloatArg(joint.speed);

                    m.addFloatArg(joint.vec.x);
                    m.addFloatArg(joint.vec.y);
                    m.addFloatArg(joint.vec.z);

                    m.addFloatArg(joint.springy_pos.x);
                    m.addFloatArg(joint.springy_pos.y);
                    m.addFloatArg(joint.springy_pos.z);

                    m.addFloatArg(joint.springy_vel.x);
                    m.addFloatArg(joint.springy_vel.y);
                    m.addFloatArg(joint.springy_vel.z);

                    b.addMessage(m);
                }
            } else {
                ofLogError() << "App::sendOsc person == NULL";
            }
            i++;
        }
        oscSender.sendBundle(b);
    }



    //--------------------------------------------------------------
    void draw() {
        drawGui();
    }


    //--------------------------------------------------------------
    void drawGui() {

        gui.begin();
        //        ImGui::ShowWindow("PR_PERSONS_RECEIVER", null, true, true);

        ImGui::CollapsingHeader("Global Params", NULL, true, true);
        ImGui::SliderFloat("pos smoothing", &pr::Receiver::posSmoothing, 0, 1);
        ImGui::SliderFloat("vel smoothing", &pr::Receiver::velSmoothing, 0, 1);
        ImGui::SliderFloat("spring strength", &pr::Receiver::springStrength, 0, 1);
        ImGui::SliderFloat("spring damping", &pr::Receiver::springDamping, 0, 1);
        ImGui::SliderInt("kill frame count", &pr::Receiver::killFrameCount, 0, 1);

        for(auto&& receiver : receivers) {
            if(receiver) receiver->drawGui();
            else ofLogError() << "App::drawGui receiver == NULL";
        }

        oscSender.drawGui();

        ImGui::CollapsingHeader("Global Stats", NULL, true, true);

        stringstream str;
        str << "Total persons: " << persons_global_all.size() << endl;
        str << "fps: " << ofGetFrameRate();
        ImGui::Text(str.str().c_str());


        gui.end();
    }


    //--------------------------------------------------------------
    void keyPressed(int key) {

    }

    //--------------------------------------------------------------
    void keyReleased(int key) {

    }

    //--------------------------------------------------------------
    void mouseMoved(int x, int y ) {

    }

    //--------------------------------------------------------------
    void mouseDragged(int x, int y, int button) {

    }

    //--------------------------------------------------------------
    void mousePressed(int x, int y, int button) {

    }

    //--------------------------------------------------------------
    void mouseReleased(int x, int y, int button) {

    }

    //--------------------------------------------------------------
    void mouseEntered(int x, int y) {

    }

    //--------------------------------------------------------------
    void mouseExited(int x, int y) {

    }

    //--------------------------------------------------------------
    void windowResized(int w, int h) {

    }

    //--------------------------------------------------------------
    void gotMessage(ofMessage msg) {

    }

    //--------------------------------------------------------------
    void dragEvent(ofDragInfo dragInfo) {

    }

};

//========================================================================
int main( ) {
    ofSetupOpenGL(1600,900,OF_WINDOW);			// <-------- setup the GL context

    // this kicks off the running of my app
    // can be OF_WINDOW or OF_FULLSCREEN
    // pass in width and height too:
    ofRunApp(new ofApp());

}
