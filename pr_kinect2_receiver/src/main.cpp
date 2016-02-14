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
    pr::OscSender osc_sender;

    // display params
    struct {
        bool show_floor = true;
        float floor_size = 12;
        bool show_all_persons = true;
        bool show_reduced_persons = true;
        float joint_radius = 0.2;
        bool show_target_pos = false;
        bool show_springy_pos = false;
        bool show_vel = false;
        float vel_mult = 1.0;

    } display;

    ofEasyCam cam;
	ofPlanePrimitive floorPlane;


    //--------------------------------------------------------------
    void setup() {
        ofBackground(0);
        ofSetVerticalSync(true);
        ofSetFrameRate(30);

        // init and allocate 3x receivers (because we have 3x kinects - this number can be hardcoded for now)
        receivers.resize(3);
        for(int i=0; i<receivers.size(); i++) receivers[i] = shared_ptr<pr::Receiver>(new pr::Receiver(i+1));

		osc_sender.setup("127.0.0.1", 8000);

        // init number of final persons to 3 (completely coincidence that we have 3 receivers as well)
        persons_global_reduced.resize(kPersonCount);


        loadFromXml(kXmlFilename);

        cam.setPosition(0, 1.5, -5);
        cam.lookAt(ofVec3f(0, 1.5, -4), ofVec3f(0, 1, 0));
		cam.setDistance(10);

		floorPlane.set(display.floor_size, display.floor_size);
		floorPlane.rotate(90, 1, 0, 0);
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

        osc_sender.loadFromXml(xml);

		xml.setTo("//Settings/Display");
		display.show_floor = xml.getBoolValue("show_floor");
		display.floor_size = xml.getFloatValue("floor_size");
		display.show_all_persons = xml.getBoolValue("show_all_persons");
		display.show_reduced_persons = xml.getBoolValue("show_reduced_persons");
		display.joint_radius = xml.getFloatValue("joint_radius");
		display.show_target_pos = xml.getBoolValue("show_target_pos");
		display.show_springy_pos = xml.getBoolValue("show_springy_pos");
		display.show_vel = xml.getBoolValue("show_vel");
		display.vel_mult = xml.getFloatValue("vel_mult");
    }

    //--------------------------------------------------------------
    void saveToXml(string filename) {

        ofXml xml;

		xml.addChild("Settings");
		xml.setTo("Settings");
		xml.addChild("Display");
		xml.setTo("Display");

		xml.addValue("show_floor", ofToString(display.show_floor));
		xml.addValue("floor_size", ofToString(display.floor_size));
		xml.addValue("show_all_persons", ofToString(display.show_all_persons));
		xml.addValue("show_reduced_persons", ofToString(display.show_reduced_persons));
		xml.addValue("joint_radius", ofToString(display.joint_radius));
		xml.addValue("show_target_pos", ofToString(display.show_target_pos));
		xml.addValue("show_springy_pos", ofToString(display.show_springy_pos));
		xml.addValue("show_vel", ofToString(display.show_vel));
		xml.addValue("vel_mult", ofToString(display.vel_mult));

		xml.setTo("//Settings");
		xml.addChild("Receivers");
		xml.setTo("Receivers");

		xml.addValue("pos_smoothing", ofToString(receivers[0]->pos_smoothing));
		xml.addValue("vel_smoothing", ofToString(receivers[0]->vel_smoothing));
		xml.addValue("spring_strength", ofToString(receivers[0]->spring_strength));
		xml.addValue("spring_damping", ofToString(receivers[0]->spring_damping));
		xml.addValue("kill_frame_count", ofToString(receivers[0]->kill_frame_count));

		// tell receivers to write the settings to xml to be saved
		for (auto&& receiver : receivers) {
			if (receiver) receiver->saveToXml(xml);
			else ofLogError() << "App::saveToXml receiver == NULL";
		}

		osc_sender.saveToXml(xml);

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

        // if at least one person
        if(persons_global_all.size() > 1) {
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
                ofLogError() << "App::update one or more persons == NULL";
            }
        } else {
            // if no one exists, don't send any person data
            persons_global_reduced.clear();
        }

        // send osc
        sendOsc();
    }


    //--------------------------------------------------------------
    void sendOsc() {
        ofxOscBundle b;

        // send metadata
        ofxOscMessage m;
        m.setAddress("/Meta");
        m.addInt32Arg(persons_global_reduced.size());
        b.addMessage(m);

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
        osc_sender.sendBundle(b);
    }



    //--------------------------------------------------------------
    void draw() {

        cam.begin();

        if(display.show_floor) {
			ofPushStyle();
				ofSetColor(128);
				ofNoFill();
				floorPlane.draw();
				ofDrawAxis(1.0);
			ofPopStyle();
			
        }

        if(display.show_all_persons) {
            for(auto&& person: persons_global_all) {
                if(person) person->draw(display.joint_radius, display.show_target_pos, display.show_springy_pos, display.show_vel, display.vel_mult);
            }
        }

        if(display.show_reduced_persons) {
            for(auto&& person: persons_global_reduced) {
                if(person) person->draw(display.joint_radius, display.show_target_pos, display.show_springy_pos, display.show_vel, display.vel_mult);
            }
        }

        cam.end();

        drawGui();
    }


    //--------------------------------------------------------------
    void drawGui() {

        gui.begin();
        //        ImGui::ShowWindow("PR_PERSONS_RECEIVER", null, true, true);

        ImGui::CollapsingHeader("Global Params", NULL, true, true);
        ImGui::SliderFloat("pos smoothing", &pr::Receiver::pos_smoothing, 0, 1);
        ImGui::SliderFloat("vel smoothing", &pr::Receiver::vel_smoothing, 0, 1);
        ImGui::SliderFloat("spring strength", &pr::Receiver::spring_strength, 0, 1);
        ImGui::SliderFloat("spring damping", &pr::Receiver::spring_damping, 0, 1);
        ImGui::SliderInt("kill frame count", &pr::Receiver::kill_frame_count, 0, 1);

        for(auto&& receiver : receivers) {
            if(receiver) receiver->drawGui();
            else ofLogError() << "App::drawGui receiver == NULL";
        }

        osc_sender.drawGui();

        // display params
        ImGui::CollapsingHeader("Display params", NULL, true, true);
        ImGui::Checkbox("show floor", &display.show_floor);
        ImGui::SliderFloat("floor size", &display.floor_size, 5, 20);
        ImGui::Checkbox("show all persons", &display.show_all_persons);
        ImGui::Checkbox("show reduced persons", &display.show_reduced_persons);

        ImGui::SliderFloat("jointRadius", &display.joint_radius, 0, 1);
        ImGui::Checkbox("show target pos", &display.show_target_pos);
        ImGui::Checkbox("show springy pos", &display.show_springy_pos);
        ImGui::Checkbox("show vel vector", &display.show_vel);
        ImGui::SliderFloat("vel mult", &display.vel_mult, 0, 5);

        // show stats
        ImGui::CollapsingHeader("Global Stats", NULL, true, true);
        stringstream str;
        str << "Total persons: " << persons_global_all.size() << endl;
        str << "fps: " << ofGetFrameRate();
        ImGui::Text(str.str().c_str());


        gui.end();
    }


    //--------------------------------------------------------------
    void keyPressed(int key) {
        switch(key) {
        case 'l' :loadFromXml(kXmlFilename); break;
        case 's' :saveToXml(kXmlFilename); break;
		case 'v': {
			cam.setPosition(0, 1.5, -5);
			cam.lookAt(ofVec3f(0, 1.5, -4), ofVec3f(0, 1, 0));
			cam.setDistance(10);
			break;
		}
        }
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
