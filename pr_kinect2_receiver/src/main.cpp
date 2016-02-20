#include "ofMain.h"

#include "Person.h"
#include "Receiver.h"
#include "ofxImGui.h"
#include "OscSender.h"
#include "ml.h"

#define kPersonAvg      0
#define kPersonLeft     1
#define kPersonRight    2
#define kPersonML       3
#define kPersonCount    4

#define kXmlFilename    "settings.xml"

map<string, string> pr::Person::joint_parents;
vector<string> pr::Person::joint_names;

class ofApp : public ofBaseApp {
    
    // params
    struct {
        
        bool do_avg_person = false;
        
        // display params
        struct {
            bool show_floor = true;
            bool show_kinect_floors = false;
            ofVec2f floor_size = { 10, 8 };
            ofVec3f floor_pos = { 0, 0, -4 };
            bool show_received_persons = true;
            bool show_final_persons = true;
            bool show_avg_person = false;
            float joint_radius = 0.2;
            bool show_target_pos = false;
            bool show_springy_pos = false;
            bool show_vel = false;
            float vel_mult = 1.0;
        } display;
    } params;
    
    ofEasyCam cam;
    ofPlanePrimitive floorPlane;
    
    
    // the receivers
    vector<pr::Receiver::Ptr> receivers;
    
    // the persons
    vector<pr::Person::Ptr> persons_global_final; // list of final, condensed persons
    vector<pr::Person::Ptr> persons_global_all;     // list of all persons from all receivers
    
    // for gui;
    ofxImGui gui;
    
    // osc
    pr::OscSender osc_sender;
    
    pr::ml::Manager ml;
    
    
    //--------------------------------------------------------------
    void setup() {
        msa::vector_utils::test_2d<vector<float>, float>();
        msa::vector_utils::test_flat<vector<float>, float>();
        
        ofBackground(0);
        ofSetVerticalSync(true);
        ofSetFrameRate(30);
        
        pr::Person::init_joint_parents();
        
        // init and allocate 3x receivers (because we have 3x kinects - this number can be hardcoded for now)
        receivers.resize(3);
        for(int i=0; i<receivers.size(); i++) receivers[i] = shared_ptr<pr::Receiver>(new pr::Receiver(i+1));
        
        //		osc_sender.setup("127.0.0.1", 8000);
        
        loadFromXml(kXmlFilename);
        
        cam.setPosition(0, 2.5, 10);
        cam.lookAt(params.display.floor_pos, ofVec3f(0, 1, 0));
        cam.setDistance(10);
        cam.setTarget(params.display.floor_pos);
        cam.setNearClip(0.1);
        
        floorPlane.set(params.display.floor_size.x, params.display.floor_size.y, params.display.floor_size.x+1, params.display.floor_size.y+1);
        floorPlane.setPosition(params.display.floor_pos);
        floorPlane.rotate(90, 1, 0, 0);
        
        ofSetWindowPosition(0, 0);
        ofSetWindowShape(ofGetScreenWidth(), ofGetScreenWidth());
    }
    
    
    //--------------------------------------------------------------
    void loadFromXml(string filename) {
        
        // load xml
        ofXml xml(filename);
        
        // tell receivers to fetch their settings from loaded xml
        for(auto&& receiver : receivers) {
            if(receiver) receiver->loadFromXml(xml);
            else ofLogError() << "App::loadFromXml receiver == NULL";
        }
        
        osc_sender.loadFromXml(xml);
        
        xml.setTo("//Settings/Display");
        params.do_avg_person = xml.getBoolValue("do_avg_person");
        params.display.show_floor = xml.getBoolValue("show_floor");
        params.display.show_kinect_floors = xml.getBoolValue("show_kinect_floors");
        //		params.display.floor_size = xml.getFloatValue("floor_size");
        params.display.show_received_persons = xml.getBoolValue("show_received_persons");
        params.display.show_final_persons = xml.getBoolValue("show_final_persons");
        params.display.show_avg_person = xml.getBoolValue("show_avg_person");
        params.display.joint_radius = xml.getFloatValue("joint_radius");
        params.display.show_target_pos = xml.getBoolValue("show_target_pos");
        params.display.show_springy_pos = xml.getBoolValue("show_springy_pos");
        params.display.show_vel = xml.getBoolValue("show_vel");
        params.display.vel_mult = xml.getFloatValue("vel_mult");
    }
    
    //--------------------------------------------------------------
    void saveToXml(string filename) {
        
        ofXml xml;
        
        xml.addChild("Settings");
        xml.setTo("Settings");
        xml.addChild("Display");
        xml.setTo("Display");
        
        xml.addValue("do_avg_person", ofToString(params.do_avg_person));
        xml.addValue("show_floor", ofToString(params.display.show_floor));
        xml.addValue("show_kinect_floors", ofToString(params.display.show_kinect_floors));
        //		xml.addValue("floor_size", ofToString(params.display.floor_size));
        xml.addValue("show_received_persons", ofToString(params.display.show_received_persons));
        xml.addValue("show_final_persons", ofToString(params.display.show_final_persons));
        xml.addValue("show_avg_person", ofToString(params.display.show_avg_person));
        xml.addValue("joint_radius", ofToString(params.display.joint_radius));
        xml.addValue("show_target_pos", ofToString(params.display.show_target_pos));
        xml.addValue("show_springy_pos", ofToString(params.display.show_springy_pos));
        xml.addValue("show_vel", ofToString(params.display.show_vel));
        xml.addValue("vel_mult", ofToString(params.display.vel_mult));
        
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
        if(persons_global_all.size() > 0) {
            // sort global list
            std::sort(persons_global_all.begin(), persons_global_all.end(), pr::Person::compare);
            
            if(persons_global_final.size() != kPersonCount) {
                ofLogWarning() << "App::update persons_global_final.size() == " << persons_global_final.size() << ". Allocating";
                persons_global_final.resize(kPersonCount);
            }
            
            persons_global_final[kPersonLeft] = persons_global_all.front();
            persons_global_final[kPersonRight] = persons_global_all.back();
            
            if(params.do_avg_person) {
                // calculate average person (allocate first if nessecary)
                if(!persons_global_final[kPersonAvg]) persons_global_final[kPersonAvg] = make_shared<pr::Person>("average");
                
                if(persons_global_final[kPersonAvg] && persons_global_final[kPersonLeft] && persons_global_final[kPersonRight]) {
                    for(auto&& jkv : persons_global_final[kPersonAvg]->joints) {
                        pr::JointInfo& joint = jkv.second;
                        string jointName = jkv.first;
                        joint.confidence    = (persons_global_final[kPersonLeft]->joints[jointName].confidence  + persons_global_final[kPersonRight]->joints[jointName].confidence)/2;
                        joint.pos.current   = (persons_global_final[kPersonLeft]->joints[jointName].pos.current + persons_global_final[kPersonRight]->joints[jointName].pos.current)/2;
                        joint.quat          = (persons_global_final[kPersonLeft]->joints[jointName].quat        + persons_global_final[kPersonRight]->joints[jointName].quat)/2;
                        joint.euler         = (persons_global_final[kPersonLeft]->joints[jointName].euler       + persons_global_final[kPersonRight]->joints[jointName].euler)/2;
                        joint.vel.current   = (persons_global_final[kPersonLeft]->joints[jointName].vel.current + persons_global_final[kPersonRight]->joints[jointName].vel.current)/2;
                        joint.speed         = (persons_global_final[kPersonLeft]->joints[jointName].speed       + persons_global_final[kPersonRight]->joints[jointName].speed)/2;
                        joint.vec           = (persons_global_final[kPersonLeft]->joints[jointName].vec         + persons_global_final[kPersonRight]->joints[jointName].vec)/2;
                        joint.springy_pos   = (persons_global_final[kPersonLeft]->joints[jointName].springy_pos + persons_global_final[kPersonRight]->joints[jointName].springy_pos)/2;
                        joint.springy_vel   = (persons_global_final[kPersonLeft]->joints[jointName].springy_vel + persons_global_final[kPersonRight]->joints[jointName].springy_vel)/2;
                    }
                    
                    
                } else {
                    ofLogError() << "App::update one or more persons == NULL";
                }
            } else {
                persons_global_final[kPersonAvg] = nullptr;
            }
            
            
            // machine learning update
            ml.update(persons_global_final);
            
        } else {
            // if no one exists, don't send any person data
            persons_global_final.clear();
        }
        
        // send osc
        sendOsc();
    }
    
    
    //--------------------------------------------------------------
    void sendOsc() {
        ofxOscBundle b;
        
        // send metadata
        ofxOscMessage m;
        m.setAddress("/meta");
        m.addInt32Arg(persons_global_final.size());
        b.addMessage(m);
        
        int i=0;
        for(auto&& person: persons_global_final) {
            if(person) {
                for(auto&& jkv : person->joints) {
                    pr::JointInfo& joint = jkv.second;
                    string jointName = jkv.first;
                    
                    ofxOscMessage m;
                    m.setAddress("/skel/" + ofToString(i) + "/" + jointName);
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
                //                ofLogError() << "App::sendOsc person == NULL";
            }
            i++;
        }
        osc_sender.sendBundle(b);
    }
    
    
    
    //--------------------------------------------------------------
    void draw() {
        if(ImGui::IsPosHoveringAnyWindow(ofVec2f(ofGetMouseX(), ofGetMouseY()))) cam.disableMouseInput();
        else cam.enableMouseInput();
        
        
        cam.begin();
        
        if(params.display.show_floor) {
            ofPushStyle();
            ofSetColor(128);
            //ofNoFill();
            floorPlane.drawWireframe();
            ofDrawAxis(1.0);
            ofPopStyle();
        }
        
        if(params.display.show_received_persons) {
            for(auto&& person: persons_global_all) {
                if(person) person->draw(params.display.joint_radius, params.display.show_target_pos, params.display.show_springy_pos, params.display.show_vel, params.display.vel_mult);
            }
        }
        
        if(params.display.show_final_persons) {
            for(int i=0; i<persons_global_final.size(); i++) {
                if(params.display.show_avg_person || i != kPersonAvg) {
                    auto person = persons_global_final[i];
                    if(person) person->draw(params.display.joint_radius, params.display.show_target_pos, params.display.show_springy_pos, params.display.show_vel, params.display.vel_mult);
                }
            }
        }
        
        for (auto&& receiver : receivers) {
            if(receiver->isEnabled()) {
                // draw axis for kinect pos/rot
                receiver->getNode().transformGL();
                ofDrawAxis(0.5);
                receiver->getNode().restoreTransformGL();
                
                if (params.display.show_kinect_floors) {
                    ofPlanePrimitive k_floor_plane(4, 4, 4, 4);
                    //                    k_floor_plane.setWidth(4.0);
                    //                    k_floor_plane.setHeight(4.0);
                    k_floor_plane.setOrientation(ofQuaternion(1, 0, 0, 1)*receiver->floorQuat);
                    k_floor_plane.setPosition(receiver->getNode().getGlobalPosition());
                    ofPushStyle();
                    ofSetColor(200);
                    k_floor_plane.drawWireframe();
                    ofPopStyle();
                }
            }
        }
        
        
        cam.end();
        
        drawGui();
    }
    
    
    //--------------------------------------------------------------
    void drawGui() {
        
        gui.begin();
        ImGui::SetNextWindowSize(ImVec2(400, ofGetHeight()));
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::Begin("MAIN");
        
        // show stats
        if(ImGui::CollapsingHeader("Global Stats", NULL, true, true)) {
            stringstream str;
            str << "Total persons: " << persons_global_all.size() << endl;
            str << "fps: " << ofGetFrameRate();
            ImGui::Text(str.str().c_str());
        }
        
        
        if(ImGui::CollapsingHeader("Global Params", NULL, true, true)) {
            ImGui::Checkbox("do_avg_person", &params.do_avg_person);
            ImGui::DragFloat("pos smoothing", &pr::Receiver::pos_smoothing, 0.01, 0, 1);
            ImGui::DragFloat("vel smoothing", &pr::Receiver::vel_smoothing, 0.01, 0, 1);
            ImGui::DragFloat("spring strength", &pr::Receiver::spring_strength, 0.01, 0, 1);
            ImGui::DragFloat("spring damping", &pr::Receiver::spring_damping, 0.01, 0, 1);
            ImGui::DragInt("kill frame count", &pr::Receiver::kill_frame_count, 1);
        }
        
        for(auto&& receiver : receivers) {
            if(receiver) receiver->drawGui();
            else ofLogError() << "App::drawGui receiver == NULL";
        }
        
        osc_sender.drawGui();
        
        // display params
        if(ImGui::CollapsingHeader("Display params", NULL, true, true)) {
            ImGui::Checkbox("show floor", &params.display.show_floor);
            ImGui::Checkbox("draw Kinect floors", &params.display.show_kinect_floors);
            //        ImGui::SliderInt("floor size", &params.display.floor_size, 5, 20);
            ImGui::Checkbox("show all persons", &params.display.show_received_persons);
            ImGui::Checkbox("show final persons", &params.display.show_final_persons);
            ImGui::Checkbox("show avg person", &params.display.show_avg_person);
            
            ImGui::DragFloat("jointRadius", &params.display.joint_radius, 0.001, 0, 2);
            ImGui::Checkbox("show target pos", &params.display.show_target_pos);
            ImGui::Checkbox("show springy pos", &params.display.show_springy_pos);
            ImGui::Checkbox("show vel vector", &params.display.show_vel);
            ImGui::DragFloat("vel mult", &params.display.vel_mult, 0.01, 0, 2);
        }
        
        ImGui::End();
        
        ImGui::ShowTestWindow(NULL);
        
        ml.draw_gui();
        
        gui.end();
    }
    
    
    //--------------------------------------------------------------
    void keyPressed(int key) {
        switch(key) {
            case 'l' :loadFromXml(kXmlFilename); break;
            case 's' :saveToXml(kXmlFilename); break;
            case 'v':
                cam.setPosition(0, 2.5, 10);
                cam.lookAt(params.display.floor_pos, ofVec3f(0, 1, 0));
                cam.setDistance(10);
                cam.setTarget(params.display.floor_pos);
                break;
                
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
