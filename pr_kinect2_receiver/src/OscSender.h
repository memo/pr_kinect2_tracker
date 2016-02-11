#pragma once

#include "ofxOscSender.h"
#include "ofxImGui.h"


namespace pr {

class OscSender {
public:
    bool enabled = true;
    int hostPort = 0;
    string hostIp = "127.0.0.1";
    ofxOscSender oscSender;


    void setup(string s, int p) {
        if(hostPort != p || hostIp != s) {
            hostPort = p;
            hostIp = s;
            init();
        }
    }

    void init() {
        oscSender.setup(hostIp, hostPort);
    }

    void sendBundle(ofxOscBundle& b) {
        if(enabled && hostPort) oscSender.sendBundle(b);
    }

    void drawGui() {
        ImGui::CollapsingHeader("OscSender", NULL, true, true);
        ImGui::Checkbox("Enabled", &enabled);
        //ImGui::InputText("Host ip", hostIp.c_str(), )
        ImGui::InputInt("Port", &hostPort, 1, 100);
    }


    void loadFromXml(ofXml& xml) {
        // load ip and port from xml
        // init osc if changed
    }

    void saveToXml(ofXml& xml) {
        // save ip and port to xml

    }




};

}
