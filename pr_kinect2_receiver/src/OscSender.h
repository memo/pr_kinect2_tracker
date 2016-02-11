#pragma once

#include "ofxOscSender.h"
#include "ofxImGui.h"


namespace pr {

class OscSender {
public:
    bool enabled = true;
    int host_port = 0;
    string host_ip = "127.0.0.1";
    ofxOscSender osc_sender;


    void setup(string s, int p) {
        if(host_port != p || host_ip != s) {
            host_port = p;
            host_ip = s;
            init();
        }
    }

    void init() {
        osc_sender.setup(host_ip, host_port);
    }

    void sendBundle(ofxOscBundle& b) {
        if(enabled && host_port) osc_sender.sendBundle(b);
    }

    void drawGui() {
        ImGui::CollapsingHeader("OscSender", NULL, true, true);
        ImGui::Checkbox("Enabled", &enabled);
        //ImGui::InputText("Host ip", hostIp.c_str(), )
        ImGui::InputInt("Port", &host_port, 1, 100);
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
