#pragma once

#include "ofxOscSender.h"
#include "ofxImGui.h"


namespace pr {

class OscSender {
public:
    bool enabled = true;
    int host_port;
    string host_ip;
    ofxOscSender osc_sender;


    void setup(string s, int p) {
		host_port = p;
        host_ip = s;
        init();
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
		xml.setTo("//Settings/Sender");
		if (host_port != xml.getIntValue("port") || host_ip != xml.getValue<string>("ipAddress")) {
			host_port = xml.getIntValue("port");
			host_ip = xml.getValue<string>("ipAddress");
			init();
		}
    }

    void saveToXml(ofXml& xml) {
        // save ip and port to xml
		xml.setTo("//Settings");
		xml.addChild("Sender");
		xml.setTo("Sender");
		xml.addValue("port", ofToString(host_port));
		xml.addValue("ipAddress", host_ip);

    }




};

}
