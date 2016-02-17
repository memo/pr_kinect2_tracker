/*
 
 Check scaling, normalization
 
 
 
 */
#pragma once

//#include "floatfann.h"
//#include "fann_cpp.h"
//
#include "ofxImGui.h"
#include "Person.h"

#include "VectorUtils.h"

//#include "MLImplFann.h"
#include "MLImplGrt.h"
#include "Model.h"

namespace pr {
    namespace ml {
        
        //        typedef vector<fann_type> DataVector;
        
        typedef GRT::Float DataType;
        typedef GRT::VectorFloat DataVector;
//        typedef float DataType;
//        typedef vector<DataType> DataVector;
        typedef msa::ml::Model<DataVector, DataType> Model;
        typedef msa::ml::MLImplGrt<DataVector, DataType> MLImpl;
        
        class Trainer {
        public:
            // TODO: add noise while recording
            
            void init() {
                ofLogNotice() << "ml::Trainer::init ";
                int dim = joints_to_include.size() * 3;
                model = make_unique<Model>("model", dim, dim, make_shared<MLImpl>());
                model->get_params().hidden_dim = hidden_dim;
                
                input_vec.clear();
                target_vec.clear();
                output_vec.clear();
                
                do_record = do_predict = false;
            }
            
            
            void update(vector<Person::Ptr>& persons) {
                if(!enabled || !model) return;
                
                Person::Ptr input_person = persons[input_person_id];
                if(input_person) input_vec = person_to_representation(input_person, joints_to_include, input_do_local);
                
                Person::Ptr target_person = persons[target_person_id];
                if(target_person) target_vec = person_to_representation(target_person, joints_to_include, target_do_local);
                
                if(input_person) {
                    if(do_record) {
                        if(target_person) {
                            model->set_output_vector(target_vec);
                            model->add_training_data(input_vec);
                        } else {
                            ofLogError() << "ml::Trainer::update record - target person null";
                        }
                    } else if(do_predict) {
                        // get prediction
                        model->predict(input_vec, output_vec);
                        
                        Person::Ptr& output_person = persons[output_person_id];
                        
                        // make sure we have a unique person to write to
                        if(!output_person || output_person == input_person) {
                            ofLogWarning() << "ml::Trainer::update predict - output null or same as input, reallocating";
                            output_person = persons[output_person_id] = make_shared<Person>("ml_output");
                        }
                        
                        representation_to_person(output_person, joints_to_include, target_do_local, output_vec);
                    }
                } else {
                    ofLogError() << "ml::Trainer::update - input person null";
                }
            }
            
            void train() {
                ofLogNotice() << "ml::Trainer::train ";
                
                if(!enabled || !model) return;
                
                do_record = do_predict = false;
                
                model->train();
            }
            
            
            void draw_gui() {
                static ImVec2 button_size(160, 20);
                ImGui::SetNextWindowSize(ImVec2(400, ofGetHeight()));
                ImGui::SetNextWindowPos(ImVec2(ofGetWidth() - 400, 0));
                
                
                // start new window
                ImGui::Begin("ML");
                ImGui::Columns(3, "mycolumns");
                //                ImGui::Separator();
                
                if(ImGui::Checkbox("Enabled", &enabled)) do_predict = do_record = false;
                ImGui::InputInt("input_person_id", &input_person_id);
                ImGui::Checkbox("input_do_local", &input_do_local);
                ImGui::NextColumn();
                
                if(ImGui::Checkbox("Record", &do_record)) do_predict = false;
                ImGui::InputInt("target_person_id", &target_person_id);
                ImGui::Checkbox("target_do_local", &target_do_local);
                ImGui::NextColumn();
                
                if(ImGui::Checkbox("Predict", &do_predict)) do_record = false;
                ImGui::InputInt("output_person_id", &output_person_id);
                ImGui::Columns(1);
                
                ImGui::InputInt("hidden_dim", &hidden_dim, 1, 10);
                if(ImGui::Button("init", button_size)) init(); ImGui::SameLine();
                if(ImGui::Button("train", button_size)) train();
                
                stringstream str;
                if(model) {
                    str << "input_dim: " << model->get_input_dim() << endl;
                    str << "hidden_dim: " << model->get_params().hidden_dim << endl;
                    str << "output_dim: " << model->get_output_dim() << endl;
                    str << "training samples: " << model->get_training_data().size() << endl;
                    ImGui::Text(str.str().c_str());
                }
                
                if(ImGui::CollapsingHeader("Viz", NULL, true, true)) {
                    //                    void ImGui::PlotHistogram(const char* label, const float* values, int values_count, int values_offset, const char* overlay_text, float scale_min, float scale_max, ImVec2 graph_size, int stride)
                    static float begin = -1, end = 1;
                    ImGui::DragFloatRange2("range", &begin, &end, 0.01f);//, 0.0f, 100.0f, "Min: %.1f", "Max: %.1f");
                    static vector<float> temp; // this is stupid that I can't get GRT to work with floats!
                    if(!input_vec.empty()) {
                        msa::vector_utils::convert(input_vec, temp);
                        ImGui::PlotHistogram("input_vec", temp.data(), temp.size(), 0, NULL, begin, end, ImVec2(0,80));
                    }
                    
                    if(!target_vec.empty()) {
                        msa::vector_utils::convert(target_vec, temp);
                        ImGui::PlotHistogram("target_vec", temp.data(), temp.size(), 0, NULL, begin, end, ImVec2(0,80));
                    }
                    
                    if(!output_vec.empty()) {
                        msa::vector_utils::convert(output_vec, temp);
                        ImGui::PlotHistogram("output_vec", temp.data(), temp.size(), 0, NULL, begin, end, ImVec2(0,80));
                    }
                }
                
                if(ImGui::CollapsingHeader("Representation", NULL, true, true)) {
                    static map<string, bool> joints_bools_map;  // map of bools for joints (for gui)
                    bool rep_changed = false;
                    
                    if(ImGui::Button("none", button_size)) {
                        rep_changed = true;
                        for(auto&& joint_name : Person::joint_names) joints_bools_map[joint_name] = false;
                    }
                    ImGui::SameLine();
                    if(ImGui::Button("all", button_size)) {
                        rep_changed = true;
                        for(auto&& joint_name : Person::joint_names) joints_bools_map[joint_name] = true;
                    }
                    
                    
                    if(ImGui::Button("hands", button_size)) {
                        rep_changed = true;
                        for(auto&& joint_name : Person::joint_names) joints_bools_map[joint_name] = false;
                        joints_bools_map["l_hand"] = joints_bools_map["r_hand"] = true;
                    }
                    ImGui::SameLine();
                    if(ImGui::Button("hands and feet", button_size)) {
                        rep_changed = true;
                        for(auto&& joint_name : Person::joint_names) joints_bools_map[joint_name] = false;
                        joints_bools_map["l_hand"] = joints_bools_map["r_hand"] = joints_bools_map["l_foot"] = joints_bools_map["r_foot"] = true;
                    }
                    
                    
                    
                    // iterate joint parents (to get names of all joints)
                    int i=0;
                    for(auto&& joint_name : Person::joint_names) {
                        if(ImGui::Checkbox(joint_name.c_str(), &joints_bools_map[joint_name])) rep_changed = true;
                        if(i++ % 2 == 0) ImGui::SameLine(button_size.x);
                    }
                    
                    
                    if(rep_changed) {
                        joints_to_include = update_representation_vec(joints_bools_map);
                        init();
                    }
                    
                    // for testing
                    //                string joint_names = "";
                    //                for(auto&& joint_name : joints_to_include) joint_names += joint_name + " ";
                    //                ImGui::Text(joint_names.c_str());
                }
                ImGui::End();
            }
            
        protected:
            bool enabled = false;
            bool do_record = false;
            bool do_predict = false;
            int input_person_id = 2;            // input person, will be present during training and prediction
            int target_person_id = 1;           // target person, will be 'imagined'
            int output_person_id = 0;           // slot to write to
            int hidden_dim = 3;
            bool input_do_local = true;         // whether to do pos relative to waist or not
            bool target_do_local = true;
            vector<string> joints_to_include;   // vector of included joints
            
            
            unique_ptr<Model> model;
            DataVector input_vec, target_vec, output_vec;  // cached vectors
            
            static vector<string> update_representation_vec(const map<string, bool>& joints_bools_map) {
                ofLogNotice() << "ml::Trainer::update_representation_vec";
                vector<string> joints_to_include;
                // iterate joints to include
                for(const auto& kv : joints_bools_map) {
                    // add to vector if true (included)
                    if(kv.second) {
                        joints_to_include.push_back(kv.first);
                        ofLogNotice() << "  " << kv.first;
                    }
                }
                return joints_to_include;
            }
            
            
            
            static DataVector person_to_representation(const Person::Ptr person, const vector<string>& joints_to_include, bool do_local) {
                DataVector data;
                // iterate joints to include
                for(const auto& joint_name : joints_to_include) {
                    ofVec3f p(person->joints[joint_name].pos.current);
                    if(do_local) p -= person->joints["waist"].pos.current;
                    data.push_back(p.x);
                    data.push_back(p.y);
                    data.push_back(p.z);
                }
                return data;
            }
            
            static void representation_to_person(Person::Ptr person, const vector<string>& joints_to_include, bool do_local, const DataVector& data) {
                // iterate joints to include
                int i=0;
                for(const auto& joint_name : joints_to_include) {
                    ofVec3f p;
                    p.x = data[i++];
                    p.y = data[i++];
                    p.z = data[i++];
                    if(do_local) p += person->joints["waist"].pos.current;
                    
                    person->joints[joint_name].pos.target = p;
                    person->joints[joint_name].pos.current = person->joints[joint_name].pos.target;
                    
                    // TODO: speed / vel / vector / springy stuff?
                }
            }
            
        };
        
    }
}