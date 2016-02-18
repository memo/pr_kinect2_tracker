/*
 
 Check scaling, normalization
 
 
 
 */
#pragma once

#include "ofxImGui.h"
#include "Person.h"

#include "VectorUtils.h"

#include "TrainingData.h"
#include "MLImplFann.h"
#include "MLImplGrt.h"

namespace pr {
    namespace ml {
        
                typedef GRT::Float DataType;
                typedef GRT::VectorFloat DataVector;
                typedef msa::ml::MLImplGrt<DataVector, DataType> MLImpl;
        
//        typedef fann_type DataType;
//        typedef vector<DataType> DataVector;
//        typedef msa::ml::MLImplFann<DataVector, DataType> MLImpl;
        
        class Manager {
        public:
            // TODO: add noise while recording
            
            Manager() {
                init();
            }
            
            void init() {
                ofLogNotice() << "ml::Main:init ";
                int dim = joints_to_include.size() * 3; // xyz
                
                model_params.input_dim = model_params.output_dim = model_params.hidden_dims[0] = dim;
                
                training_data.set_dimensions(dim, dim);
                
                input_vec.clear();
                target_vec.clear();
                output_vec.clear();
                
                do_record = do_predict = false;
                
                trained = false;
            }
            
            
            void update(vector<Person::Ptr>& persons) {
                if(!enabled || model_params.input_dim == 0 || model_params.output_dim == 0) return;
                
                if(num_hidden_layers != model_params.hidden_dims.size()) {
                    ofLogWarning() << "ml::Main:update - setting hidden_dims size to " << num_hidden_layers;
                    model_params.hidden_dims.resize(num_hidden_layers);
                }
                
                Person::Ptr input_person = persons[input_person_id];
                if(input_person) {
                    input_vec = person_to_representation(input_person, joints_to_include, input_do_local);
                    if(add_input_noise > 0) for(auto&& d : input_vec) d += ofRandomf() * add_input_noise;
                }
                
                Person::Ptr target_person = persons[target_person_id];
                if(target_person) target_vec = person_to_representation(target_person, joints_to_include, target_do_local);
                
                if(input_person) {
                    if(do_record) {
                        if(target_person) {
                            training_data.add_sample(input_vec, target_vec);
                        } else {
                            ofLogError() << "ml::Main:update record - target person null";
                        }
                    } else if(do_predict && predict(input_vec, output_vec)) {   // get prediction
                        
                        // check enough persons in array
                        if(output_person_id >= persons.size()) {
                            ofLogWarning() << "ml::Main:update predict - not enough persons for output person";
                            persons.resize(output_person_id+1);
                        }
                        
                        Person::Ptr& output_person = persons[output_person_id];
                        
                        // make sure we have a unique person to write to
                        if(!output_person || output_person == input_person) {
                            ofLogWarning() << "ml::Main:update predict - output person null or same as input, reallocating";
                            output_person = persons[output_person_id] = make_shared<Person>("ml_output");
                        }
                        
                        // make purple for easy seeing
                        output_person->color.set(255, 0, 255);
                        
                        representation_to_person(output_person, joints_to_include, target_do_local, output_vec);
                    }
                } else {
                    ofLogError() << "ml::Main:update - input person null";
                }
            }
            
            
            bool predict(const DataVector& input_vector, DataVector& output_vector) {
                if(!enabled || !trained) return false;
                
                if (input_vector.size() != model_params.input_dim) {
                    ofLogError() << "ml::Main::predict - data dimensions does not match input dimensions: " << model_params.input_dim;
                    return false;
                }
                
                // if trained, predict output
                output_vector.resize(model_params.output_dim);
                
                DataVector input_vector_norm(input_vector.size());
                DataVector output_vector_norm(output_vector.size());
                
                // normalize input
                msa::vector_utils::normalize(input_vector, training_data.get_input_min_values(), training_data.get_input_max_values(), training_data.normalize_min, training_data.normalize_max, input_vector_norm);
                
                // run network
                ml_impl.predict(input_vector_norm, output_vector_norm);
                
                // unnormalize output
                msa::vector_utils::unnormalize(output_vector_norm, training_data.get_output_min_values(), training_data.get_output_max_values(), training_data.normalize_min, training_data.normalize_max, output_vector);
                
                return true;
            }
            
            
            
            void train() {
                if(!enabled) return;
                
                ofLogNotice() << "ml::Main:train ";
                
                do_record = do_predict = false;
                
                if (training_data.size() > 0) {
                    training_data.calc_range();
                    trained = ml_impl.train(training_data, model_params, train_params);
                    if(trained) ofLogNotice() << "ml::Main:train - success";
                    else ofLogNotice() << "ml::Main:train - error";
                } else {
                    ofLogWarning() << "ml::Main:train - no data";
                }
            }
            
            
            
            void draw_gui() {
                static ImVec2 button_size(160, 20);
                ImGui::SetNextWindowSize(ImVec2(400, ofGetHeight()));
                ImGui::SetNextWindowPos(ImVec2(ofGetWidth() - 400, 0));
                
                
                // start new window
                ImGui::Begin("ML");
                ImGui::Columns(3);
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
                
                ImGui::SliderFloat("add_input_noise", &add_input_noise, 0, 1);
                
                
                if(ImGui::CollapsingHeader("Model Parameters", NULL, true, true)) {
                    stringstream str;
                    str << "input_dim: " << model_params.input_dim << endl;
                    str << "output_dim: " << model_params.output_dim << endl;
                    ImGui::Text(str.str().c_str());
                    
                    ImGui::InputInt("num_hidden_layers", &num_hidden_layers, 1, 10);
                    for(int i=0; i<model_params.hidden_dims.size(); i++)
                        ImGui::InputInt(("hidden_dim_" + ofToString(i)).c_str(), &model_params.hidden_dims[i], 1, 10);
                }
                if(ImGui::CollapsingHeader("Training Parameters", NULL, true, true)) {
                    ImGui::InputFloat("learning_rate", &train_params.learning_rate, 0.01);
                    ImGui::InputFloat("learning_momentum", &train_params.learning_momentum, 0.01);
                    ImGui::InputFloat("min_delta", &train_params.min_delta, 0.000001);
                    ImGui::InputInt("max_epochs", &train_params.max_epochs, 100);
                    ImGui::InputInt("epochs_between_reports", &train_params.epochs_between_reports, 10);
                    
                    ImGui::InputInt("num_train_sessions", &train_params.num_train_sessions, 1);
                    ImGui::Checkbox("use_validation", &train_params.use_validation);
                    ImGui::SliderInt("validation_size", &train_params.validation_size, 0, 100);
                    ImGui::Checkbox("randomize_train_order", &train_params.randomize_train_order);
                    ImGui::Checkbox("use_normalization", &train_params.use_normalization);
                    // TODO: add activation function, algorithm, other params?
                }
                
                if(ImGui::CollapsingHeader("Training Data", NULL, true, true)) {
                    stringstream str;
                    str << "input_dim: " << training_data.get_input_dim() << endl;
                    str << "output_dim: " << training_data.get_output_dim() << endl;
                    str << "training samples: " << training_data.size() << endl;
                    ImGui::Text(str.str().c_str());
                }
                
                if(ImGui::Button("init", button_size)) init(); ImGui::SameLine();
                if(ImGui::Button("train", button_size)) train();
                
                if(ImGui::Button("save", button_size)) {
                    ofFileDialogResult fr = ofSystemSaveDialog(str_joints_to_include + ".txt", "Save training data");
                    if(fr.bSuccess) training_data.save(fr.getPath());
                }
                ImGui::SameLine();
                if(ImGui::Button("load", button_size)) {
                    ofFileDialogResult fr = ofSystemLoadDialog();
                    if(fr.bSuccess) training_data.load(fr.getPath());    // TODO: how to match joint into to data :/
                }
                
                
                if(ImGui::CollapsingHeader("Viz", NULL, true, true)) {
                    //                    void ImGui::PlotHistogram(const char* label, const float* values, int values_count, int values_offset, const char* overlay_text, float scale_min, float scale_max, ImVec2 graph_size, int stride)
                    static float begin = -1, end = 1;
                    ImGui::DragFloatRange2("range", &begin, &end, 0.01f);//, 0.0f, 100.0f, "Min: %.1f", "Max: %.1f");
                    static vector<float> temp; // sucks that I can't get GRT to work with floats and ImGui can't display doubles, so I have to convert :/
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
                    
                    if(!target_vec.empty() && output_vec.size() == target_vec.size()) {
                        for(int i=0; i<target_vec.size(); i++) temp[i] = target_vec[i] - output_vec[i];
                        ImGui::PlotHistogram("diff_vec", temp.data(), temp.size(), 0, NULL, begin, end, ImVec2(0,80));
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
                    
                    ImGui::Text(str_joints_to_include.c_str());
                    
                    // iterate joint parents (to get names of all joints)
                    int i=0;
                    for(auto&& joint_name : Person::joint_names) {
                        if(ImGui::Checkbox(joint_name.c_str(), &joints_bools_map[joint_name])) rep_changed = true;
                        if(i++ % 2 == 0) ImGui::SameLine(button_size.x);
                    }
                    
                    
                    if(rep_changed) {
                        joints_to_include = update_representation_vec(joints_bools_map, str_joints_to_include);
                        init();
                    }
                }
                ImGui::End();
            }
            
        protected:
            // generic low level ml
            MLImpl ml_impl;
            bool trained = false;
            int num_hidden_layers = 1;
            msa::ml::mlp::ModelParameters model_params;
            msa::ml::mlp::TrainingParameters train_params;
            msa::ml::TrainingData<DataVector, DataType> training_data;
            
            // higher level generic ml
            bool enabled = false;
            bool do_record = false;
            bool do_predict = false;
            DataVector input_vec, target_vec, output_vec;  // cached vectors for gui etc.
            
            // app specific ml stuff
            int input_person_id = 2;            // input person, will be present during training and prediction
            int target_person_id = 1;           // target person, will be 'imagined'
            int output_person_id = 3;           // slot to write to
            bool input_do_local = true;         // whether to do pos relative to waist or not
            bool target_do_local = false;
            float add_input_noise = 0;                // add noise to input data
            vector<string> joints_to_include;   // vector of included joints
            string str_joints_to_include;       // useful for debugging and data filenames
            
            
            static vector<string> update_representation_vec(const map<string, bool>& joints_bools_map, string& str_joints_to_include) {
                ofLogNotice() << "ml::Main:update_representation_vec";
                vector<string> joints_to_include;
                // iterate joints to include
                for(const auto& kv : joints_bools_map) {
                    // add to vector if true (included)
                    if(kv.second) {
                        joints_to_include.push_back(kv.first);
                        ofLogNotice() << "  " << kv.first;
                    }
                }
                
                str_joints_to_include = "";
                for(auto&& joint_name : joints_to_include) str_joints_to_include += joint_name + " ";
                
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