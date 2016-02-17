/*
 
 Check scaling, normalization
 
 
 
 */
#pragma once

#include "floatfann.h"
#include "fann_cpp.h"

#include "ofxImGui.h"
#include "Person.h"

#include "VectorUtils.h"

namespace pr {
    namespace ml {
        
        typedef vector<fann_type> DataVector;
        
        class Model {
        public:
            bool do_scaling         = true;
            int num_layers          = 3;
            int input_dim           = 9;
            int output_dim          = 9;
            int hidden_dim          = 9;
            //    float learning_rate     = 0.1;    // not used in rdprop
            //    float momentum          = 0.9;
            float desired_error     = 0.001;
            int max_epochs          = 10000;
            int epochs_per_report   = 100;
            
            
            void init() {
                trained = false;
                clear_training_data();
            }
            
            int training_data_size() const {
                if(input_dim <= 0 || output_dim <= 0) {
                    ofLogError() << "ml::Model::size - invalid dimensions";
                    return 0;
                }
                
                if(inputs.size() / input_dim != targets.size() / output_dim) {
                    ofLogError() << "ml::Model::size - input-target dimension mismatch";
                    return 0;
                }
                
                return inputs.size() / input_dim;
            }
            
            
            void clear_training_data() {
                ofLogNotice() << "ml::Model::clear_training_data";
                inputs.clear();
                targets.clear();
                input_min_values.clear();
                input_max_values.clear();
                target_min_values.clear();
                target_max_values.clear();
            }
            
            void save_training_data(string filename) {
                ofLogNotice() << "ml::Model::save_training_data " << filename;
                training_data.save_train(filename);
            }
            
            void load_training_data(string filename) {
                ofLogNotice() << "ml::Model::load_training_data " << filename;
                training_data.read_train_from_file(filename);
            }
            
            void save_network(string filename) {
                ofLogNotice() << "ml::Model::save_network " << filename;
                net.save(filename);
            }
            
            void load_network(string filename) {
                ofLogNotice() << "ml::Model::load_network " << filename;
                net.create_from_file(filename);
            }
            
            
            void add_training_data(const DataVector& input_vec, const DataVector& target_vec) {
                if(input_vec.size() != input_dim) {
                    ofLogError() << "ML::add_data - input dim doesn't match" << input_vec.size() << " != " << input_dim;
                    return;
                }
                if(target_vec.size() != output_dim) {
                    ofLogError() << "ML::add_data - output dim doesn't match" << target_vec.size() << " != " << output_dim;
                    return;
                }
                // insert to end of current vectors
                inputs.insert(inputs.end(), input_vec.begin(), input_vec.end());
                targets.insert(targets.end(), target_vec.begin(), target_vec.end());
            }
            
            void train() {
                ofLogNotice() << "ml::Model::train " << "num_layers: " << num_layers << ", input_dim: " << input_dim << ", hidden_dim: " << hidden_dim << ", output_dim: " << output_dim;
                // init network
                
                net.create_standard(num_layers, input_dim, hidden_dim, output_dim);
                //        net.set_learning_rate(learning_rate);
                //        net.set_learning_momentum(momentum);
                //        net.set_activation_steepness_hidden(1.0);
                //        net.set_activation_steepness_output(1.0);
                
                net.set_activation_function_hidden(FANN::SIGMOID_SYMMETRIC);
                net.set_activation_function_output(FANN::SIGMOID_SYMMETRIC);
                net.set_training_algorithm(FANN::TRAIN_RPROP);
//                net.print_parameters();
                
                if(training_data_size() == 0) {
                    ofLogError() << " - no data, or input num samples doesn't match target num data";
                    return;
                }
                
                // scale training data
                if(do_scaling) {
                    msa::vector_utils::get_range(inputs, input_dim, input_min_values, input_max_values);
                    msa::vector_utils::normalize(inputs, input_min_values, input_max_values, -1.0f, 1.0f, inputs);

                    msa::vector_utils::get_range(targets, output_dim, target_min_values, target_max_values);
                    msa::vector_utils::normalize(targets, target_min_values, target_max_values, -1.0f, 1.0f, targets);
                    
                    //                    net.set_scaling_params(training_data, -1, 1, -1, 1);
                    //                    net.scale_train(training_data);
                };
                
                // copy data across
                training_data.set_train_data(training_data_size(), input_dim, inputs.data(), output_dim, targets.data());
                save_training_data(ofToDataPath("training_data.txt"));
                
                // Initialize and train the network with the data
                net.init_weights(training_data);
                
                ofLogNotice() << "Max Epochs " << setw(8) << max_epochs << ". " << "Desired Error: " << left << desired_error << right << endl;
                net.set_callback(print_callback, NULL);
                net.train_on_data(training_data, max_epochs, epochs_per_report, desired_error);
                
                trained = true;
                
                cout << endl << "Testing network." << endl;
            }
            
            void predict(const DataVector& input_vec, DataVector& output_vec) {
                if(!trained) return;
                
                if(output_vec.size() != output_dim) output_vec.resize(output_dim);
                
                fann_type* ret = net.run(const_cast<fann_type*>(input_vec.data()));
                memcpy(output_vec.data(), ret, output_dim);
//                if(do_scaling) net.descale_output(output_vec.data());
                if(do_scaling) msa::vector_utils::unnormalize(output_vec, target_min_values, target_max_values, -1.0f, 1.0f, output_vec);
            }
            
            
        protected:
            bool trained = false;
            
            FANN::neural_net net;
            FANN::training_data training_data;
            
            // these are treated like tables, but stored as 1d vectors
            DataVector inputs, targets;
            
            // range for normalization and unnormalization
            DataVector input_min_values, input_max_values, target_min_values, target_max_values;
            
            // Callback function that simply prints the information to cout
            static int print_callback(FANN::neural_net &net, FANN::training_data &train,
                                      unsigned int max_epochs, unsigned int epochs_between_reports,
                                      float desired_error, unsigned int epochs, void *user_data)
            {
                cout << "Epochs     " << setw(8) << epochs << ". " << "Current Error: " << left << net.get_MSE() << right << endl;
                return 0;
            }
        };
        
        
        
        class Trainer {
        public:
            bool enabled = false;
            bool do_record = false;
            bool do_predict = false;
            int input_person_id = 2;            // input person, will be present during training and prediction
            int target_person_id = 1;           // target person, will be 'imagined'
            int output_person_id = 0;           // slot to write to
            bool input_do_local = true;         // whether to do pos relative to waist or not
            bool target_do_local = true;
            vector<string> joints_to_include;   // vector of included joints
            
            // TODO: add noise while recording
            
            
            void init() {
                ofLogNotice() << "ml::Trainer::init ";
                model.input_dim = model.output_dim = model.hidden_dim = joints_to_include.size() * 3;
                model.clear_training_data();
                model.init();
                
                input_vec.clear();
                target_vec.clear();
                output_vec.clear();
                
                do_record = do_predict = false;
            }
            
            
            void update(vector<Person::Ptr>& persons) {
                if(!enabled) return;
                
                Person::Ptr input_person = persons[input_person_id];
                if(input_person) input_vec = person_to_representation(input_person, joints_to_include, input_do_local);
                
                Person::Ptr target_person = persons[target_person_id];
                if(target_person) target_vec = person_to_representation(target_person, joints_to_include, target_do_local);
                
                if(input_person) {
                    if(do_record) {
                        if(target_person) {
                            model.add_training_data(input_vec, target_vec);
                        } else {
                            ofLogError() << "ml::Trainer::update record - target person null";
                        }
                    } else if(do_predict) {
                        // get prediction
                        model.predict(input_vec, output_vec);
                        
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
                
                if(!enabled) return;
                
                do_record = do_predict = false;
                
                model.train();
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
                
                if(ImGui::Button("init", button_size)) init(); ImGui::SameLine();
                if(ImGui::Button("train", button_size)) train();
                
                stringstream str;
                str << "input_dim: " << model.input_dim << endl;
                str << "hidden_dim: " << model.hidden_dim << endl;
                str << "output_dim: " << model.output_dim << endl;
                str << "training samples: " << model.training_data_size() << endl;
                ImGui::Text(str.str().c_str());
                
                if(ImGui::CollapsingHeader("Viz", NULL, true, true)) {
                    //                    void ImGui::PlotHistogram(const char* label, const float* values, int values_count, int values_offset, const char* overlay_text, float scale_min, float scale_max, ImVec2 graph_size, int stride)
                    static float begin = -1, end = 1;
                    ImGui::DragFloatRange2("range", &begin, &end, 0.01f);//, 0.0f, 100.0f, "Min: %.1f", "Max: %.1f");
                    if(!input_vec.empty()) ImGui::PlotHistogram("input_vec", input_vec.data(), input_vec.size(), 0, NULL, begin, end, ImVec2(0,80));
                    if(!target_vec.empty()) ImGui::PlotHistogram("target_vec", target_vec.data(), target_vec.size(), 0, NULL, begin, end, ImVec2(0,80));
                    if(!output_vec.empty()) ImGui::PlotHistogram("output_vec", output_vec.data(), output_vec.size(), 0, NULL, begin, end, ImVec2(0,80));
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
            Model model;
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