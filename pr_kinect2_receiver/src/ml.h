/*
 
 Check scaling, normalization
 
 
 
 */
#pragma once

#include "floatfann.h"
#include "fann_cpp.h"

#include "ofxImGui.h"
#include "Person.h"

namespace pr {
    namespace ml {
        
        typedef vector<fann_type> DataVector;
        
        class Model {
        public:
            FANN::neural_net net;
            
            bool do_scaling         = false;
            int num_layers          = 3;
            int input_dim           = 9;
            int output_dim          = 9;
            int hidden_dim          = 9;
            //    float learning_rate     = 0.1;    // not used in rdprop
            //    float momentum          = 0.9;
            float desired_error     = 0.001;
            int max_epochs          = 100000;
            int epochs_per_report   = 1000;
            
            void clear_training_data() {
                ofLogNotice() << "ml::Model::clear_training_data";
                inputs.clear();
                targets.clear();
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
                
                
                // Initialize and train the network with the data
                net.init_weights(training_data);
                
                update_training_data();
                
                ofLogNotice() << "Max Epochs " << setw(8) << max_epochs << ". " << "Desired Error: " << left << desired_error << right << endl;
                net.set_callback(print_callback, NULL);
                net.train_on_data(training_data, max_epochs, epochs_per_report, desired_error);
                
                cout << endl << "Testing network." << endl;
            }
            
            void predict(const DataVector& input_vec, DataVector& output_vec) {
                if(output_vec.size() != output_dim) output_vec.resize(output_dim);
                
                fann_type* ret = net.run(const_cast<fann_type*>(input_vec.data()));
                memcpy(output_vec.data(), ret, output_dim);
                if(do_scaling) net.descale_output(output_vec.data());
            }
            
            
        protected:
            // training data
            FANN::training_data training_data;
            
            // these are treated like tables, rather than 1d vectors
            vector < fann_type > inputs;
            vector < fann_type > targets;
            
            void update_training_data() {
                ofLogNotice() << "ml::Model::update_training_data ";
                
                if(inputs.size() == 0) {
                    ofLogError() << " - no data";
                    return;
                }
                
                if(inputs.size() != targets.size()) {
                    ofLogError() << " - input num data doesn't match target num data " << inputs.size() << " != " << targets.size();
                    return;
                }
                
                // copy data across
                training_data.set_train_data(inputs.size(), input_dim, inputs.data(), output_dim, targets.data());
                
                // scale training data
                if(do_scaling) {
                    net.set_scaling_params(training_data, -1, 1, -1, 1);
                    net.scale_train(training_data);
                };
            }
            
            
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
            int input_person_id = 2;            // input person
            bool input_do_local = true;         // whether to do pos relative to waist or not
            int target_person_id = 1;           // the 'imaginary' person (i.e. the trainer)
            bool target_do_local = true;
            vector<string> joints_to_include;   // vector of included joints
            
            // TODO: add noise while recording
            
            
            void init() {
                ofLogNotice() << "ml::Trainer::init ";
                model.input_dim = model.output_dim = model.hidden_dim = joints_to_include.size() * 3;
                model.clear_training_data();
            }
            
            
            void update(vector<Person::Ptr>& persons) {
                if(!enabled) return;
                
                Person::Ptr& input_person = persons[input_person_id];
                Person::Ptr& target_person = persons[target_person_id];
                
                if(input_person) {
                    if(do_record) {
                        do_predict = false;
                        if(target_person) {
                            model.add_training_data(
                                                    person_to_representation(input_person, joints_to_include, input_do_local),
                                                    person_to_representation(target_person, joints_to_include, target_do_local)
                                                    );
                        } else {
                            ofLogError() << "ml::Trainer::update record - target person null";
                        }
                    } else if(do_predict) {
                        // get prediction
                        model.predict(person_to_representation(input_person, joints_to_include, input_do_local), prediction);
                        
                        // make sure we have a unique person to write to
                        if(!target_person || target_person == input_person) {
                            ofLogWarning() << "ml::Trainer::update predict - target null or same as input, reallocating";
                            target_person = make_shared<pr::Person>();
                        }
                        
                        representation_to_person(target_person, joints_to_include, target_do_local, prediction);
                        // what to do if input and target is same person pointer :/
                    }
                } else {
                    ofLogError() << "ml::Trainer::update - input person null";
                }
            }
            
            
            void drawGui() {
                // start new window
                ImGui::Begin("ML");
                ImGui::Checkbox("Enabled", &enabled);
                ImGui::Checkbox("Record", &do_record);
                ImGui::Checkbox("Predict", &do_predict);
                
                ImGui::InputInt("input_person_id", &input_person_id);
                ImGui::Checkbox("input_do_local", &input_do_local);
                ImGui::InputInt("target_person_id", &target_person_id);
                ImGui::Checkbox("target_do_local", &target_do_local);
                
                //                ImGui::SliderInt("input_dim", &model.input_dim, 1, 50);
                //                ImGui::SliderInt("hidden_dim", &model.hidden_dim, 1, 50);
                //                ImGui::SliderInt("output_dim", &model.output_dim, 1, 50);
                
                if(ImGui::Button("Init")) init();
                
                stringstream str;
                str << "input_dim: " << model.input_dim << endl;
                str << "hidden_dim: " << model.hidden_dim << endl;
                str << "model.output_dim: " << model.output_dim << endl;
                ImGui::Text(str.str().c_str());
                
                ImGui::CollapsingHeader("Joints");
                bool all_joints = ImGui::Button("All joints");
                bool no_joints = ImGui::Button("No joints");
                
                bool rep_changed = false;
                // iterate joint parents (to get names of all joints)
                static map<string, bool> joints_bools_map;  // map of bools for joints (for gui)
                for(auto&& joint_name : Person::joint_names) {
                    if(all_joints) { joints_bools_map[joint_name] = true; rep_changed = true; }
                    if(no_joints) { joints_bools_map[joint_name] = false; rep_changed = true; }
                    if(ImGui::Checkbox(joint_name.c_str(), &joints_bools_map[joint_name])) rep_changed = true;
                }
                
                if(rep_changed) {
                    joints_to_include = update_representation_vec(joints_bools_map);
                    init();
                }
                
                // for testing
                //                string joint_names = "";
                //                for(auto&& joint_name : joints_to_include) joint_names += joint_name + " ";
                //                ImGui::Text(joint_names.c_str());
                
                ImGui::End();
            }
            
        protected:
            Model model;
            DataVector prediction;  // cached vector used for prediction
            
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