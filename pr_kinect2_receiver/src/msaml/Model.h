//                                     __
//    ____ ___  ___  ____ ___  ____   / /__   __
//   / __ `__ \/ _ \/ __ `__ \/ __ \ / __/ | / /
//  / / / / / /  __/ / / / / / /_/ // /_ | |/ /
// /_/ /_/ /_/\___/_/ /_/ /_/\____(_)__/ |___/
//
//
// Created by Memo Akten, www.memo.tv
//


#pragma once

#include "MLImplBase.h"
#include "TrainingData.h"



namespace msa {
    namespace ml {


        // Vector type is templated (e.g. could be std::vector<float> or std::vector<double> or any other class that has same API
        // instances of this are instances of trainable features
        template <typename DataVector, typename T>
		class Model {
		public:
            typedef std::shared_ptr<Model> Ptr;

            bool verbose = false;

			// create model with name, dimensions and implementation
            Model(std::string model_name, int input_dim, int output_dim, typename MLImplBase<DataVector, T>::Ptr ml_impl);

			// set output vector for a model
            void set_output_vector(const DataVector& output_vector);

			// add data to the training set
            void add_training_data(const DataVector& input_vector);

			// train data on a model
			bool train();

			// predict data for a model
            void predict(const DataVector& input_vector, DataVector& output_vector);

			// save data
            bool save_training_data_to_file(std::string path);
            bool save_training_data_to_folder(std::string folder);

			// load data
            bool load_training_data_from_file(std::string path);
            bool load_training_data_from_folder(std::string folder);

			// clear data
            void clear_training_data();

			// dump model info to console
            void get_detailed_info() const;
            void get_summary_info() const;

			// return mode of Model (classification, regression, time-series etc).
			std::string mode() const;
            
            int get_input_dim() const { return _training_params.input_dim; }
            int get_output_dim() const { return _training_params.output_dim; }
            TrainingData<DataVector, T>& get_training_data() { return _training_data; }
            mlp::Parameters& get_params() { return _training_params; }

            
//            TrainingData<DataVector, T>& get_training_data() const { return _training_data; }
            
//            void setImplementation(typename MLImplBase<T>::Ptr ml_impl);

		private:
			std::string _name;
			bool _trained = false;

            TrainingData<DataVector, T> _training_data;
			mlp::Parameters _training_params;

            typename MLImplBase<DataVector, T>::Ptr _impl;
            

			// store output vector for when we are training
            DataVector _output_vector;
        };
        
        
        
        
        
        inline void dump_method_header(const std::string& method_name, const std::string& model_name) {
            std::cout << method_name << " [" << model_name << "] ";
        }
        
        template <typename DataVector, typename T>
        Model<DataVector, T>::Model(std::string model_name, int input_dim, int output_dim, typename MLImplBase<DataVector, T>::Ptr ml_impl) : _name(model_name) {
            std::cout << "Model ";
            this->_impl = ml_impl;

            _training_params.input_dim = input_dim;
            _training_params.output_dim = output_dim;
            _training_data.set_dimensions(_training_params.input_dim, _training_params.output_dim);
            get_summary_info();
        }
        
        
        // set output vector for a model
        template <typename DataVector, typename T>
        void Model<DataVector, T>::set_output_vector(const DataVector& output_vector) {
            if (verbose) {
                dump_method_header("Model::set_output_vector", _name);
                std::cout << vector_utils::to_string(output_vector);
            }
            
            if (output_vector.size() != _training_params.output_dim) {
                std::cout << " ... ERROR: data dimensions does not match output dimensions: " << _training_params.output_dim << std::endl;
                return;
            }
            _output_vector = output_vector;
            
            if(verbose) std::cout << std::endl;
        }
        
        
        
        // add data to the training set
        template <typename DataVector, typename T>
        void Model<DataVector, T>::add_training_data(const DataVector& input_vector) {
            if (verbose) {
                dump_method_header("Model::add_training_data", _name);
                std::cout << vector_utils::to_string(input_vector);
            }
            
            if (input_vector.size() != _training_params.input_dim) {
                std::cout << " ... ERROR: data dimensions does not match input dimensions: " << _training_params.input_dim << std::endl;
                return;
            }
            
            _training_data.add_sample(input_vector, _output_vector);
            
            if (verbose) {
                std::cout << " -> ";
                std::cout << vector_utils::to_string(_output_vector);
                std::cout << std::endl;
            }
        }
        
        
        
        // train a model
        template <typename DataVector, typename T>
        bool Model<DataVector, T>::train() {
            std::cout << "----------------------------------------------------------------" << std::endl;
            dump_method_header("Model::train", _name);
            
            if(!_impl) {
                std::cout << "ERROR: no implementation assigned for " << _name;
                return false;
            }
            
            _training_data.calc_range();
            
            if (_training_data.size() > 0) _trained = _impl->train(_training_data, _training_params);
            
            if (_trained) {
                std::cout << "Trained" << std::endl;
                std::cout << "----------------------------------------------------------------" << std::endl;
                return true;    // success
            }
            else {
                std::cout << "WARNING: Failed to train" << std::endl;
                return false;   // fail
            }
            return true;
        }
        
        
        // predict data for a model
        template <typename DataVector, typename T>
        void Model<DataVector, T>::predict(const DataVector& input_vector, DataVector& output_vector) {
            if (verbose) {
                dump_method_header("Model::predict", _name);
                std::cout << vector_utils::to_string(input_vector);
            }
            if (input_vector.size() != _training_params.input_dim) {
                std::cout << " ... ERROR: data dimensions does not match input dimensions: " << _training_params.input_dim << std::endl;
                return;
            }
            
            if(!_impl) {
                std::cout << "ERROR: no implementation assigned for " << _name;
                return;
            }

            
            // if trained, predict output
            if (_trained) {
                output_vector.resize(_training_params.output_dim);
                
                DataVector input_vector_norm(input_vector.size());
                DataVector output_vector_norm(output_vector.size());
                
                // normalize input
                vector_utils::normalize(input_vector, _training_data.get_input_min_values(), _training_data.get_input_max_values(), (T)_training_data.normalize_min, (T)_training_data.normalize_max, input_vector_norm);
                
                // run network
                _impl->predict(input_vector_norm, output_vector_norm);
                
                // unnormalize output
                vector_utils::unnormalize(output_vector_norm, _training_data.get_output_min_values(), _training_data.get_output_max_values(), (T)_training_data.normalize_min, (T)_training_data.normalize_max, output_vector);

                
                // dump to console
                if (verbose) {
                    std::cout << " -> ";
                    std::cout << vector_utils::to_string(output_vector);
                }
            }
            else {
                std::cout << " ... ERROR: Network not trained" << std::endl;
                return;
            }
            
            if(verbose) std::cout << std::endl;
        }
        
        
        // save and load data
        template <typename DataVector, typename T>
        bool Model<DataVector, T>::save_training_data_to_file(std::string path) {
            std::cout << "Model::save_training_data_to_file [" << _name << "] " << path;
            
            if (_training_data.save(path)) {
                std::cout << " ... SUCCESS" << std::endl;
                return true;
            }

            std::cout << " ... FAIL" << std::endl;
            return false;
        }
        
        
        template <typename DataVector, typename T>
        bool Model<DataVector, T>::save_training_data_to_folder(std::string folder) {
            std::cout << "Model::save_training_data_to_folder [" << _name << "] " << folder << std::endl;
            if (folder.back() != '/') folder += '/';
            return save_training_data_to_file(folder + _name + ".txt");
        }
        
        
        template <typename DataVector, typename T>
        bool Model<DataVector, T>::load_training_data_from_file(std::string path) {
            std::cout << "Model::load_training_data_from_file [" << _name << "] " << path;
            
            if (_training_data.load(path)) {
                std::cout << " ... SUCCESS" << std::endl;
                return true;
            }

            std::cout << " ... FAIL" << std::endl;
            return false;
        }
        
        
        template <typename DataVector, typename T>
        bool Model<DataVector, T>::load_training_data_from_folder(std::string folder) {
            std::cout << "Model::load_training_data_from_folder [" << _name << "] " << folder << std::endl;
            if (folder.back() != '/') folder += '/';
            return load_training_data_from_file(folder + _name + ".txt");
        }
        
        
        template <typename DataVector, typename T>
        void Model<DataVector, T>::clear_training_data() {
            dump_method_header("Model::clear_training_data", _name);
            
            _training_data.clear();
        }
        
        
        template <typename DataVector, typename T>
        void Model<DataVector, T>::get_detailed_info() const {
            if(!_impl) {
                std::cout << "ERROR: no implementation assigned for " << _name;
                return;
            }
            
            std::cout << "----------------------------------------------------------------" << std::endl;
            std::cout << _name << std::endl;
            std::cout << "Training data: " << std::endl;
            std::cout << _training_data.to_string(" | ", true) << std::endl;
            std::cout << _impl->to_string(" | ", true) << std::endl;
            std::cout << "----------------------------------------------------------------" << std::endl;
        }
        
        
        template <typename DataVector, typename T>
        void Model<DataVector, T>::get_summary_info() const {
            if(!_impl) {
                std::cout << "ERROR: no implementation assigned for " << _name;
                return;
            }

            dump_method_header("", _name);
            std::cout << " | " << _impl->type();
            std::cout << " | input_dims: " << _training_params.input_dim;
            std::cout << " | output_dims: " << _training_params.output_dim;
            
            std::cout << " | Data: ";
            std::cout << _training_data.to_string();
            
            std::cout << std::endl;
        }
        
        
        template <typename DataVector, typename T>
        std::string Model<DataVector, T>::mode() const {
            return "Regression";
        }
        
        
//        template <typename DataVector, typename T>
//        void Model<DataVector, T>::setImplementation(typename MLImplBase<T>::Ptr ml_impl) {
//            this->_impl = ml_impl;
//        }

        
    }
}
