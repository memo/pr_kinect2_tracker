//                                     __
//    ____ ___  ___  ____ ___  ____   / /__   __
//   / __ `__ \/ _ \/ __ `__ \/ __ \ / __/ | / /
//  / / / / / /  __/ / / / / / /_/ // /_ | |/ /
// /_/ /_/ /_/\___/_/ /_/ /_/\____(_)__/ |___/
//
//
// Created by Memo Akten, www.memo.tv
//
//
// Stores a set of training data points as an input+output pairs
//
//


#pragma once

#include "VectorUtils.h"

#include <vector>
#include <string>
#include <memory>
#include <cstdarg>
#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>


namespace msa {
    namespace ml {

        // Vector type is templated (e.g. could be std::vector<float> or std::vector<double> or any other class that has same API
        template <typename DataVector>
        class TrainingData {
        public:
            using DataType = typename DataVector::value_type;

            DataType normalize_min = -1;
            DataType normalize_max = 1;

            // set input output dimensions
            void set_dimensions(int input_dim, int output_dim);

            // add single data point to the training set
            void add_sample(const DataVector& input_vec, const DataVector& output_vec);

            // clear all data
            void clear();

            // file IO
            bool save(const std::string path) const;
            bool load(const std::string path);

            // update min/max values
            void normalize();

            // number of data samples
            int size() const { return (int)_input_vectors.size(); }

            // getters
            int get_input_dim() const { return _input_dim; }
            int get_output_dim() const { return _output_dim; }

            const std::vector<DataVector>& get_input_vectors() const { return _input_vectors; }
            const std::vector<DataVector>& get_output_vectors() const { return _output_vectors; }

            const std::vector<DataVector>& get_input_vectors_norm() const { return _input_vectors_norm; }
            const std::vector<DataVector>& get_output_vectors_norm() const { return _output_vectors_norm; }

            const DataVector& get_input_min_values() const { return _input_min; }
            const DataVector& get_input_max_values() const { return _input_max; }
            const DataVector& get_output_min_values() const { return _output_min; }
            const DataVector& get_output_max_values() const { return _output_max; }

            // get info
            std::string to_string(std::string separator = " | ", bool verbose = false) const;

        private:
            int _input_dim;
            int _output_dim;

            std::vector<DataVector> _input_vectors;   // original input vectors
            std::vector<DataVector> _output_vectors;  // original output vectors


            DataVector _input_min;  // minimum input values
            DataVector _input_max;  // maximum input values
            DataVector _output_min; // minimum output values
            DataVector _output_max; // maximum output values

            std::vector<DataVector> _input_vectors_norm;   // normalized input vectors
            std::vector<DataVector> _output_vectors_norm;  // normalized output vectors
        };



        template <typename DataVector>
        void TrainingData<DataVector>::set_dimensions(int input_dim, int output_dim) {
            clear();
            _input_dim = input_dim;
            _output_dim = output_dim;
        }



        template <typename DataVector>
        void TrainingData<DataVector>::add_sample(const DataVector& input_vec, const DataVector& output_vec) {
            _input_vectors.push_back(input_vec);
            _output_vectors.push_back(output_vec);
        }



        template <typename DataVector>
        void TrainingData<DataVector>::clear() {
            _input_vectors.clear();
            _output_vectors.clear();
        }



        template <typename DataVector>
        std::string TrainingData<DataVector>::to_string(std::string separator, bool verbose) const {
            std::stringstream s;
            s << "input dim: " << _input_dim << separator;
            s << "output dim: " << _output_dim << separator;
            s << "size : " << size() << separator;
            if (verbose) {
                s << "input range: " << vector_utils::to_string(_input_min) << " - " << vector_utils::to_string(_input_max) << separator;
                s << "output range: " << vector_utils::to_string(_output_min) << " - " << vector_utils::to_string(_output_max) << separator;
                s << std::endl;
                for(int i=0; i<_input_vectors.size(); i++) {
                    s << vector_utils::to_string(_input_vectors[i]) << " -> " << vector_utils::to_string(_output_vectors[i]) << std::endl;
                }
            }
            return s.str();
        }



        template <typename DataVector>
        bool TrainingData<DataVector>::save(const std::string path) const {
            std::ofstream file(path);
            if (!file.is_open()) return false;

            file << size() << " " << _input_dim << " " << _output_dim << std::endl;
            for(int i=0; i<_input_vectors.size(); i++) {
                for (auto&& v : _input_vectors[i]) file << v << " ";
                file << std::endl;
                for (auto&& v : _output_vectors[i]) file << v << " ";
                file << std::endl;
            }
            file.close();
            return true;
        }



        template <typename DataVector>
        bool TrainingData<DataVector>::load(const std::string path) {
            std::ifstream file(path);
            if (!file.is_open()) return false;

            // read header line
            std::string line;
            if (!getline(file, line)) {
                file.close();
                return false;
            }

            std::istringstream iss(line);
            int num_samples, input_dim, output_dim;
            iss >> num_samples >> input_dim >> output_dim;		// read info from header line
            std::cout << std::endl << "num_samples:" << num_samples << " input_dim:" << input_dim << " output_dim:" << output_dim;

            // if dimensions don't match, report error and return
            if (input_dim != _input_dim) {
                std::cout << " ... ERROR input dimensions don't match" << std::endl;
                file.close();
                return false;
            }
            if (output_dim != _output_dim) {
                std::cout << " ... ERROR input dimensions don't match" << std::endl;
                file.close();
                return false;
            }
            std::cout << std::endl;

            clear();
            DataVector input_vector(input_dim);
            DataVector output_vector(output_dim);
            DataType f;

            while (true) {

                // read input row
                if (getline(file, line)) {
                    std::istringstream iss(line);
                    int i = 0;
                    while (iss >> f && i < input_dim) input_vector[i++] = f;	// TODO: warning if i goes above input_dim
                }
                else {
                    break;
                }

                // read output row
                if (getline(file, line)) {
                    std::istringstream iss(line);
                    int i = 0;
                    while (iss >> f && i < output_dim) output_vector[i++] = f;	// TODO: warning if i goes above output_dim
                }
                else {
                    break;
                }
                add_sample(input_vector, output_vector);

            }

            file.close();
            return true;
        }



        template <typename DataVector>
        void TrainingData<DataVector>::normalize() {
            // calculate range for input and output data
            vector_utils::get_range(_input_vectors, _input_min, _input_max);
            vector_utils::get_range(_output_vectors, _output_min, _output_max);

            // hacky (but quick?) way of initializing size of all normalized input and output data (including data entries)
            _input_vectors_norm = _input_vectors;
            _output_vectors_norm = _output_vectors;

//            _input_vectors_norm.resize(_input_vectors.size());//, _input_vectors[0].size());
//            _output_vectors_norm.resize(_output_vectors.size());//, _output_vectors[0].size());


            for(int i=0; i<size(); i++) {
                vector_utils::normalize(_input_vectors[i], _input_min, _input_max, normalize_min, normalize_max, _input_vectors_norm[i]);
                vector_utils::normalize(_output_vectors[i], _output_min, _output_max, normalize_min, normalize_max, _output_vectors_norm[i]);
            }
        }


    }
}
