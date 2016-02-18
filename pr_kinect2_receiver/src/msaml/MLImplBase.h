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
// Base class / interface for an implementation of an ml algorithm
//


#pragma once

#include "TrainingData.h"
#include <map>
#include <vector>

namespace msa {
    namespace ml {
        namespace mlp {
            
            // TODO: read these from OSC and set ANN parameters
            //            enum ActivationFunction {
            //                kLinear,
            //                kBinaryLogistic,    //  0...1
            //                kBipolarLogistic,   // -1...1
            //                kTanh               // -1...1
            //            };
            
            //            enum TrainingAlgorithm {
            //                kBackpropBatch,
            //                kBackpropOnline,
            //                kRprop
            //            };
            
            struct ModelParameters {
                int input_dim = 2;                  // dimensions of input vector
                int output_dim = 2;                 // dimensions of output vector
                vector<int> hidden_dims = { 3 };	// dimensions per hidden layer
            };
            
            struct TrainingParameters {
                float learning_rate = 0.1f;
                float learning_momentum = 0.1f;
                float min_delta = 1.0e-5f;
                int max_epochs = 10000;
                int epochs_between_reports = 100;
                //                float activation_steepness_hidden = 0.5f;
                //                float activation_steepness_output = 0.5f;
                //                ActivationFunction activation_function_hidden = ActivationFunction::kBinaryLogistic;
                //                ActivationFunction activation_function_output = ActivationFunction::kBinaryLogistic;
                //                int training_algorithm = TrainingAlgorithm::kBackpropBatch;
                int num_train_sessions = 20;    // how many times to randomly train (with different start weights)
                bool use_validation = true;
                int validation_size = 10;    // what % of data to use for validation
                bool randomize_train_order = true;
                bool use_normalization = true;
            };
        };
        
        
        // Vector type is templated (e.g. could be std::vector<float> or std::vector<double> or any other class that has same API
        template <typename DataVector>
        class MLImplBase {
        public:
            typedef std::shared_ptr<MLImplBase> Ptr;
            
            virtual ~MLImplBase() { destroy(); }
            
            virtual std::string type() const = 0;
            
            // train data on a model
            virtual bool train(const vector<DataVector>& inputs, const vector<DataVector>& outputs, const mlp::ModelParameters& model_params, const mlp::TrainingParameters& train_params) = 0;
            
            // predict data for a model (assumes input and output is already normalized)
            // output_vector must be pre-allocated!
            virtual void predict(const DataVector& input_vector, DataVector& output_vector) const = 0;
            
            // clean up
            virtual void destroy() {};
            
            // get info
            virtual std::string to_string(std::string separator = " | ", bool verbose = false) const { return ""; }
        };
    }
}

