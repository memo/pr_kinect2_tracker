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
// Base class / interface for an implementation of mlp
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

            struct Parameters {
                unsigned int input_dim = 2;		// dimensions of input vector
                unsigned int output_dim = 2;	// dimensions of output vector
                unsigned int num_layers = 3;	// number of layers (including input and output)
                unsigned int hidden_dim = 3;	// number of hidden neurons
                float learning_rate = 0.1f;
                float learning_momentum = 0.9f;
                float desired_error = 1.0e-5f;
                unsigned int max_epochs = 10000;
                unsigned int epochs_between_reports = 100;
//                float activation_steepness_hidden = 0.5f;
//                float activation_steepness_output = 0.5f;
//                ActivationFunction activation_function_hidden = ActivationFunction::kBinaryLogistic;
//                ActivationFunction activation_function_output = ActivationFunction::kBinaryLogistic;
//                int training_algorithm = TrainingAlgorithm::kBackpropBatch;
            };
        };
        
        
        // Vector type is templated (e.g. could be std::vector<float> or std::vector<double> or any other class that has same API
        template <typename DataVector, typename T>
        class MLImplBase {
		public:
            typedef std::shared_ptr<MLImplBase> Ptr;
            
			virtual ~MLImplBase() { destroy(); }

			// create model with name and dimensions
//			virtual void setDimensions(int idims, int odims) = 0;

			virtual std::string type() const = 0;

			// train data on a model
            virtual bool train(const TrainingData<DataVector, T>& training_data, const mlp::Parameters& params) = 0;

			// predict data for a model (assumes input and output is already normalized)
            virtual void predict(const DataVector& input_vector_norm, DataVector& output_vector_norm) const = 0;

			// clean up
			virtual void destroy() {};

			// get info
            virtual std::string to_string(std::string separator = " | ", bool verbose = false) const { return ""; }
		};
	}
}
