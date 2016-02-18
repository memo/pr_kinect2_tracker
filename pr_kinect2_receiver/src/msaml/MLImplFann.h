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

#include "doublefann.h"  // TODO: need to manage including float vs double

namespace msa {
    namespace ml {
        
        // Vector type is templated (e.g. could be std::vector<float> or std::vector<double> or any other class that has same API
        template <typename DataVector, typename T>
        class MLImplFann : public MLImplBase<DataVector, T> {
        public:
            std::string type() const override {
            return "FANN";
        }
        
        bool train(const TrainingData<DataVector, T>& data, const mlp::ModelParameters& model_params, const mlp::TrainingParameters& train_params) override {
        destroy();
        
        // create data
        fann_train_data * fdata = fann_create_train(data.size(), model_params.input_dim, model_params.output_dim);
        
        for (int i = 0; i < data.size(); i++) {
            memcpy(fdata->input[i], &data.get_input_vectors_norm()[i], vector_utils::num_bytes(data.get_input_vectors_norm()[i]));
            memcpy(fdata->output[i], &data.get_output_vectors_norm()[i], vector_utils::num_bytes(data.get_output_vectors_norm()[i]));
        }
        
        
        // create ann
        vector<unsigned int> layers;
        layers.push_back(model_params.input_dim);
        for(auto hidden_dim : model_params.hidden_dims) { layers.push_back(hidden_dim); }
        layers.push_back(model_params.output_dim);
        _ann = fann_create_standard_array(layers.size(), layers.data());
        if (!_ann) return false;
        
        fann_set_learning_rate(_ann, train_params.learning_rate);
        fann_set_learning_momentum(_ann, train_params.learning_momentum);
        fann_set_training_algorithm(_ann, FANN_TRAIN_RPROP);
        fann_set_activation_function_hidden(_ann, FANN_SIGMOID_SYMMETRIC);
        fann_set_activation_function_output(_ann, FANN_SIGMOID_SYMMETRIC);
        
        // train
        fann_train_on_data(_ann, fdata, train_params.max_epochs, train_params.epochs_between_reports, 0);
        
        fann_destroy_train(fdata);
        
        return true;
    }
    
    
    void predict(const DataVector& input_vector_norm, DataVector& output_vector_norm) const override {
    // run network
    T *fann_output = fann_run(_ann, (T*)input_vector_norm.data());
    for(int i=0; i<output_vector_norm.size(); i++) output_vector_norm[i] = fann_output[i];
//    memcpy(output_vector_norm.data(), fann_output, vector_utils::num_bytes(output_vector_norm));
}

void destroy() override {
if (_ann) {
    fann_destroy(_ann);
    _ann = NULL;
}
}

private:
fann *_ann = NULL;
};
}
}
