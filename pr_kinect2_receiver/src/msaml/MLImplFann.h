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
//#include "floatfann.h"

namespace msa {
    namespace ml {
        
        // Vector type is templated (e.g. could be std::vector<float> or std::vector<double> or any other class that has same API
        template <typename DataVector>
        class MLImplFann : public MLImplBase<DataVector> {
        public:
            std::string type() const override {
            return "FANN";
        }
        
        bool train(const vector<DataVector>& inputs, const vector<DataVector>& outputs, const mlp::ModelParameters& model_params, const mlp::TrainingParameters& train_params) override {
        destroy();
        
        int num_training_data = inputs.size();
        
        // create data structure
        fann_train_data * fdata = fann_create_train(num_training_data, model_params.input_dim, model_params.output_dim);
        
        // copy data into fann structure
        for (int i=0; i<num_training_data; i++) {
            for(int j=0; j<inputs[i].size(); j++) { fdata->input[i][j] = inputs[i][j]; }
            for(int j=0; j<outputs[i].size(); j++) { fdata->output[i][j] = outputs[i][j]; }
            //            memcpy(fdata->input[i], &inputs[i], vector_utils::num_bytes(inputs[i]));
            //            memcpy(fdata->output[i], &outputs[i], vector_utils::num_bytes(outputs[i]));
        }
        
        
        // build layers dimensions
        vector<unsigned int> layers;
        layers.push_back(model_params.input_dim);
        for(auto hidden_dim : model_params.hidden_dims) { layers.push_back(hidden_dim); }
        layers.push_back(model_params.output_dim);
        
        // create ann
        _ann = fann_create_standard_array(layers.size(), layers.data());
        if (!_ann) return false;
        
        fann_set_learning_rate(_ann, train_params.learning_rate);
        fann_set_learning_momentum(_ann, train_params.learning_momentum);
        fann_set_training_algorithm(_ann, FANN_TRAIN_BATCH);//FANN_TRAIN_RPROP);//FANN_TRAIN_QUICKPROP);//);
        fann_set_activation_function_hidden(_ann, FANN_SIGMOID_SYMMETRIC_STEPWISE);
        fann_set_activation_function_output(_ann, FANN_SIGMOID_SYMMETRIC_STEPWISE);
        
        // train
        fann_train_on_data(_ann, fdata, train_params.max_epochs, train_params.epochs_between_reports, 0);
        
        fann_destroy_train(fdata);
        
        return true;
    }
    
    
    void predict(const DataVector& input_vector, DataVector& output_vector) const override {
    // run network
    fann_type* fann_output = fann_run(_ann, (fann_type*)input_vector.data());
    for(int i=0; i<output_vector.size(); i++) { output_vector[i] = fann_output[i]; }
    //memcpy(output_vector.data(), fann_output, vector_utils::num_bytes(output_vector));
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
