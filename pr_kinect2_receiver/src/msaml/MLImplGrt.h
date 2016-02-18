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

#include "GRT/GRT.h"

namespace msa {
namespace ml {

// Vector type is templated (e.g. could be std::vector<float> or std::vector<double> or any other class that has same API
template <typename DataVector>
class MLImplGrt : public MLImplBase<DataVector> {
public:

    std::string type() const override {
        return "GRT";
    }

    bool train(const vector<DataVector>& inputs, const vector<DataVector>& outputs, const mlp::ModelParameters& model_params, const mlp::TrainingParameters& train_params) override {
        destroy();
        
        // create data
        GRT::RegressionData gdata;
        gdata.setInputAndTargetDimensions(model_params.input_dim, model_params.output_dim);
        for (int i = 0; i < outputs.size(); i++) gdata.addSample(inputs[i], outputs[i]);


        // create ann
        GRT::MLP mlp;
        
        // TODO: use these from params struct
        mlp.setInputLayerActivationFunction(GRT::Neuron::LINEAR);
        mlp.setHiddenLayerActivationFunction(GRT::Neuron::BIPOLAR_SIGMOID);
        mlp.setOutputLayerActivationFunction(GRT::Neuron::BIPOLAR_SIGMOID);
        
        if(model_params.hidden_dims.size() > 1) {
            std::cout << "WARNING: MLImplGrt::train can only have 1 hidden layer, desired " << model_params.hidden_dims.size();
        }
        mlp.init(model_params.input_dim, model_params.hidden_dims[0], model_params.output_dim);

        //Set the training settings
        mlp.setMaxNumEpochs(train_params.max_epochs); //max number of epochs (1 epoch is 1 complete iteration of the training data) that are allowed
        mlp.setMinChange(train_params.min_delta); //min change allowed in training error between any two epochs
        mlp.setNumRandomTrainingIterations(train_params.num_train_sessions); //number of times the MLP will be trained, each training iteration starts with new random values
        mlp.setUseValidationSet(train_params.use_validation); //sets aside a small portiion of the training data to be used as a validation set to mitigate overfitting
        mlp.setValidationSetSize(train_params.validation_size); //Use 20% of the training data for validation during the training phase
        mlp.setRandomiseTrainingOrder(train_params.randomize_train_order); //Randomize the order of the training data so that the training algorithm does not bias the training
        mlp.enableScaling(train_params.use_normalization);
        mlp.setLearningRate(train_params.learning_rate);
        mlp.setMomentum(train_params.learning_momentum);
        _pipeline.setRegressifier(mlp);


        // train
        _pipeline.train(gdata);

        return true;
    }


    void predict(const DataVector& input_vector_norm, DataVector& output_vector_norm) const override {
        _pipeline.predict(input_vector_norm);
        output_vector_norm = _pipeline.getRegressionData();
    }

    void destroy() override {
        _pipeline.reset();
    }

private:
    mutable GRT::GestureRecognitionPipeline _pipeline;  // wrapper for classifier and pre/post processing modules
};

}
}
