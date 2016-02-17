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
template <typename DataVector, typename T>
class MLImplGrt : public MLImplBase<DataVector, T> {
public:

    std::string type() const override {
        return "GRT";
    }

    bool train(const TrainingData<DataVector, T>& data, const mlp::Parameters& params) override {
        destroy();

        // create data
        _gdata.setInputAndTargetDimensions(params.input_dim, params.output_dim);
        for (int i = 0; i < data.size(); i++) _gdata.addSample(data.get_input_vectors_norm()[i], data.get_output_vectors_norm()[i]);


        // create ann
        GRT::MLP mlp;

        // TODO: use these from params struct
        mlp.setInputLayerActivationFunction(GRT::Neuron::LINEAR);
        mlp.setHiddenLayerActivationFunction(GRT::Neuron::BIPOLAR_SIGMOID);// BIPOLAR_SIGMOID);
        mlp.setOutputLayerActivationFunction(GRT::Neuron::BIPOLAR_SIGMOID);
        mlp.init(params.input_dim, params.hidden_dim, params.output_dim);

        //Set the training settings
        mlp.setMaxNumEpochs(params.max_epochs); //max number of epochs (1 epoch is 1 complete iteration of the training data) that are allowed
        mlp.setMinChange(params.desired_error); //min change allowed in training error between any two epochs
        //mlp.setTrainingRate(params.learning_rate);
        mlp.setNumRandomTrainingIterations(20); //number of times the MLP will be trained, each training iteration starts with new random values
        mlp.setUseValidationSet(true); //sets aside a small portiion of the training data to be used as a validation set to mitigate overfitting
        mlp.setValidationSetSize( 20 ); //Use 20% of the training data for validation during the training phase
        mlp.setRandomiseTrainingOrder(true); //Randomize the order of the training data so that the training algorithm does not bias the training
        mlp.enableScaling(true);
        mlp.setLearningRate(params.learning_rate);
        mlp.setMomentum(params.learning_momentum);
        _pipeline.setRegressifier(mlp);


        // train
        _pipeline.train(_gdata);

        return true;
    }


    void predict(const DataVector& input_vector_norm, DataVector& output_vector_norm) const override {
        _pipeline.predict(input_vector_norm);
        output_vector_norm = _pipeline.getRegressionData();
        //				GRT::VectorDouble ov(_pipeline.getRegressionData());
    }

    void destroy() override {
        _pipeline.reset();
    }

private:
    mutable GRT::GestureRecognitionPipeline _pipeline;  // wrapper for classifier and pre/post processing modules
    GRT::RegressionData _gdata;
};

}
}
