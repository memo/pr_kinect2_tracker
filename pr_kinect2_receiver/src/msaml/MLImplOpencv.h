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

#include "opencv2/opencv.hpp"

namespace msa {
    namespace ml {
        
        // Vector type is templated (e.g. could be std::vector<float> or std::vector<double> or any other class that has same API
        template <typename DataVector>
        class MLImplOpencv : public MLImplBase<DataVector> {
        public:
            using DataType = typename DataVector::value_type;

            
            std::string type() const override {
            return "OPENCV";
        }
        
        bool train(const vector<DataVector>& inputs, const vector<DataVector>& outputs, const mlp::ModelParameters& model_params, const mlp::TrainingParameters& train_params) override {
        destroy();
        
        // create data structures
        cv::Mat cv_inputs(inputs.size(), model_params.input_dim, CV_64F);
        cv::Mat cv_outputs(outputs.size(), model_params.output_dim, CV_64F);
        
        // copy data into cv::Mat (TODO: could be done by just referencing data from cv::Mat?)
        for (int i=0; i<inputs.size(); i++) {
//            auto cv_input_row = cv_inputs.row(i);
//            
//            for(int j=0; j<inputs[i].size(); j++) cv_inputs.ptr<float>(i)
            memcpy(cv_inputs.ptr<DataType>(i), &inputs[i], vector_utils::num_bytes(inputs[i]));
            memcpy(cv_outputs.ptr<DataType>(i), &outputs[i], vector_utils::num_bytes(outputs[i]));
        }
        
        // build layers
        int num_layers = 2 + model_params.hidden_dims.size();
        cv::Mat layers(num_layers, 1, CV_32S);
        layers.row(0) = cv::Scalar(model_params.input_dim);
        layers.row(num_layers - 1) = cv::Scalar(model_params.output_dim);
        for(int i=0; i<model_params.hidden_dims.size(); i++) layers.row(i+1) = model_params.hidden_dims[i];
            
            
            CvANN_MLP_TrainParams params;
        CvTermCriteria criteria;
        criteria.max_iter = train_params.max_epochs;
        criteria.epsilon = train_params.min_delta;
        criteria.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
        params.train_method = CvANN_MLP_TrainParams::RPROP; // CvANN_MLP_TrainParams::BACKPROP;  //
        params.bp_dw_scale = train_params.learning_rate;
        params.bp_moment_scale = train_params.learning_momentum;
        params.term_crit = criteria;
        
        mlp.create(layers, CvANN_MLP::SIGMOID_SYM);
        
        // train
        int num_epochs = mlp.train(cv_inputs, cv_outputs, cv::Mat(), cv::Mat(), params);
        std::cout << "MLImplOpencv trained in " << num_epochs << " epochs";
        
        return true;
    }
    
    
    void predict(const DataVector& input_vector, DataVector& output_vector) const override {
    cv::Mat cv_input(1, { static_cast<int>(input_vector.size()) }, CV_64F);
    cv::Mat cv_output(1, { static_cast<int>(output_vector.size()) }, CV_64F);
    mlp.predict(cv_input, cv_output);
    memcpy(output_vector.data(), cv_output.ptr<float>(), vector_utils::num_bytes(output_vector));
    
}

void destroy() override {
}

private:
CvANN_MLP mlp;

};

}
}
