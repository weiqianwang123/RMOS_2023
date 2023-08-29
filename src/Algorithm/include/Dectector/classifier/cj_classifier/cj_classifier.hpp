//
// Created by Wang on 23-6-19.
//

#ifndef RMOS_CJ_CLASSIFIER_HPP
#define RMOS_CJ_CLASSIFIER_HPP



// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "../../../Base/armor.hpp"
#include "../../detector_interfaces/classifier_interface.hpp"

namespace detector
{
    class CjClassifier : public ClassifierInterface{
    public:
        CjClassifier()
        {
            const std::string model_path = "./src/Algorithm/configure/Detector/classifier/cj_classifier/model/mlp.onnx";
            const std::string label_path = "./src/Algorithm/configure/Detector/classifier/cj_classifier/model/label.txt";
            net_ = cv::dnn::readNetFromONNX(model_path);
            std::ifstream label_file(label_path);
            std::string line;
            while (std::getline(label_file, line)) {
                class_names_.push_back(line);
            }
            threshold_ = 0.7;
            ignore_classes_.emplace_back("base");
            ignore_classes_.emplace_back("negative");



        }

        bool classifyArmors(const cv::Mat &image,std::vector<base::Armor>& armors) override;


    private:
        int classifyArmor(const cv::Mat &num_roi,double& confidence) override;

        cv::dnn::Net net_;
        std::vector<std::string> class_names_;
        std::vector<std::string> ignore_classes_;
        double threshold_;



    };
}






#endif //RMOS_CJ_CLASSIFIER_HPP
