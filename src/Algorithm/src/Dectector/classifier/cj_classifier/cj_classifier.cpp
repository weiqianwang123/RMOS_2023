//
// Created by Wang on 23-6-19.
//
#include "Dectector/classifier/cj_classifier/cj_classifier.hpp"



namespace detector
{
    bool CjClassifier::classifyArmors(const cv::Mat &image,std::vector<base::Armor>& armors)
    {
        // Light length in image
        const int light_length = 12;
        // Image size after warp
        const int warp_height = 28;
        const int small_armor_width = 32;
        const int large_armor_width = 54;
        // Number ROI size
        const cv::Size roi_size(20, 28);

        for (auto & armor : armors)
        {
            // Warp perspective transform
            cv::Point2f lights_vertices[4] ={
                    armor.left.down, armor.left.up, armor.right.up,
                    armor.right.down};

            const int top_light_y = (warp_height - light_length) / 2 - 1;
            const int bottom_light_y = top_light_y + light_length;
            const int warp_width = armor.type == base::SMALL ? small_armor_width : large_armor_width;
            cv::Point2f target_vertices[4] ={
                    cv::Point(0, bottom_light_y),
                    cv::Point(0, top_light_y),
                    cv::Point(warp_width - 1, top_light_y),
                    cv::Point(warp_width - 1, bottom_light_y),
            };
            cv::Mat number_image;
            auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
            cv::warpPerspective(image, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

            // Get ROI
            number_image =
                    number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

            // Binarize
            cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
            cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            double confidence = 0;
            armor.num_id = this->classifyArmor(number_image, confidence);
            armor.number = class_names_[armor.num_id];
            armor.confidence = confidence;

            cv::imshow("2",number_image);

        }


               armors.erase(
                std::remove_if(
                        armors.begin(), armors.end(),
                        [this](const base::Armor & armor) {
                            if (armor.confidence < threshold_) {
                                return true;
                            }

                            for (const auto & ignore_class : ignore_classes_) {
                                if (armor.number == ignore_class) {
                                    return true;
                                }
                            }

                            bool mismatch_armor_type = false;
                            if (armor.type == base::BIG) {
                                mismatch_armor_type =
                                        armor.number == "outpost" || armor.number == "2" || armor.number == "guard";
                            } else if (armor.type == base::SMALL) {
                                mismatch_armor_type = armor.number == "1" || armor.number == "base";
                            }
                            return mismatch_armor_type;
                        }),
                armors.end());

        return true;
    }

    int CjClassifier::classifyArmor(const cv::Mat &num_roi, double &confidence)
    {

        cv::Mat number_image = num_roi.clone();
        // Normalize
        number_image = number_image / 255.0;

        // Create blob from image
        cv::Mat blob;
        cv::dnn::blobFromImage(number_image, blob);

        // Set the input blob for the neural network
        net_.setInput(blob);
        // Forward pass the image blob through the model
        cv::Mat outputs = net_.forward();

        // Do softmax
        float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat softmax_prob;
        cv::exp(outputs - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;


        cv::Point class_id_point;
        minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;

        return label_id;


    }
}