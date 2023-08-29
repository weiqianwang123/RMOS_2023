//
// Created by Wang on 23-6-26.
//

#ifndef RMOS_ONNX_CLASSIFIER_HPP
#define RMOS_ONNX_CLASSIFIER_HPP

#include <opencv2/opencv.hpp>
#include "onnx/onnxruntime_cxx_api.h"

#include "../../../Base/armor.hpp"
#include "../../detector_interfaces/classifier_interface.hpp"

namespace detector {
    class OnnxClassifier : public ClassifierInterface {
    public:
        bool classifyArmors(const cv::Mat &image, std::vector <base::Armor> &armors) override;


    private:
        int classifyArmor(const cv::Mat &num_roi, double &confidence) override;


        void gamma(const cv::Mat & src, cv::Mat &dst, float fGamma)
        {
            unsigned char lut[256];

            for (int i = 0; i < 256; i++)
            {
                float normalize = (float)(i/255.0);
                lut[i] = cv::saturate_cast<uchar>(pow(normalize, fGamma) * 255.0f);
            }

            src.copyTo(dst);
            cv::MatIterator_<cv::Vec3b> it,end;
            for (it = dst.begin<cv::Vec3b>(), end = dst.end<cv::Vec3b>(); it != end; it++)
            {
                (*it)[0] = lut[((*it)[0])];
                (*it)[1] = lut[((*it)[1])];
                (*it)[2] = lut[((*it)[2])];
            }

        }


    public:
        explicit OnnxClassifier(char model_path[], cv::Size infer_size, float conf_thres);

        OnnxClassifier();


        void verify_input_output_count(OrtSession *session);

        bool classify(cv::Mat &input_image, int &label, float &conf, std::string &name);

        bool preprocessing(cv::Mat &image, float **output, size_t *output_count);

        static void letterbox(const cv::Mat &image, cv::Mat &outImage,
                              const cv::Size &newShape,
                              const cv::Scalar &color,
                              bool auto_,
                              bool scaleFill,
                              int stride);

        int run_inference(OrtSession *session, int &label, float &conf);

        void ort_release();

    private:
        const OrtApi *g_ort = NULL;
        ORTCHAR_T *model_path_;
        OrtEnv *env_;
        OrtSessionOptions *session_options_;
        OrtSession *session_;

        cv::Mat input_image_;
        cv::Mat process_image_;
        cv::Size infer_size_;
        float conf_thres_ = 0.5;


    };
}
#endif //RMOS_ONNX_CLASSIFIER_HPP
