//
// Created by nuc12 on 23-6-26.
//
#pragma once
#include <opencv2/opencv.hpp>
#include <utility>
#include "Dectector/classifier/onnx_classifier/onnx_classifier.hpp"
#include <cmath>


namespace detector {

#define ORT_ABORT_ON_ERROR(expr)                                 \
    do {                                                         \
        OrtStatus* onnx_status = (expr);                         \
        if (onnx_status != NULL) {                               \
        const char* msg = g_ort->GetErrorMessage(onnx_status);   \
        std::cout << msg << std::endl;                           \
        g_ort->ReleaseStatus(onnx_status);                       \
        abort();                                                 \
        }                                                        \
    } while (0);

    std::string robot_names[15]={"Base","Hero","Engineer","Infantry3","Infantry4",
                                 "Infantry5","Sentry","Outpost","Error1","Error2",
                                 "Error4","Balance3","Balance4"};

    bool OnnxClassifier::classifyArmors(const cv::Mat &image, std::vector <base::Armor> &armors)
    {
        for (auto &armor : armors)
        {
            cv::Mat temp;
            double box_height_enlarge = 1.0; // 这里的参数暂时先直接放在函数里
            armor.rect.y -= armor.rect.height / 2.0 * box_height_enlarge;
            armor.rect.height *= 2.0 * box_height_enlarge;
            armor.rect &= cv::Rect2d(cv::Point(0, 0), image.size());
            temp = image(cv::Rect2d(armor.rect));
            gamma(temp, temp, 0.6);
            cv::Mat num_image = cv::Mat::zeros(144,240,CV_8UC3);
            cv::resize(temp,num_image,num_image.size());
            //cv::imshow("num",num_image);
            double confidence = 0;
            armor.num_id = this->classifyArmor(temp,confidence);
            armor.confidence = confidence;
            switch (armor.num_id)
            {

                    case 0:
                    case 1:
                    case 11:
                    case 12:
                    case 13:
                        armor.type = base::ArmorType::BIG;
                        break;

                    case 2:
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                    case 7:
                        armor.type = base::ArmorType::SMALL;
                        break;

                    default:
                        armor.type = base::ArmorType::WRONG;
                        break;
            }

        }
        armors.erase(
                std::remove_if(
                        armors.begin(), armors.end(),
                        [this](const base::Armor & armor) {

                            if ( armor.type == base::ArmorType::WRONG) {
                                return true;
                            }
                            else
                            {
                                return false;
                            }

                        }),
                armors.end());


        return true;
    }


    int OnnxClassifier::classifyArmor(const cv::Mat &num_roi, double &confidence)
    {
        int getlabel = -1;
        float getconf = -1.0;
        std::string getname = "None";
        bool cls_success = false;
        cv::Mat temp = num_roi;
        cls_success = this->classify(temp,getlabel,getconf,getname);
        if(cls_success)
        {

            confidence = getconf;
            return getlabel;

        }
        else
        {
            return -1;
        }

    }

    void OnnxClassifier::verify_input_output_count(OrtSession* session) {
        size_t count;
        ORT_ABORT_ON_ERROR(g_ort->SessionGetInputCount(session, &count));
        assert(count == 1);
        ORT_ABORT_ON_ERROR(g_ort->SessionGetOutputCount(session, &count));
        assert(count == 1);
    }

    OnnxClassifier::OnnxClassifier(char model_path[], cv::Size infer_size, float conf_thres) {
        g_ort = OrtGetApiBase()->GetApi(ORT_API_VERSION);
        model_path_ = model_path;
        infer_size_ = infer_size;
        conf_thres_ = conf_thres;
        //std::cout<<"onnx model path: "<<model_path_<<std::endl;
        ORT_ABORT_ON_ERROR(g_ort->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "test", &env_));
        ORT_ABORT_ON_ERROR(g_ort->CreateSessionOptions(&session_options_));
        g_ort->SetIntraOpNumThreads(session_options_,1);
        g_ort->SetSessionExecutionMode(session_options_,ORT_SEQUENTIAL);
        g_ort->SetSessionGraphOptimizationLevel(session_options_,ORT_ENABLE_ALL);
        ORT_ABORT_ON_ERROR(g_ort->CreateSession(env_, model_path, session_options_, &session_));
        verify_input_output_count(session_);
    }
    OnnxClassifier::OnnxClassifier()
    {
        g_ort = OrtGetApiBase()->GetApi(ORT_API_VERSION);
        char onnxmodel_path[90] = "./src/Algorithm/configure/Detector/classifier/onnx_classifier/model/modelv4_150.onnx"; //25: 0.1-0.2 50:0.1-0.4 75:0.1-0.8
        const float set_cls_conf_thres = 0.6;
        model_path_ = onnxmodel_path;
        infer_size_ =  cv::Size(54,54);
        conf_thres_ = set_cls_conf_thres;
        ORT_ABORT_ON_ERROR(g_ort->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "test", &env_));
        ORT_ABORT_ON_ERROR(g_ort->CreateSessionOptions(&session_options_));
        g_ort->SetIntraOpNumThreads(session_options_,1);
        g_ort->SetSessionExecutionMode(session_options_,ORT_SEQUENTIAL);
        g_ort->SetSessionGraphOptimizationLevel(session_options_,ORT_ENABLE_ALL);
        ORT_ABORT_ON_ERROR(g_ort->CreateSession(env_, model_path_, session_options_, &session_));
        verify_input_output_count(session_);
    }



    void OnnxClassifier::ort_release() {
        g_ort->ReleaseSessionOptions(session_options_);
        g_ort->ReleaseSession(session_);
        g_ort->ReleaseEnv(env_);
    }

    bool OnnxClassifier::classify(cv::Mat &input_image, int &label, float &conf, std::string& name) {
        input_image_ = input_image;
        process_image_ = input_image.clone();
        if(process_image_.empty())
        {
            return false;
        }
        int flag = run_inference(session_, label, conf);
        if(flag==-1)
        {
            return false;
        }
        name = robot_names[label];
        //std::cout<<conf<<std::endl;
        if(conf >= conf_thres_)return true;
        else return false;
    }

    int OnnxClassifier::run_inference(OrtSession *session, int &label, float &conf) {
        OrtMemoryInfo *memory_info;
        float *model_input;
        size_t model_input_ele_count = infer_size_.width * infer_size_.height;
        ORT_ABORT_ON_ERROR(g_ort->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &memory_info));
        const int64_t input_shape[] = {1, 1, infer_size_.width, infer_size_.height};
        const size_t input_shape_len = sizeof(input_shape) / sizeof(input_shape[0]);
        const size_t model_input_len = model_input_ele_count * sizeof(float);

        bool flag = preprocessing(process_image_,&model_input,&model_input_ele_count);
        if(flag==false)
        {
            return -1;
        }
        //std::cout<<input_shape_len<<" "<<model_input_len<<std::endl;
        OrtValue *input_tensor = NULL;
        ORT_ABORT_ON_ERROR(g_ort->CreateTensorWithDataAsOrtValue(memory_info, model_input, model_input_len, input_shape,
                                                                 input_shape_len, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
                                                                 &input_tensor));
        assert(input_tensor != NULL);

        int is_tensor;
        ORT_ABORT_ON_ERROR(g_ort->IsTensor(input_tensor, &is_tensor));
        assert(is_tensor);
        g_ort->ReleaseMemoryInfo(memory_info);

        const char *input_names[] = {"input"};
        const char *output_names[] = {"output"};

        std::array<float, 13> results_{};
        int result_{0};
        const int64_t output_shape[] = {1, 13};
        const size_t output_shape_len = sizeof(output_shape) / sizeof(output_shape[0]);

        OrtValue *output_tensor = NULL;
        ORT_ABORT_ON_ERROR(
                g_ort->CreateTensorWithDataAsOrtValue(memory_info, results_.data(), results_.size() * sizeof(float),
                                                      output_shape, output_shape_len,
                                                      ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &output_tensor));

        ORT_ABORT_ON_ERROR(
                g_ort->Run(session, NULL, input_names, (const OrtValue *const *) &input_tensor, 1, output_names, 1,
                           &output_tensor));
        assert(output_tensor != NULL);
        ORT_ABORT_ON_ERROR(g_ort->IsTensor(output_tensor, &is_tensor));
        assert(is_tensor);

        int tmp_label = -1;
        float max_conf = 0.0;
        for (size_t i = 0; i < results_.size(); i++) {
            //std::cout << i << " : "<< exp(results_[i]) << std::endl;
            if(exp(results_[i]) > max_conf)
            {
                max_conf = exp(results_[i]);
                tmp_label = i;
            }
        }
        label = tmp_label;
        conf = max_conf;
        g_ort->ReleaseValue(output_tensor);
        g_ort->ReleaseValue(input_tensor);
        free(model_input);
        return 1;
    }

    void OnnxClassifier::letterbox(const cv::Mat& image, cv::Mat& outImage,
                                   const cv::Size& newShape = cv::Size(54, 54),
                                   const cv::Scalar& color = cv::Scalar(7, 7, 7),
                                   bool auto_ = false,
                                   bool scaleFill = false,
                                   int stride = 32)
    {
        cv::Size shape = image.size();
        float r = std::min((float)newShape.height / (float)shape.height,
                           (float)newShape.width / (float)shape.width);

        float ratio[2] {r, r};
        int newUnpad[2] {(int)std::round((float)shape.width * r),
                         (int)std::round((float)shape.height * r)};

        auto dw = (float)(newShape.width - newUnpad[0]);
        auto dh = (float)(newShape.height - newUnpad[1]);

        if (auto_)
        {
            dw = (float)((int)dw % stride);
            dh = (float)((int)dh % stride);
        }
        else if (scaleFill)
        {
            dw = 0.0f;
            dh = 0.0f;
            newUnpad[0] = newShape.width;
            newUnpad[1] = newShape.height;
            ratio[0] = (float)newShape.width / (float)shape.width;
            ratio[1] = (float)newShape.height / (float)shape.height;
        }

        dw /= 2.0f;
        dh /= 2.0f;

        if (shape.width != newUnpad[0] && shape.height != newUnpad[1])
        {
            cv::resize(image, outImage, cv::Size(newUnpad[0], newUnpad[1]));
        }

        int top = int(std::round(dh - 0.1f));
        int bottom = int(std::round(dh + 0.1f));
        int left = int(std::round(dw - 0.1f));
        int right = int(std::round(dw + 0.1f));
        cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
    }

    bool OnnxClassifier::preprocessing(cv::Mat &image, float** infer_data, size_t* infer_count) {
        cv::Mat resizedImage;
        cv::Mat floatImage;
        *infer_count = infer_size_.height * infer_size_.width * 1;
        float* temp_data = (float*)malloc(*infer_count * sizeof(float));

        cv::Mat dst = cv::Mat::zeros(image.size(),image.type());
        cv::Mat m = cv::Mat::zeros(image.size(),image.type());
        int t_bright = 0;
        double t_contrast = 100;
        double contrast = t_contrast / 100.0;
        cv::addWeighted(image, contrast,m,0.0,0,dst);
        cv::addWeighted(dst,1.0,m,0,t_bright,dst);
        image = dst.clone();

        /*
        cv::namedWindow("cls",cv::WINDOW_AUTOSIZE);
        cv::imshow("cls",image);
        cv::waitKey(1);
        */
        //std::cout<<"wc"<<std::endl;

        letterbox(image, resizedImage, infer_size_,
                  cv::Scalar(7, 7, 7), false, false, 32);
        if(resizedImage.empty())
        {
            //std::cout<<"empty"<<std::endl;
            return false;
        }
        cv::cvtColor(resizedImage, resizedImage, cv::COLOR_BGR2GRAY);
        resizedImage.convertTo(floatImage, CV_32FC1, 1 / 255.0);

        //cv::namedWindow("out",cv::WINDOW_NORMAL);
        //cv::imshow("out",resizedImage);
        //cv::waitKey(0);

        int i = 0;
        for (int row = 0; row < 54; row++) {
            for (int col = 0; col < 54; col++) {
                //std::cout<<floatImage.at<float>(row,col)<<" ";
                temp_data[i] = floatImage.at<float>(row,col);
                i++;
            }
            //std::cout<<std::endl;
        }
        *infer_data = temp_data;
        return true;
    }

}
