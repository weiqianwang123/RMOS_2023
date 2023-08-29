//
// Created by Wang on 23-6-15.
//

#ifndef RMOS_CLASSFIER_INTERFACE_HPP
#define RMOS_CLASSFIER_INTERFACE_HPP

#include <opencv2/core.hpp>

namespace detector
{
    class ClassifierInterface{

    public:
        virtual bool classifyArmors(const cv::Mat &image,std::vector<base::Armor>& armors) = 0;
    private:
        virtual int classifyArmor(const cv::Mat &num_roi,double& confidence) = 0;
    };

} // namespace detector


#endif //RMOS_CLASSFIER_INTERFACE_HPP
