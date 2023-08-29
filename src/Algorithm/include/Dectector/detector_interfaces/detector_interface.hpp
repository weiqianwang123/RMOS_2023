//
// Created by Wang on 23-6-15.
//

#ifndef RMOS_DETECTOR_INTERFACE_HPP
#define RMOS_DETECTOR_INTERFACE_HPP

#include "../../Base/armor.hpp"

namespace detector
{

    class DetectorInterface
    {
    public:
        /**
        *  @param  image   输入的原图像
        *  @param  armors  得到的装甲板集
        *  @return 是否识别的到目标
        */
            virtual bool detectArmors(const cv::Mat & image, std::vector<base::Armor>& armors) = 0;
            virtual bool setEnemyColor(int enemy_color) = 0;

            cv::Mat debug_binary_;
    private:

            virtual bool findLights(const cv::Mat & image, std::vector<base::LightBlob>& lights) = 0;

            virtual bool isLight(base::LightBlob light) = 0;

            virtual bool matchLights(std::vector<base::LightBlob>& lights,std::vector<base::Armor>& armors) = 0;

            virtual bool isArmor(base::LightBlob light_1,base::LightBlob light_2) = 0;



    };

} // namespace detector




#endif //RMOS_DETECTOR_INTERFACE_HPP
