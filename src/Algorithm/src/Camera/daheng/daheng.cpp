//
// Created by Wang on 23-6-14.
//

#include "Camera/daheng/daheng.hpp"

namespace camera
{
    DahengCam::DahengCam(const std::string camera_sn) : camera_sn_(camera_sn),
                                                        device_(nullptr),
                                                        pFrameBuffer_(nullptr)
    {
        // default param
        params_[CamParamType::Height] = 1200;
        params_[CamParamType::Width] = 1920;
        params_[CamParamType::AutoExposure] = 0;     // 自动曝光 ，0为否
        params_[CamParamType::Exposure] = 1000; // 曝光时间 单位 us
        params_[CamParamType::AutoWhiteBalance] = 0; //自动白平衡 ，0为否
        params_[CamParamType::RGain] = 19.8;           // 红通道增益
        params_[CamParamType::GGain] = 10;           // 绿通道增益
        params_[CamParamType::BGain] = 19.8;           // 蓝通道增益
        params_[CamParamType::Gamma] = 9;           // 伽马增益

        /*TO DO */
        params_[CamParamType::Fps] = 100;
        params_[CamParamType::Brightness] = 0;
        params_[CamParamType::WhiteBalance] = 0;
        params_[CamParamType::Gain] = 0;
        params_[CamParamType::Hue] = 0;
        params_[CamParamType::Contrast] = 0;
        params_[CamParamType::Saturation] = 0;
        this->is_open_ = false;
        this->error_message_ = "";
    }

    DahengCam::~DahengCam()
    {
        if (is_open_)
        {
            bool if_delete = this->close();
        }
    }

    bool DahengCam::is_open()
    {
        return is_open_;
    }

    bool DahengCam::open()
    {
        if (is_open_)
        {
            return true;
        }
        // init the DahengCam sdk
        if (!this->init_sdk())
        {
            return false;
        }
        // set the parameter
        if (!this->setInit())
        {
            return false;
        }
        rgbImagebuf_ = new uint8_t[params_[CamParamType::Height] * params_[CamParamType::Width] * 3];

        GX_STATUS status;
        status = GXStreamOn(device_);
        if (status != GX_STATUS_SUCCESS)
        {
            bool if_close = this->close();
            this->error_message_ = "DahengCam open failed ";
            return false;
        }
        this->is_open_ = true;
        return true;
    }

    bool DahengCam::close()
    {
        GX_STATUS status;
        status = GXStreamOff(device_);
        if (this->rgbImagebuf_ != NULL)
        {
            delete[] this->rgbImagebuf_;
            this->rgbImagebuf_ = NULL;
        }
        status = GXCloseDevice(device_);
        status = GXCloseLib();
        if (status != GX_STATUS_SUCCESS)
        {
            this->error_message_ = "DahengCam close failed ";
            return false;
        }
        this->is_open_ = false;
        return true;
    }

    bool DahengCam::grab_image(cv::Mat &image)
    {
        GX_STATUS status;
        // 采样
        status = GXDQBuf(device_, &pFrameBuffer_, 1000);
        if (status != GX_STATUS_SUCCESS)
        {
            this->error_message_ = "DahengCam get buffer failed ";
            return false;
        }
        //转换格式
        VxInt32 DXstatus = DxRaw8toRGB24(pFrameBuffer_->pImgBuf,
                                         rgbImagebuf_,
                                         pFrameBuffer_->nWidth,
                                         pFrameBuffer_->nHeight,
                                         RAW2RGB_NEIGHBOUR,
                                         DX_PIXEL_COLOR_FILTER(4), // DX_PIXEL_COLOR_FILTER(colorfilter_),
                                         false);
        if (DXstatus != DX_OK)
        {
            this->error_message_ = "DahengCam RawtoRGB24 failed ";
            return false;
        }
        auto speed_test_start_begin_time = std::chrono::steady_clock::now();
        image.create(pFrameBuffer_->nHeight, pFrameBuffer_->nWidth, CV_8UC3);
        memcpy(image.data, rgbImagebuf_, pFrameBuffer_->nHeight * pFrameBuffer_->nWidth * 3);
        // 重新回采
        status = GXQBuf(device_, pFrameBuffer_);
        if (status != GX_STATUS_SUCCESS)
        {
            this->error_message_ = "DahengCam get buffer failed ";
            return false;
        }
        auto cost = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count();
        return true;
    }

    bool DahengCam::set_parameter(CamParamType type, int value)
    {
        if (params_.find(type) != params_.end())
        {
            params_[type] = value;
            return true;
        }
        else
        {
            this->error_message_ = "DahengCam set_parameter failed";
            return false;
        }
    }

    bool DahengCam::reset_parameters(const std::map<CamParamType, int> &param_map)
    {
        this->close();
        for (auto it = param_map.begin(); it != param_map.end(); it++)
            this->set_parameter(it->first, it->second);
        return this->open();
    }

    bool DahengCam::get_parameter(CamParamType type, int &value)
    {
        if (params_.find(type) != params_.end())
        {
            value = params_[type];
            return true;
        }
        else
        {
            this->error_message_ = "DahengCam get_Parameter failed";
            return false;
        }
    }

    bool DahengCam::init_sdk()
    {
        GX_STATUS status;
        // init the lib of DahengCam
        status = GXInitLib();
        if (status != GX_STATUS_SUCCESS)
        {
            GXCloseLib();
            this->error_message_ = "DahengCam initsdk InitLib failed ";
            return false;
        }

        // get device lists
        uint32_t ui32DeviceNum = 0;
        status = GXUpdateDeviceList(&ui32DeviceNum, 1000);
        if (status != GX_STATUS_SUCCESS || ui32DeviceNum <= 0)
        {
            GXCloseLib();
            this->error_message_ = "DahengCam initsdk getDeviceList failed ";

            return false;
        }

        /*TODO*/
        // Open device by sn
        status = GXOpenDeviceByIndex(1, &device_); //  open the  device

        if (status != GX_STATUS_SUCCESS)
        {
            GXCloseLib();
            this->error_message_ = "DahengCam initsdk OpentDevice failed ";

            return false;
        }

        size_t nSize = 0;
        // get  vendor name
        GXGetStringLength(device_, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
        char *pszVendorName = new char[nSize];
        GXGetString(device_, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
        delete[] pszVendorName;
        pszVendorName = NULL;

        // get nodel_name
        GXGetStringLength(device_, GX_STRING_DEVICE_MODEL_NAME, &nSize);
        char *pszModelName = new char[nSize];
        GXGetString(device_, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
        delete[] pszModelName;
        pszModelName = NULL;

        // get  serial_number
        GXGetStringLength(device_, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
        char *pszSerialNumber = new char[nSize];
        GXGetString(device_, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
        delete[] pszSerialNumber;
        pszSerialNumber = NULL;

        // get version
        GXGetStringLength(device_, GX_STRING_DEVICE_VERSION, &nSize);
        char *pszDeviceVersion = new char[nSize];
        GXGetString(device_, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
        delete[] pszDeviceVersion;
        pszDeviceVersion = NULL;

        // set the model and the buffer num
        status = GXSetEnum(device_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
        status = GXSetEnum(device_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
        status = GXSetAcqusitionBufferNumber(device_, ACQ_BUFFER_NUM);

        return true;
    }

    bool DahengCam::setInit()
    {
        GX_STATUS status;
        status = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
        status = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
        status = GXSetInt(device_, GX_INT_WIDTH, params_[CamParamType::Width]);
        status = GXSetInt(device_, GX_INT_HEIGHT, params_[CamParamType::Height]);
        status = GXSetInt(device_, GX_INT_OFFSET_X, (1920 - params_[CamParamType::Width]) / 2);
        status = GXSetInt(device_, GX_INT_OFFSET_Y, (1200 - params_[CamParamType::Height]) / 2);

        // set Exposure
        if (params_[CamParamType::AutoExposure])
        {
            status = GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
            status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, 5000);
            status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, 20000);
        }
        else
        {
            status = GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
            status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, params_[CamParamType::Exposure] - 200);
            status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, params_[CamParamType::Exposure] + 200);
            status = GXSetFloat(device_, GX_FLOAT_EXPOSURE_TIME, params_[CamParamType::Exposure]);
        }

        // set AutoWhiteBalance Gain
        if (params_[CamParamType::AutoWhiteBalance])
        {
            status = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        }
        else
        {
            status = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
            status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
            status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, params_[CamParamType::RGain] / 10.0);
            status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
            status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, params_[CamParamType::GGain] / 10.0);
            status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
            status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, params_[CamParamType::BGain] / 10.0);
        }

        // Set Gamma
        status = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
        status = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MIN, 0);
        status = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MAX, 10);
        status = GXSetFloat(device_, GX_FLOAT_GAIN, params_[CamParamType::Gamma]);




        if (status != GX_STATUS_SUCCESS)
        {
            this->error_message_ = "DahengCam setInit failed ";
            return false;
        }
        return true;



    }
} // namespace camera