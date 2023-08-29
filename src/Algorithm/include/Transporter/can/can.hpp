//
// Created by nuc12 on 23-7-8.
//

#ifndef RMOS_CAN_HPP
#define RMOS_CAN_HPP


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <chrono>
#include <iostream>


#include "../../Base/target.hpp"
#include "../../Base/armor.hpp"

#define CAN_NAME "can0"

#define STATE_SEND_ID 0x106
#define DATA_SEND_ID 0x108

#define IMU_TIME_RECEIVE_ID 0x100    // 时间ID
#define IMU_RECEIVE_ID 0x0FF         // 四元数ID
#define MODE_RECEIVE_ID 0x110        // 模式ID
#define BS_RECEIVE_ID 0x125          // 弹速等级ID
#define HERO_BS_RECEIVE_ID 0x127     // 英雄弹速等级ID
#define ROBOT_RECEIVE_ID 0x129       // 机器人状态信息ID
#define COMPETITION_RECEVIE_ID 0x12B // 比赛信息ID


namespace transporter
{

    class Can
    {


    private:
        int socket_fd;
        struct sockaddr_can addr;
        struct ifreq interface_request;

    public:
        /**
         *  @brief  接收数据，根据id区分数据包，需要大于1000Hz频率接收
         *  @return error_code
         */
        int receive(uint &id, u_char *buf, u_char &dlc);

        /**
         * @brief   发送数据
         * @param   id  数据对应的ID
         * @param   buf 数据，长度小于等于8
         * @param   dlc 数据长度
         * @return  error_code
         */
        int send(uint id, u_char *buf, u_char dlc);

        Can();
        ~Can();

        enum CanError
        {
            SUCCESS,
            DLC_ERROR,
            WRITE_ERROR,
            READ_ERROR,
            TIME_ERROR
        };
    };


}








#endif //RMOS_CAN_HPP
