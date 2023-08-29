//
// Created by Wang on 23-6-14.
//

#ifndef RMOS_CONST_HPP
#define RMOS_CONST_HPP
namespace base
{
    enum Color
    {
        RED,
        BLUE,
    };

    enum ArmorType
    {
        WRONG,
        SMALL,
        BIG,
        RUNE_ARMOR,
        GRAY_BIG_ARMOR,
        GRAY_SMALL_ARMOR,
        DEAD_ARMOR,
    };

    enum TrackState
    {
        LOST,       // 丢失目标
        DETECTING,    // 丢失目标但在寻找目标
        TEMP_LOST,      // 丢失目标
        TRACKING,      // 持续识别目标
    };

    enum Mode{
        NORMAL,     // 普通模式
        NORMAL_RUNE, // 小符模式
        RUNE,       // 大符模式
        OUTPOST,    // 前哨站模式
    };


    enum DebugOption{
    CONTEST,
    SHOW_ARMOR,  
    SHOW_BIN,
    SHOW_DETECT_COST,
    SHOW_RVIZ,
    SHOW_TOTAL_COST,
    };

}
#endif //RMOS_CONST_HPP
