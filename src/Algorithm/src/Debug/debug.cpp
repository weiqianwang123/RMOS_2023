
#include "Debug/debug.hpp"



namespace debug
{
       bool get_debug_option(int option)
    {
        try
        {
            cv::FileStorage fs("./src/Algorithm/configure/Debug/debug.xml", cv::FileStorage::READ);
            bool result;

            fs["contest"]>>result;
            if(result)
            {
                fs.release();
                return false;
            }
            
            switch (option)
            {
            case base::DebugOption::SHOW_ARMOR:
                fs["show_armor"]>>result;
                fs.release();
                return result;
            case base::DebugOption::SHOW_BIN:
                fs["show_bin"]>>result;
                fs.release();
                return result;
            case base::DebugOption::SHOW_DETECT_COST:
                fs["show_detect_cost"]>>result;
                fs.release();
                return result;
            case base::DebugOption::SHOW_RVIZ:
                fs["show_rviz"]>>result;
                fs.release();
                return result;
            case base::DebugOption::SHOW_TOTAL_COST:
                fs["show_total_cost"]>>result;
                fs.release();
                return result;
            
            default:
                return false;
            }
            return false;
        }
        catch(const cv::Exception &e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
    }
        
       
        

}