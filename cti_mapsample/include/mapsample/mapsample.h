#ifndef __MY_CLOUD_H__
#define __MY_CLOUD_H__

#include <cti_msgs/GnssRTK.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <boost/thread.hpp>
#include "boost/shared_ptr.hpp"
#include <string>
#include <regex>
#include <thread>

using namespace std;

#define APP_CMD_START 1
#define APP_CMD_STOP 2
#define APP_CMD_VIEW 3
#define APP_CMD_VIEWCLOSE 4

#define RES_STATE_FREE 0
#define RES_STATE_RECORD 1
#define RES_STATE_GPSERROR 2
#define RES_STATE_RECORD_WITHERROR_GPS 3
#define RES_STATE_SENSORFAIL 4

enum UPLOAD_STATE{
    UPLOAD_IDLE=0,
    UPLOAD_START=1,
    UPLOAD_SUCCESS=2,
    UPLOAD_FAILED=3
};

struct UpLoadManage
{
    bool enable{false};
    UPLOAD_STATE upLoadState;
    std::string localfile;
    std::string remotefile;
    std::vector<std::string> s_split(const std::string& in, const std::string& delim) {
        std::regex re{delim};
        return std::vector<std::string>{std::sregex_token_iterator(in.begin(), in.end(), re, -1),std::sregex_token_iterator()};
    }
    bool upload(const std::string file){
        bool ret = false;
        if(access(file.c_str(),F_OK) == 0)
        {
            std::string delim = "/";
            std::vector<std::string> res = s_split(file, delim);
            remotefile = "/cti_data/"+res.back();
            localfile = file;
            enable = true;
            ret = true;
        }
        return ret;
    }
};

enum MAPSAMPLE_STATE
{
    MAPSAMPLE_IDLE=0,
    MAPSAMPLE_START=1,
    MAPSAMPLE_STOP=2
};

struct MapSampleManage
{
    bool enable{false};
    int  option_cnt{0};
    MAPSAMPLE_STATE  MapSampleState;
    std::string startMappingCmd;
    std::string stopMappingCmd;
    std::string pathfile;
    std::string LogRealTime()
    {
        timespec tp;
        if (::clock_gettime(CLOCK_REALTIME_COARSE, &tp)) {
            return "00000000-000000";
        }
        struct tm localctm;
    #if !defined _WIN32 || !_WIN32
        struct tm* const chk = ::localtime_r(&tp.tv_sec, &localctm);
    #else
        struct tm* const chk = ::localtime(&tp.tv_sec);
        if (!chk) {
            ::memset(&localctm, chk, sizeof(struct tm));
        }
    #endif
        if (!chk) {
            ::memset(&localctm, 0, sizeof(struct tm));
        }
        if (0 == localctm.tm_mday) {
            localctm.tm_mday = 1;
        }
        //--
        int const tz = int(int64_t(localctm.tm_gmtoff / 3600.0));
        char buf[40];
        ::snprintf(buf, sizeof(buf),"%04d%02d%02d-%02d%02d%02d",
            localctm.tm_year + 1900,
            localctm.tm_mon + 1,
            localctm.tm_mday,
            localctm.tm_hour,
            localctm.tm_min,
            localctm.tm_sec
        );
        return buf;
    }

    bool startMapping() {
        bool ret = false;
        if(access(startMappingCmd.c_str(),F_OK) == 0)
        {
            MapSampleState = MAPSAMPLE_START;
            std::system(startMappingCmd.c_str());
            std::cout << "start mapping" << std::endl;
            ret = true;
        }
        return ret;
    }

    bool stopMapping() {
        bool ret = false;
        if(access(stopMappingCmd.c_str(),F_OK) == 0)
        {
            MapSampleState = MAPSAMPLE_STOP;
            std::system(stopMappingCmd.c_str());
            std::string HOME(getenv("HOME"));
            std::string filename = "lio-map-" + LogRealTime() + ".zip";
            std::string cmd_str = "cd ${HOME} && zip -r " + filename + " ${HOME}/lio-map";
            std::system(cmd_str.c_str());
            pathfile = HOME + "/" + filename;
            std::cout << "stop mapping" << std::endl;
            ret = true;
        }
        option_cnt = 0;
        return ret;
    }
};

struct MapSampleData {
    double lon{0};
    double lat{0};
    double alt{0};
    float  lat_err{0};
    float  lon_err{0};
    float  sats_used{0};
    //--
    int  percent_value{0};
    bool mapping_state{false};
    bool sensor_state{false};
    std::string message{""};
};

class MapSample
{
public:
    MapSample();
    ~MapSample();
    void run();
    void doWork();
    void upLoad();
    void sendWebDataState(bool trigger=false);
 private:
    void init();
    void mapRecordStateCallback(const std_msgs::Int32 &msg);
    void gnnsLocationCallback(const cti_msgs::GnssRTKConstPtr &nav_satfix);
    void mappingStatusCallback(const std_msgs::Bool &msg);
    void sensorStateCallback(const std_msgs::Bool &msg);
    void rosWebCmdCallback(const std_msgs::Int32 &msg);
    void timercallback(const ros::TimerEvent&);
 private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh_;
    ros::Subscriber status_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber sensorState_sub;
    ros::Subscriber webcmd_sub;
    ros::Publisher  webstate_pub;
    ros::Timer timer;

    MapSampleData data;
    UpLoadManage  upload;
    MapSampleManage mapsample;
};

#endif
