#ifndef __MY_CLOUD_H__
#define __MY_CLOUD_H__

#include <cti_msgs/GnssRTK.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include "cti_msgs/TabState.h"
#include <jsoncpp/json/json.h>
#include <cv_bridge/cv_bridge.h>         //cv_bridge
#include <sensor_msgs/image_encodings.h> //图像编码格式
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>
// #include "opencv2/opencv.hpp"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h" //转换函数头文件

#include "cti_msgs/BoxState.h"
#include "cti_msgs/TabState.h"
#include "std_msgs/Float32MultiArray.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "cti_rblite_msgs/BoxAskResponse.h"
#include "cti_rblite_msgs/BoxInfo.h"
#include "cti_rblite_msgs/NavDeliveryResp.h"
#include "cti_rblite_msgs/NavHeader.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "cti_vector_map_msgs/ClearAreaArray.h"
#include "cti_vector_map_msgs/RoadEdgeArray.h"
#include "cti_msgs/BatteryCellsState.h"
#include "cti_msgs/VehicleCtlRunInfo.h"
#include "cti_msgs/ErrorStatusArray.h"
#include "cti_msgs/ErrorStatus.h"
#include "cti_msgs/smoothPathService.h"
#include "cti_msgs/DataArray.h"
#include "cti_msgs/Data.h"
#include "sensor_msgs/PointCloud2.h"

#include <streambuf>
#include <boost/cast.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include "boost/shared_ptr.hpp"
#include <typeinfo>
#include <string>
#include <regex>
#include <thread>
#include <math.h>
#include <queue>
#include <set>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>

#include <boost/date_time.hpp>

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

// int aa;
// aa++;

struct DIARY_JSON
{

    std::string time;
    std::string message;
    std::string error_code;
};

enum UPLOAD_STATE
{
    UPLOAD_IDLE = 0,
    UPLOAD_START = 1,
    UPLOAD_SUCCESS = 2,
    UPLOAD_FAILED = 3
};

struct UpLoadManage
{
    bool enable{false};
    UPLOAD_STATE upLoadState;
    std::string localfile;
    std::string remotefile;
    std::vector<std::string> s_split(const std::string &in, const std::string &delim)
    {
        std::regex re{delim};
        return std::vector<std::string>{std::sregex_token_iterator(in.begin(), in.end(), re, -1), std::sregex_token_iterator()};
    }
    bool upload(const std::string file)
    {
        bool ret = false;
        if (access(file.c_str(), F_OK) == 0)
        {
            std::string delim = "/";
            std::vector<std::string> res = s_split(file, delim);
            remotefile = "/cti_data/" + res.back();
            localfile = file;
            enable = true;
            ret = true;
        }
        return ret;
    }
};

enum MAPSAMPLE_STATE
{
    MAPSAMPLE_IDLE = 0,
    MAPSAMPLE_START = 1,
    MAPSAMPLE_STOP = 2
};

struct MapSampleManage
{
    bool enable{false};
    int option_cnt{0};
    MAPSAMPLE_STATE MapSampleState;
    std::string startMappingCmd;
    std::string stopMappingCmd;
    std::string pathfile;
    std::string LogRealTime()
    {
        timespec tp;
        if (::clock_gettime(CLOCK_REALTIME_COARSE, &tp))
        {
            return "00000000-000000";
        }
        struct tm localctm;
#if !defined _WIN32 || !_WIN32
        struct tm *const chk = ::localtime_r(&tp.tv_sec, &localctm);
#else
        struct tm *const chk = ::localtime(&tp.tv_sec);
        if (!chk)
        {
            ::memset(&localctm, chk, sizeof(struct tm));
        }
#endif
        if (!chk)
        {
            ::memset(&localctm, 0, sizeof(struct tm));
        }
        if (0 == localctm.tm_mday)
        {
            localctm.tm_mday = 1;
        }
        //--
        int const tz = int(int64_t(localctm.tm_gmtoff / 3600.0));
        char buf[40];
        ::snprintf(buf, sizeof(buf), "%04d%02d%02d-%02d%02d%02d",
                   localctm.tm_year + 1900,
                   localctm.tm_mon + 1,
                   localctm.tm_mday,
                   localctm.tm_hour,
                   localctm.tm_min,
                   localctm.tm_sec);
        return buf;
    }

    bool startMapping()
    {
        bool ret = false;
        if (access(startMappingCmd.c_str(), F_OK) == 0)
        {
            MapSampleState = MAPSAMPLE_START;
            std::system(startMappingCmd.c_str());
            std::cout << "start mapping" << std::endl;
            ret = true;
        }
        return ret;
    }

    bool stopMapping()
    { //建图结束后 打包地图
        bool ret = false;
        if (access(stopMappingCmd.c_str(), F_OK) == 0)
        {
            MapSampleState = MAPSAMPLE_STOP;
            std::system(stopMappingCmd.c_str());
            std::string HOME(getenv("HOME"));
            std::string filename = "lslam-map-" + LogRealTime() + ".zip";
            std::string cmd_str = "cd ${HOME} && zip -r " + filename + " lslam-map";
            std::system(cmd_str.c_str());
            pathfile = HOME + "/" + filename;
            std::cout << "stop mapping" << std::endl;
            ret = true;
        }
        option_cnt = 0;
        return ret;
    }
};

struct MapSampleData
{
    double lon{0};
    double lat{0};
    double alt{0};
    float lat_err{0};
    float lon_err{0};
    float sats_used{0};
    //--
    int percent_value{0};
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
    void Check_Mapping_State();
    void sendWebDataState(bool trigger = false);
    void sendRoadedge();
    void sendCleararea();
    void pub_nav_version();
    void log_init(std::string log_path);

private:
    void init();
    void pubErrorMsg(const std::string &modular_name, const uint &error_code, const uint &error_level, const std::string &error_msg);
    void mapRecordStateCallback(const std_msgs::Int32 &msg);
    void gnnsLocationCallback(const cti_msgs::GnssRTKConstPtr &nav_satfix);
    void mappingStatusCallback(const std_msgs::Bool &msg);
    void sensorStateCallback(const std_msgs::Bool &msg);
    void rosWebCmdCallback(const std_msgs::Int32 &msg);
    void timercallback(const ros::TimerEvent &);
    void mapToJpgCallback(const nav_msgs::OccupancyGrid &msg);
    void rosweb_cmd_lidar(const std_msgs::Int32 &msg);
    void ndt_pose_Callback(const geometry_msgs::PoseStamped &msg);
    void points_data_Callback(const std_msgs::String &msg);
    void points_diary_Callback(const std_msgs::String &msg);
    void senfor_state_Callback(const cti_msgs::BoxState &msg);
    void save_web_diary(const std::string &msg, const std::string &msgT = "无", const std::string &msg_time = boost::lexical_cast<std::string>(boost::posix_time::second_clock::local_time()));
    void dynamic_save_web_diary(const cti_msgs::ErrorStatusArray &msg);
    void ult1_state_Callback(const std_msgs::Float32MultiArray &msg);
    void ult2_state_Callback(const std_msgs::Float32MultiArray &msg);
    void ult0_state_Callback(const std_msgs::Float32MultiArray &msg);
    void node_alive_Callback(const diagnostic_msgs::DiagnosticArray &msg);
    void box_msg_Callback(const cti_rblite_msgs::BoxAskResponse &msg);
    void NavDeliveryResp_Callback(const cti_rblite_msgs::NavDeliveryResp &msg);
    void send_web_diary(std::queue<DIARY_JSON> diary_data_down);
    void Mapping_statu_Callback(const std_msgs::Bool &msg);
    void Imu_statu_Callback(const sensor_msgs::Imu &msg);
    void roadedge_point_Callback(const cti_vector_map_msgs::RoadEdgeArray &msg);
    void cleararea_point_Callback(const cti_vector_map_msgs::ClearAreaArray &msg);
    void batcell_state_Callback(const cti_msgs::BatteryCellsState &msg);
    void dustboxbat_state_Callback(const cti_msgs::BatteryCellsState &msg);
    void camera1_Callback(const sensor_msgs::Image &msg);
    void error_msg_Callback(cti_msgs::ErrorStatusArray msg);
    void camera2_Callback(const sensor_msgs::Image &msg);
    bool ifsuccesschange(cti_msgs::smoothPathService::Request &re, cti_msgs::smoothPathService::Response &req);
    void pointCloud2ToZ_rslidar(const sensor_msgs::PointCloud2 &msg);
    void pointCloud2ToZ_rslidar_Bpearl(const sensor_msgs::PointCloud2 &msg);
    void pointCloud2ToZ_rslidar_lh_back_sacan(const sensor_msgs::PointCloud2 &msg);
    void pointCloud2ToZ_rslidar_livox(const sensor_msgs::PointCloud2 &msg);
    void pointCloud2ToZ_rslidar_mid(const sensor_msgs::PointCloud2 &msg);
    void web_red_err(const std_msgs::Bool &msg);
    void web_fix_frpc(const std_msgs::Bool &msg);
    void web_fix_version(const cti_msgs::DataArray &msg);
    void pub_web_red_err();
    void pub_webTimer();
    void err_monitor();
    void thread_web_lidar_calibration(const std::string &msg);
    void openAndsendtf_luanch();
    void check_tf_exit_luanch();

    void robot_env_sub(const std_msgs::String &msg);
    void upload_points_Date();

private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh_;
    ros::Subscriber status_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber sensorState_sub;
    ros::Subscriber webcmd_sub;
    ros::Subscriber webcmd_lidar_sub;
    ros::Subscriber mapToJpg_sub;
    ros::Subscriber ndt_pose_sub;
    ros::Subscriber points_data_sub;
    ros::Subscriber points_diary_sub;
    ros::Subscriber sensor_state_sub;
    ros::Subscriber ult1_state_sub;
    ros::Subscriber ult2_state_sub;
    ros::Subscriber ult0_state_sub;
    ros::Subscriber node_alive_sub;
    ros::Subscriber box_msg_sub;
    ros::Subscriber NavDeliveryResp_msg_sub;
    ros::Subscriber Mapping_statu_sub;
    ros::Subscriber Imu_statu_sub;
    ros::Subscriber roadedge_point_sub;
    ros::Subscriber cleararea_point_sub;
    ros::Subscriber batcell_state_sub;
    ros::Subscriber dustboxbat_state_sub;
    ros::Subscriber ctlrun_info_state_sub;
    ros::Subscriber camare1_sub;
    ros::Subscriber camare2_sub;
    ros::Subscriber error_msg_sub;
    ros::Subscriber rslidar_web_sub;
    ros::Subscriber rslidar_Bpearl_web_sub;
    ros::Subscriber rslidar_lh_back_web_sub;
    ros::Subscriber rslidar_behind_web_sub;
    ros::Subscriber rslidar_mid_web_sub;
    ros::Subscriber temp_use_rob_env_sub;
    ros::Subscriber web_red_err_sub;
    ros::Subscriber web_fix_version_sub;
    ros::Subscriber web_fix_frpc_sub;

    ros::ServiceServer ifchange_opoints_srv;

    ros::Publisher rslidar_mid_scan_pub;
    ros::Publisher rslidar_livox_pub;
    ros::Publisher web_send_calibration_tf_launch;
    ros::Publisher web_timer_pub;
    ros::Publisher rslidar_msg_pub;
    ros::Publisher rslidar_Bpearl_msg_pub;
    ros::Publisher rslidar_lh_back_pub;
    ros::Publisher camera2_cmd_pub;
    ros::Publisher change_line_pub;
    ros::Publisher nav_version_web_pub;
    ros::Publisher odometer_web_pub;
    ros::Publisher dustbox_battery_web_pub;
    ros::Publisher vihicle_battery_web_pub;
    ros::Publisher cleararea_point_web_pub;
    ros::Publisher roadedge_point_web_pub;
    ros::Publisher Imu_web_pub;
    ros::Publisher diary_web_pub;
    ros::Publisher NavDeliveryResp_web_pub;
    ros::Publisher box_msg_web_pub;
    ros::Publisher sensor_status_web_pub;
    ros::Publisher webstate_pub;
    ros::Publisher web_tf_pub;
    ros::Publisher web_tf_msg_pub;
    ros::Publisher mapToPng_encodebase64_pub;
    ros::Publisher camera_image_encodebase64_pub;
    ros::Publisher web_tf_person_view_pub;
    ros::Publisher error_publisher;
    ros::Publisher check_tf_file_publisher;
    ros::Publisher monitor_red_err_pub;
    ros::Publisher web_frpc_statu_pub;
    ros::Publisher web_version_status_pub;
    ros::Publisher web_fix_tip_pub;
    ros::Timer timer;
    tf::TransformListener tf_listen_; //监听者
    tf::TransformBroadcaster tf_broadcaster_vihicle_seconcd_person_view;
    tf::TransformListener tf_listen_vihicle_seconcd_person_view;

    int lidar_mode{-1};
    double Mappint_last_time{-1};
    bool flag_Mapping_start{false};
    bool camera1_flag{false};
    bool camera2_flag{false};

    struct ERR_RED_WEB
    {
        double web_red_err_start_time = ros::Time::now().toSec();
        bool first_flag{false};
        std_msgs::Bool web_red_err_flag;
    };
    ERR_RED_WEB ERR_RED_WEB;

    struct FIX_FILE
    {
        std_msgs::String warn_content;
    };
    FIX_FILE FIX_FILE;

    bool rslidar_web_switch{0};
    bool rslidar_Bpearl_web_switch{0};
    bool rslidar_back_scan_web_switch{0};
    bool rslidar_livox_web_switch{0};
    bool rslidar_mid_scan_switch{0};

    bool flag_ww{0};
    bool flag_first_in{1};
    bool err_end_flag{0};

    std::string robot_env;

    double err_monitor_timer_{0};

    std::string lidar_png_out_path;
    std::string shutdown_cmd_path;
    std::string check_pointdata_cmd_path;
    std::string CTI_SW_VER;

    std_msgs::String camera_cmd;

    cti_msgs::ErrorStatusArray cur_Err_code;
    cti_msgs::ErrorStatusArray tran_Err_code;
    cti_msgs::ErrorStatusArray relea_Err_code;
    cti_msgs::ErrorStatusArray rank_Err_code;

    MapSampleData data;
    UpLoadManage upload;
    MapSampleManage mapsample;
};

#endif
