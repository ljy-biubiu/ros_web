#include "mapsample/mapsample.h"
#include "FTPClient.h"
#include "matToPng_base64.h"
#include <cmath>
#include "string.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "ctilog/log.hpp"
#include "ctilog/loghelper.cpp.hpp"
#include "ros/package.h"

using namespace embeddedmz;
using namespace cti::log;
constexpr char const *kN = "config";
const std::string home_str = std::string(std::getenv("HOME"));

MapSample::MapSample() : pnh_("~")
{
    init();
}

MapSample::~MapSample()
{
    if (ros::isStarted())
    {
        ros::shutdown();
    }
}

void MapSample::init()
{
    if (!ros::master::check())
    {
        return;
    }
    ros::start();
    //--

    std::string log_path_;
    if (pnh_.getParam("CTI_RUN_LOG_PATH", log_path_))
    {
        log_path_ += "/webtool.log";
    }
    else
    {
        log_path_ = home_str + "/log/webtool.log";
    }
    log_init(log_path_); //初始化系统日志

    std::string gpsTopic;
    std::string status_topic;
    std::string sensor_state_topic;
    std::string roswebCmd_topic;
    std::string roswebState_topic;
    std::string lidartoimage_topic;
    std::string mapToPng_encodebase64_topic;
    std::string rosweb_cmd_lidar;
    std::string web_tf_pub_topic;
    std::string pub_web_tf_msg_topic;
    std::string ndt_pose_topic;

    pnh_.param<std::string>("gpsTopic", gpsTopic, "/cti/chassis_serial/gnss");
    pnh_.param<std::string>("shutdown_nowCmd_path", shutdown_cmd_path, "");
    pnh_.param<std::string>("extract_Pointdata_Cmd_path", check_pointdata_cmd_path, "");
    pnh_.param<std::string>("startMappingCmd", mapsample.startMappingCmd, "");
    pnh_.param<std::string>("stopMappingCmd", mapsample.stopMappingCmd, "");
    pnh_.param<std::string>("mapping_status_topic", status_topic, "/mapping_status");
    pnh_.param<std::string>("sensor_state_topic", sensor_state_topic, "/cti/status_monitor/sensor_state");
    pnh_.param<std::string>("rosweb_cmd_topic", roswebCmd_topic, "/cti/rosweb/cmd");
    pnh_.param<std::string>("rosweb_state_topic", roswebState_topic, "/cti/rosweb/state");
    pnh_.param<std::string>("lidarTopic", lidartoimage_topic, "/cti/semantics/object_map/test_costmap");
    pnh_.param<std::string>("mapToPng_encodebase64_topic", mapToPng_encodebase64_topic, "/cti/cti_mapsample/mapToPng_encodebase64_topic");
    pnh_.param<std::string>("lidar_png_out_path", lidar_png_out_path, "/home/ljy/dev/catkin_wx/src/cti_data_viewer/cti_www/www/image");
    pnh_.param<std::string>("rosweb_cmd_lidar", rosweb_cmd_lidar, "/cti/rosweb/cmd_lidar");
    pnh_.param<std::string>("web_tf_pub_topic", web_tf_pub_topic, "/cti/rosweb/pub_web_tf");
    pnh_.param<std::string>("pub_web_tf_msg_topic", pub_web_tf_msg_topic, "/cti/rosweb/pub_web_tf_msg");
    pnh_.param<std::string>("ndt_pose_topic", ndt_pose_topic, "/ndt_pose_topic");
    nh.param<std::string>("CTI_SW_VER", CTI_SW_VER, "V3？");

    web_tf_msg_pub = nh.advertise<std_msgs::String>(pub_web_tf_msg_topic, 1);
    web_tf_pub = nh.advertise<std_msgs::String>(web_tf_pub_topic, 1);
    webstate_pub = nh.advertise<std_msgs::String>(roswebState_topic, 1);
    camera_image_encodebase64_pub = nh.advertise<std_msgs::String>("/cti/web/camera1", 1);
    mapToPng_encodebase64_pub = nh.advertise<std_msgs::String>(mapToPng_encodebase64_topic, 1);
    web_tf_person_view_pub = nh.advertise<geometry_msgs::PointStamped>("/cti/rosweb/pub_tf_person_view_msg", 1);
    sensor_status_web_pub = nh.advertise<std_msgs::String>("cti/robot_config/sensorState_web", 1, true);
    box_msg_web_pub = nh.advertise<std_msgs::String>("cti/web/box_msg", 1, true);
    NavDeliveryResp_web_pub = nh.advertise<std_msgs::String>("cti/web/NavDeliveryResp", 1, true);
    diary_web_pub = nh.advertise<std_msgs::String>("/cti/web/diary", 1, true);
    Imu_web_pub = nh.advertise<std_msgs::String>("/cti/web/imu", 1, true);
    roadedge_point_web_pub = nh.advertise<std_msgs::String>("/cti/web/roadedge_point", 1, true);
    cleararea_point_web_pub = nh.advertise<std_msgs::String>("/cti/web/cleararea_point", 1, true);
    vihicle_battery_web_pub = nh.advertise<std_msgs::String>("/cti/web/vihicle_battery", 1, true);
    dustbox_battery_web_pub = nh.advertise<std_msgs::String>("/cti/web/dustbox_battery", 1, true);
    odometer_web_pub = nh.advertise<std_msgs::String>("/cti/web/odometer", 1, true);
    nav_version_web_pub = nh.advertise<std_msgs::String>("/cti/web/nav_version", 1, true);
    change_line_pub = nh.advertise<std_msgs::String>("/cti/web/change_line", 1, true);
    camera2_cmd_pub = nh.advertise<std_msgs::String>("/box_assemble/calib_cmd", 1, true);
    rslidar_msg_pub = nh.advertise<sensor_msgs::PointCloud>("/cti/web/rslidar", 1, false);
    rslidar_Bpearl_msg_pub = nh.advertise<sensor_msgs::PointCloud>("/cti/web/rslidarr_Bpearl", 1, false);
    rslidar_lh_back_pub = nh.advertise<sensor_msgs::PointCloud>("/cti/web/rslidarr_lh_back", 1, false);
    rslidar_livox_pub = nh.advertise<sensor_msgs::PointCloud>("/cti/web/rslidar_livox", 1, false);
    rslidar_mid_scan_pub = nh.advertise<sensor_msgs::PointCloud>("/cti/web/rslidar_mid_scan", 1, false);
    error_publisher = nh.advertise<cti_msgs::ErrorStatus>("/cti/error_status/set", 20, true);
    check_tf_file_publisher = nh.advertise<std_msgs::Bool>("/cti/web/tf_exit", 1, true);
    monitor_red_err_pub = nh.advertise<std_msgs::Bool>("/cti/web/red_err_monitor", 1, true);
    web_frpc_statu_pub = nh.advertise<std_msgs::String>("/cti/web/web_frpc_statu", 1, true);
    web_version_status_pub = nh.advertise<cti_msgs::DataArray>("/cti/web/web_version_status", 1, false);
    web_fix_tip_pub = nh.advertise<std_msgs::String>("/cti/web/fix_file_tip", 1, false);
    web_timer_pub = nh.advertise<std_msgs::String>("/cti/web/timer", 1, true);
    web_send_calibration_tf_launch = nh.advertise<std_msgs::String>("/cti/web/tf_launch", 1, false);
    ifchange_opoints_srv = nh.advertiseService("/cti/srv/ifchangepoints", &MapSample::ifsuccesschange, this);
    temp_use_rob_env_sub = nh.subscribe("/cti/robot_env", 1, &MapSample::robot_env_sub, this);
    web_fix_frpc_sub = nh.subscribe("/cti/web/fix_frpc", 1, &MapSample::web_fix_frpc, this);
    web_fix_version_sub = nh.subscribe("/cti/web/fix_version", 1, &MapSample::web_fix_version, this);
    web_red_err_sub = nh.subscribe("/cti/web/red_error_show", 1, &MapSample::web_red_err, this);
    rslidar_mid_web_sub = nh.subscribe("/calibration/sensor/lhlidar/lh_middle_pointcloud2", 1, &MapSample::pointCloud2ToZ_rslidar_mid, this);
    rslidar_behind_web_sub = nh.subscribe("/calibration/livox/lidar", 1, &MapSample::pointCloud2ToZ_rslidar_livox, this);
    rslidar_lh_back_web_sub = nh.subscribe("/calibration/sensor/lslidar/back/lh_lslidar_pointcloud", 1, &MapSample::pointCloud2ToZ_rslidar_lh_back_sacan, this);
    rslidar_web_sub = nh.subscribe("/calibration/sensor/rslidar/PointCloud2", 1, &MapSample::pointCloud2ToZ_rslidar, this);
    rslidar_Bpearl_web_sub = nh.subscribe("/calibration/sensor/rslidar/Bpearl_PointCloud2", 1, &MapSample::pointCloud2ToZ_rslidar_Bpearl, this);
    error_msg_sub = nh.subscribe("/cti/error_status/get", 5, &MapSample::error_msg_Callback, this);
    camare2_sub = nh.subscribe("/box_assemble/processed_image", 1, &MapSample::camera2_Callback, this);
    camare1_sub = nh.subscribe("/trash_cam/image_raw", 1, &MapSample::camera1_Callback, this);
    dustboxbat_state_sub = nh.subscribe("/cti/chassis_serial/dustbox_batcell_state", 1, &MapSample::dustboxbat_state_Callback, this);
    batcell_state_sub = nh.subscribe("/cti/cti_fpga/batcell_state", 1, &MapSample::batcell_state_Callback, this);
    roadedge_point_sub = nh.subscribe("cti/vector_map_info/roadedge", 1, &MapSample::roadedge_point_Callback, this);
    cleararea_point_sub = nh.subscribe("cti/vector_map_info/cleararea", 1, &MapSample::cleararea_point_Callback, this);
    Imu_statu_sub = nh.subscribe("/cti/fpga_serial/imu", 1, &MapSample::Imu_statu_Callback, this);
    Mapping_statu_sub = nh.subscribe("/mapping_status", 1, &MapSample::Mapping_statu_Callback, this);
    NavDeliveryResp_msg_sub = nh.subscribe("rblite/navdeliveryresp", 1, &MapSample::NavDeliveryResp_Callback, this);
    box_msg_sub = nh.subscribe("/cloud_scheduling_node/response/ask/boxinfo", 1, &MapSample::box_msg_Callback, this);
    node_alive_sub = nh.subscribe("/cti/monitor/node_alive", 10, &MapSample::node_alive_Callback, this);
    ult1_state_sub = nh.subscribe("/cti/ultrasonic/data_1", 10, &MapSample::ult1_state_Callback, this);
    ult2_state_sub = nh.subscribe("/cti/ultrasonic/data_2", 10, &MapSample::ult2_state_Callback, this);
    ult0_state_sub = nh.subscribe("/cti/ultrasonic/data_0", 10, &MapSample::ult0_state_Callback, this);
    sensor_state_sub = nh.subscribe("/cti/robot_config/sensorState", 10, &MapSample::senfor_state_Callback, this);
    points_diary_sub = nh.subscribe("/cti/rosweb/submit_data_points_diary", 10, &MapSample::points_diary_Callback, this);
    points_data_sub = nh.subscribe("/cti/rosweb/submit_data_points", 10, &MapSample::points_data_Callback, this);
    webcmd_lidar_sub = nh.subscribe(rosweb_cmd_lidar, 10, &MapSample::rosweb_cmd_lidar, this);
    webcmd_sub = nh.subscribe(roswebCmd_topic, 10, &MapSample::rosWebCmdCallback, this);
    status_sub = nh.subscribe(status_topic, 10, &MapSample::mappingStatusCallback, this);
    gps_sub = nh.subscribe(gpsTopic, 10, &MapSample::gnnsLocationCallback, this);
    sensorState_sub = nh.subscribe(sensor_state_topic, 10, &MapSample::sensorStateCallback, this);
    mapToJpg_sub = nh.subscribe(lidartoimage_topic, 10, &MapSample::mapToJpgCallback, this);
    ndt_pose_sub = nh.subscribe(ndt_pose_topic, 10, &MapSample::ndt_pose_Callback, this);

    timer = nh.createTimer(ros::Duration(0.1), &MapSample::timercallback, this);

    ///////////////////////////////////////////////////////////////////////////检测修图工具的文件是否存在    ////////已弃用
    // std::cout<<"-------------------ckeckout------------"<<std::endl;ftp
    // std::string cmd_str = check_pointdata_cmd_path+"extact_dataINpoints.sh";
    // std::system(cmd_str.c_str());

    save_web_diary("------开机------");
}

void MapSample::run()
{
    std::thread dowork_thread(boost::bind(&MapSample::doWork, this));
    ros::spin();
    // std::thread new_thread( &MapSample::thread_web_lidar_calibration,this,"roslaunch calibration calibration_v5.launch" );	//子线程
    // new_thread.detach();
}

void MapSample::doWork()
{
    CFTPClient::FileInfo ResFileInfo = {0, 0.0};
    static double old_remote_file_size = 0.0f;
    double local_file_size = 0.0f;
    double remote_file_size = 0.0f;
    struct stat statbuf;
    bool bRes = false;
    bool flag = false;

    int waitcnt = 0;
    while (ros::ok())
    {
        //--delay--
        ros::Duration(0.1).sleep();
        if (upload.enable)
        {
            CFTPClient FTPClient([](const std::string &strLogMsg)
                                 { std::cout << strLogMsg << std::endl; });

            if (FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp001", "Ftp001@2020"))
            {
                upload.upLoadState = UPLOAD_STATE::UPLOAD_START;
                FTPClient.RemoveFile(upload.remotefile);
                bool upload_result = FTPClient.UploadFile(upload.localfile, upload.remotefile);
                std::cout << "uploading..." << std::endl;

                /////////////////////////////////////////////////////////////////////////////////////////////////////////
                if (stat(upload.localfile.c_str(), &statbuf) == 0 && flag == false)
                {
                    flag = true;
                    local_file_size = statbuf.st_size;
                    printf("local file size>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>%0.fKB\n", local_file_size);
                }
                while (!bRes)
                {
                    bRes = FTPClient.Info_(upload.remotefile, ResFileInfo);
                }
                remote_file_size = ResFileInfo.dFileSize;
                old_remote_file_size = remote_file_size;
                int upload_percent = remote_file_size * 100 / local_file_size;
                data.message = "当前上传进度为： " + boost::lexical_cast<std::string>(upload_percent) + "%";
                std::cout << data.message << std::endl;
                /////////////////////////////////////////////////////////////////////////////////////////////////////////
                if (upload_result)
                {
                    upload.upLoadState = UPLOAD_STATE::UPLOAD_SUCCESS;
                    std::cout << "success" << std::endl;
                }
                else
                {
                    upload.upLoadState = UPLOAD_STATE::UPLOAD_FAILED;
                    std::cout << "failed" << std::endl;
                }
                FTPClient.CleanupSession();
                upload.enable = false;
                data.message = "数据上传完成";
            }
            else
            {
                waitcnt++;
            }
            if (waitcnt > 20)
            {
                upload.upLoadState = UPLOAD_FAILED;
                upload.enable = false;
                std::cout << "upload failed" << std::endl;
            }
            sendWebDataState(true);
        }
        else
        {
            waitcnt = 0;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////imu数据处理玉发布

////////////////////////////////////////////////////////////////////////////////////////////////线路发布   RoadEdgeArray

void MapSample::log_init(std::string log_path)
{
    Logger::setDefaultLogger(log_path);
    Logger::getLogger().setOutputs(Logger::Output::Both);
    Logger::getLogger().setLogLevel(LogLevel::Debu);
    Logger::getLogger().enableTid(false);
    Logger::getLogger().enableIdx(true);
}

void MapSample::web_fix_version(const cti_msgs::DataArray &msg)
{
    static std::vector<std::string> version_data_sub;
    fstream fio_version;
    std::string read_version_path{"/home/neousys/cti-launch/boot_sh/version"};
    if (msg.datas[0].name == "switch_on")
    {

        ///////////////////////////////////////////////////////////////////////////////// extact port from version

        fio_version.open(read_version_path);
        if (!fio_version.is_open())
        {
            std::cout << "打开" + read_version_path + "失败" << std::endl;
            FIX_FILE.warn_content.data = "打开" + read_version_path + "失败";
            web_fix_tip_pub.publish(FIX_FILE.warn_content);
            return;
        }

        std::string version_data{""};
        std::string version_data_port{""};
        cti_msgs::Data tmp_data;
        cti_msgs::DataArray version_array;

        ////////////////////////////////////////////////////////////////////////////////////// divide datas by :
        getline(fio_version, version_data);
        Info("修改前的version：" << version_data);
        char *str = (char *)version_data.c_str(); // string --> char
        const char *split = ":";
        char *p = strtok(str, split); //:分隔依次取出


        while (p != NULL)
        {
            version_data_sub.push_back(p);
            p = strtok(NULL, split);
        }

        tmp_data.name = "hw_version";
        tmp_data.data = version_data_sub[0];
        version_array.datas.push_back(tmp_data);

        tmp_data.name = "garden_name";
        tmp_data.data = version_data_sub[1];
        version_array.datas.push_back(tmp_data);

        tmp_data.name = "hw_version_numb";
        tmp_data.data = version_data_sub[2];
        version_array.datas.push_back(tmp_data);

        tmp_data.name = "vehicle_numb";
        tmp_data.data = version_data_sub[3];
        version_array.datas.push_back(tmp_data);

        web_version_status_pub.publish(version_array);
        return;
    }

    for (auto data_ : msg.datas)
    {
        int a = 0;
        if (data_.name == "hw_version")
        {
            version_data_sub[0] = data_.data;
        }
        if (data_.name == "garden_name")
        {
            version_data_sub[1] = data_.data;
        }
        if (data_.name == "hw_version_numb")
        {
            version_data_sub[2] = data_.data;
        }
        if (data_.name == "vehicle_numb")
        {
            version_data_sub[3] = data_.data;
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////// write down the port in .ini
    fio_version.open(read_version_path, ios::out);
    if (!fio_version.is_open())
    {
        std::cout << "打开" + read_version_path + "失败" << std::endl;
        FIX_FILE.warn_content.data = "打开" + read_version_path + "失败";
        web_fix_tip_pub.publish(FIX_FILE.warn_content);
        return;
    }

    std::string combine_version{""};
    combine_version = version_data_sub[0] + ":" + version_data_sub[1] + ":" + version_data_sub[2] + ":" + version_data_sub[3];
    std::cout << combine_version << std::endl;
    fio_version << combine_version;
    fio_version.close();

    /////////////////////////////////////////////////////////////////////////////////////////////// check the version

    fio_version.open(read_version_path);
    if (!fio_version.is_open())
    {
        std::cout << "打开" + read_version_path + "失败" << std::endl;
        FIX_FILE.warn_content.data = "打开" + read_version_path + "失败";
        web_fix_tip_pub.publish(FIX_FILE.warn_content);
        return;
    }
    //////////////////////////////////////////////////////////////////////////////////////////// put check data to web
    std::string version_data{""};
    getline(fio_version, version_data);
    FIX_FILE.warn_content.data = "修改后的version：\n" + version_data;
    web_fix_tip_pub.publish(FIX_FILE.warn_content);
    Info("修改后的version：" << version_data);

    fio_version.close();
}

void MapSample::web_fix_frpc(const std_msgs::Bool &msg)
{
    //覆盖frpc版本
    fstream fout_frpc;
    fstream fin_version;

    std::string read_frpc_path{"/etc/cti-frpc.ini"};
    std::string read_version_path{"/home/neousys/cti-launch/boot_sh/version"};

    ///////////////////////////////////////////////////////////////////////////////// extact port from version

    fin_version.open(read_version_path);
    if (!fin_version.is_open())
    {
        std::cout << "打开" + read_version_path + "失败" << std::endl;
        FIX_FILE.warn_content.data = "打开" + read_version_path + "失败";
        web_fix_tip_pub.publish(FIX_FILE.warn_content);
        return;
    }

    std::string version_data{""};
    std::vector<std::string> version_data_sub;
    std::string version_data_port{""};

    ////////////////////////////////////////////////////////////////////////////////////// divide datas by :
    getline(fin_version, version_data);
    char *str = (char *)version_data.c_str(); // string --> char
    const char *split = ":";
    char *p = strtok(str, split); //:分隔依次取出

    while (p != NULL)
    {
        version_data_sub.push_back(p);
        p = strtok(NULL, split);
    }

    //////////////////////////////////////////////////////////////////////////////////////// exstact numbers form tmp_data
    char *str_ = (char *)version_data_sub[3].c_str(); // string --> char

    while (*str_ != NULL)
    {
        if (*str_ == 'A')
        {
            str_++;
            continue;
        }
        else
        {
            version_data_port = version_data_port + *str_;
            str_++;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////// extract datas from .ini
    fout_frpc.open(read_frpc_path, ios::out | ios::in);
    if (!fout_frpc.is_open())
    {
        std::cout << "打开" + read_frpc_path + "失败" << std::endl;
        FIX_FILE.warn_content.data = "打开" + read_frpc_path + "失败";
        web_fix_tip_pub.publish(FIX_FILE.warn_content);
        return;
    }

    std::vector<std::string> frpc_data;
    std::string frpc_tmp;

    while (getline(fout_frpc, frpc_tmp))
    {
        frpc_data.push_back(frpc_tmp);
    }
    fout_frpc.close();

    ////////////////////////////////////////////////////////////////////////////////////////////// pub msg to web
    std_msgs::String msg_frpc;
    msg_frpc.data = "当前端口号为：" + frpc_data[4] + "\n" + "修复后端口号为：remote_port = 2" + version_data_port;
    Info(msg_frpc.data);
    web_frpc_statu_pub.publish(msg_frpc);
    std::cout << "======================================================" << std::endl;
    //////////////////////////////////////////////////////////////////////////////////////////// write down the port in .ini
    fout_frpc.open(read_frpc_path, ios::out | ios::in);

    frpc_data[4] = "remote_port = 2" + version_data_port;
    for (int i = 0; frpc_data.size() > i; i++)
    {
        fout_frpc << frpc_data[i] << std::endl;
    }
    fout_frpc.close();

    FIX_FILE.warn_content.data = "当前端口号为：2" + frpc_data[4];
    web_fix_tip_pub.publish(FIX_FILE.warn_content);
}

void MapSample::web_red_err(const std_msgs::Bool &msg)
{
    if (msg.data == true)
    {
        ERR_RED_WEB.web_red_err_start_time = ros::Time::now().toSec();
        ERR_RED_WEB.first_flag = true;
    }
}

void MapSample::pub_web_red_err()
{
    if (ros::Time::now().toSec() - ERR_RED_WEB.web_red_err_start_time < 2 && ERR_RED_WEB.first_flag == true)
    {
        ERR_RED_WEB.web_red_err_flag.data = true;
        monitor_red_err_pub.publish(ERR_RED_WEB.web_red_err_flag);
    }
    else
    {
        ERR_RED_WEB.web_red_err_flag.data = false;
        monitor_red_err_pub.publish(ERR_RED_WEB.web_red_err_flag);
    }
}

void MapSample::robot_env_sub(const std_msgs::String &msg)
{
    robot_env = msg.data;
}

void MapSample::pubErrorMsg(const std::string &modular_name, const uint &error_code, const uint &error_level, const std::string &error_msg)
{
    cti_msgs::ErrorStatus error_msgs;
    error_msgs.stamp = ros::Time::now();
    error_msgs.module_name = modular_name;
    error_msgs.error_code = error_code;
    error_msgs.error_info = error_msg;
    error_msgs.level = error_level;
    error_publisher.publish(error_msgs);
}

/////////////////////////////////////////暂时用
void MapSample::upload_points_Date()
{

    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });

    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");
    FTPClient.RemoveFile("/test/tf.launch.tar.gz");
    std::string file_path = "/home/neousys/cti_vmap";

    std::string cmd_str = "cd " + file_path + " && tar -zcvf " + robot_env + ".tar.gz " + robot_env;
    std::system(cmd_str.c_str());

    bool upload_result = FTPClient.UploadFile(file_path + "/" + robot_env + ".tar.gz", "/test/" + robot_env + ".tar.gz"); /////////文件 文件
    std::cout << "uploading..." << std::endl;
    if (upload_result)
    {
        std::cout << "success" << std::endl;
    }
    else
    {
        std::cout << "failed" << std::endl;
    }
    FTPClient.CleanupSession();
}

void MapSample::openAndsendtf_luanch()
{
    ifstream ifs;
    std::string read_file_path;
    std_msgs::String pub_data;
    read_file_path = "/home/neousys/cti-launch/launch/tf.launch";
    std::cout << "Read file path: " << read_file_path << std::endl;
    ifs.open(read_file_path);
    if (!ifs.is_open())
    {
        pub_data.data = " 问题巨大，没有找到tf.launch文件 ";
        web_send_calibration_tf_launch.publish(pub_data);
        std::cout << " 问题巨大，没有找到tf.launch文件 " << std::endl;
        return;
    }

    std::string str((std::istreambuf_iterator<char>(ifs)),
                    std::istreambuf_iterator<char>());
    pub_data.data = str;
    web_send_calibration_tf_launch.publish(pub_data);

    ifs.close();
}

void MapSample::check_tf_exit_luanch()
{
    static int count = 100;
    if (count == 100)
    {
        count--;
        const std::string home_str = std::string(std::getenv("HOME"));
        ifstream ifs;
        std::string read_file_path;
        std_msgs::Bool pub_data;
        read_file_path = home_str + "/cti-launch/launch/tf.launch";
        ifs.open(read_file_path);
        if (!ifs.is_open())
        {
            pub_data.data = 0;
            check_tf_file_publisher.publish(pub_data);
            ifs.close();
            return;
        }
        pub_data.data = 1;
        check_tf_file_publisher.publish(pub_data);
        ifs.close();
        return;
    }
    count--;
    if (count == 0)
    {
        count = 100;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////rslidar calibration
void MapSample::pointCloud2ToZ_rslidar(const sensor_msgs::PointCloud2 &msg)
{
    //static int numb{0};
    //static int timer{0};

    if (rslidar_web_switch == 1)
    {
        //numb ++;
        // if(numb == 1)
        // {
        sensor_msgs::PointCloud out_pointcloud;
        sensor_msgs::PointCloud out_pointcloud_aft;
        sensor_msgs::PointCloud out_pointcloud_last;
        geometry_msgs::Point32 point_;
        sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud);
        for (int i = 0; i < out_pointcloud.points.size(); i++)
        {
            if ((out_pointcloud.points[i].x < 10 && out_pointcloud.points[i].x > -10) && (out_pointcloud.points[i].y < 10 && out_pointcloud.points[i].y > -10) && (out_pointcloud.points[i].z < 10 && out_pointcloud.points[i].z > -10))
            {
                point_.x = setPercision(out_pointcloud.points[i].x);
                point_.y = setPercision(out_pointcloud.points[i].y);
                point_.z = setPercision(out_pointcloud.points[i].z);
                out_pointcloud_aft.points.push_back(point_);
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        cloud2->width = out_pointcloud_aft.points.size();
        cloud2->height = 1;
        cloud2->is_dense = false;
        cloud2->resize(cloud2->width * cloud2->height);

        int i_{0};
        // //遍历点云输出坐标
        for (auto &point : cloud2->points)
        {
            point.x = out_pointcloud_aft.points[i_].x;
            point.y = out_pointcloud_aft.points[i_].y;
            point.z = out_pointcloud_aft.points[i_].z;
            i_++;
        }

        // for (int i=0; i<cloud2->points.size(); i++)
        // {
        //     std::cout<<"X输出："<<cloud2->points[i].x<<std::endl;
        //     std::cout<<"Y输出："<<cloud2->points[i].y<<std::endl;
        //     std::cout<<"Z输出："<<cloud2->points[i].z<<std::endl;
        // }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud2);
        voxel.setLeafSize(0.2f, 0.2f, 0.2f); //体素大小设置为30*30*30cm
        voxel.filter(*cloud_voxel_filtered);

        for (auto point : cloud_voxel_filtered->points)
        {
            point_.x = point.x;
            point_.y = point.y;
            point_.z = point.z;
            out_pointcloud_last.points.push_back(point_);
            //std::cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<std::endl;
        }
        rslidar_msg_pub.publish(out_pointcloud_aft);

        // for (int i=0; i<out_pointcloud_aft.points.size(); i++)
        // {
        //     std::cout<<"X输出："<<out_pointcloud_aft.points[i].x<<std::endl;
        //     std::cout<<"Y输出："<<out_pointcloud_aft.points[i].y<<std::endl;
        //     std::cout<<"Z输出："<<out_pointcloud_aft.points[i].z<<std::endl;
        // }

        // std::cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<std::endl;
        // std::cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<std::endl;
        std::cout << "ONE" << out_pointcloud_aft.points.size() << std::endl;
        std::cout << "TWO" << out_pointcloud.points.size() << std::endl;
        std::cout << "Three" << out_pointcloud_last.points.size() << std::endl;
        //}

        // if( numb == 3)
        // {
        //     numb = 0;
        // timer++;
        // if(timer == 180)
        // {
        //     rslidar_web_switch = 0;
        // }
        //}
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////rslidar calibration

void MapSample::pointCloud2ToZ_rslidar_mid(const sensor_msgs::PointCloud2 &msg)
{
    if (rslidar_mid_scan_switch == 1)
    {
        sensor_msgs::PointCloud out_pointcloud;
        sensor_msgs::PointCloud out_pointcloud_aft;
        geometry_msgs::Point32 point_;
        sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud);
        for (int i = 0; i < out_pointcloud.points.size(); i++)
        {
            if ((out_pointcloud.points[i].x < 10 && out_pointcloud.points[i].x > -10) && (out_pointcloud.points[i].y < 10 && out_pointcloud.points[i].y > -10) && (out_pointcloud.points[i].z < 10 && out_pointcloud.points[i].z > -10))
            {
                point_.x = setPercision(out_pointcloud.points[i].x);
                point_.y = setPercision(out_pointcloud.points[i].y);
                point_.z = setPercision(out_pointcloud.points[i].z);
                out_pointcloud_aft.points.push_back(point_);
            }
        }
        rslidar_mid_scan_pub.publish(out_pointcloud_aft);

        std::cout << "ONE" << out_pointcloud_aft.points.size() << std::endl;
        std::cout << "TWO" << out_pointcloud.points.size() << std::endl;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////rslidar calibration
void MapSample::pointCloud2ToZ_rslidar_Bpearl(const sensor_msgs::PointCloud2 &msg)
{
    // static int numb{0};
    // static int timer{0};

    if (rslidar_Bpearl_web_switch == 1)
    {
        std::cout << "-------------------------------dsfsd-----------------------" << std::endl;
        //numb ++;
        // if(numb == 1)
        // {
        sensor_msgs::PointCloud out_pointcloud;
        sensor_msgs::PointCloud out_pointcloud_aft;
        sensor_msgs::PointCloud out_pointcloud_last;
        geometry_msgs::Point32 point_;
        sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud);
        for (int i = 0; i < out_pointcloud.points.size(); i++)
        {
            if ((out_pointcloud.points[i].x < 10 && out_pointcloud.points[i].x > -10) && (out_pointcloud.points[i].y < 10 && out_pointcloud.points[i].y > -10) && (out_pointcloud.points[i].z < 10 && out_pointcloud.points[i].z > -10))
            {
                point_.x = setPercision(out_pointcloud.points[i].x);
                point_.y = setPercision(out_pointcloud.points[i].y);
                point_.z = setPercision(out_pointcloud.points[i].z);
                out_pointcloud_aft.points.push_back(point_);
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        cloud2->width = out_pointcloud_aft.points.size();
        cloud2->height = 1;
        cloud2->is_dense = false;
        cloud2->resize(cloud2->width * cloud2->height);

        int i_{0};
        // //遍历点云输出坐标
        for (auto &point : cloud2->points)
        {
            point.x = out_pointcloud_aft.points[i_].x;
            point.y = out_pointcloud_aft.points[i_].y;
            point.z = out_pointcloud_aft.points[i_].z;
            i_++;
        }

        // for (int i=0; i<cloud2->points.size(); i++)
        // {
        //     std::cout<<"X输出："<<cloud2->points[i].x<<std::endl;
        //     std::cout<<"Y输出："<<cloud2->points[i].y<<std::endl;
        //     std::cout<<"Z输出："<<cloud2->points[i].z<<std::endl;
        // }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud2);
        voxel.setLeafSize(0.2f, 0.2f, 0.2f); //体素大小设置为30*30*30cm
        voxel.filter(*cloud_voxel_filtered);

        for (auto point : cloud_voxel_filtered->points)
        {
            point_.x = point.x;
            point_.y = point.y;
            point_.z = point.z;
            out_pointcloud_last.points.push_back(point_);
            //std::cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<std::endl;
        }
        rslidar_Bpearl_msg_pub.publish(out_pointcloud_aft);

        // for (int i=0; i<out_pointcloud_last.points.size(); i++)
        // {
        //     std::cout<<"X输出："<<out_pointcloud_last.points[i].x<<std::endl;
        //     std::cout<<"Y输出："<<out_pointcloud_last.points[i].y<<std::endl;
        //     std::cout<<"Z输出："<<out_pointcloud_last.points[i].z<<std::endl;
        // }

        // std::cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<std::endl;
        std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
        // std::cout<<"ONE"<<out_pointcloud_aft.points.size()<<std::endl;
        // std::cout<<"TWO"<<out_pointcloud.points.size()<<std::endl;
        std::cout << "Three" << out_pointcloud_last.points.size() << std::endl;
        //}

        // if( numb == 3 )
        // {
        //     numb = 0;
        //     timer++;
        //     if(timer == 180 )
        //     {
        //         rslidar_Bpearl_web_switch = 0;
        //     }
        // }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////rslidar calibration

void MapSample::pointCloud2ToZ_rslidar_livox(const sensor_msgs::PointCloud2 &msg)
{
    if (rslidar_livox_web_switch == 1)
    {
        sensor_msgs::PointCloud out_pointcloud;
        sensor_msgs::PointCloud out_pointcloud_aft;

        geometry_msgs::Point32 point_;
        sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud);
        for (int i = 0; i < out_pointcloud.points.size(); i++)
        {
            if ((out_pointcloud.points[i].x < 10 && out_pointcloud.points[i].x > -10) && (out_pointcloud.points[i].y < 10 && out_pointcloud.points[i].y > -10) && (out_pointcloud.points[i].z < 10 && out_pointcloud.points[i].z > -10))
            {
                point_.x = setPercision(out_pointcloud.points[i].x);
                point_.y = setPercision(out_pointcloud.points[i].y);
                point_.z = setPercision(out_pointcloud.points[i].z);
                out_pointcloud_aft.points.push_back(point_);
            }
        }
        rslidar_livox_pub.publish(out_pointcloud_aft);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////rslidar calibration
void MapSample::pointCloud2ToZ_rslidar_lh_back_sacan(const sensor_msgs::PointCloud2 &msg)
{
    //static int numb{0};
    //static int timer{0};

    if (rslidar_back_scan_web_switch == 1)
    {
        // numb ++;
        // if(numb == 1)
        // {
        sensor_msgs::PointCloud out_pointcloud;
        sensor_msgs::PointCloud out_pointcloud_aft;

        geometry_msgs::Point32 point_;
        sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud);
        for (int i = 0; i < out_pointcloud.points.size(); i++)
        {
            if ((out_pointcloud.points[i].x < 10 && out_pointcloud.points[i].x > -10) && (out_pointcloud.points[i].y < 10 && out_pointcloud.points[i].y > -10) && (out_pointcloud.points[i].z < 10 && out_pointcloud.points[i].z > -10))
            {
                point_.x = setPercision(out_pointcloud.points[i].x);
                point_.y = setPercision(out_pointcloud.points[i].y);
                point_.z = setPercision(out_pointcloud.points[i].z);
                out_pointcloud_aft.points.push_back(point_);
            }
        }
        rslidar_lh_back_pub.publish(out_pointcloud_aft);
        //}
        // if( numb == 3)
        // {
        //     numb = 0;
        //     timer++;
        //     if(timer == 180)
        //     {
        //         rslidar_back_scan_web_switch = 0;
        //     }
        // }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////rslidar calibration

bool MapSample::ifsuccesschange(cti_msgs::smoothPathService::Request &re, cti_msgs::smoothPathService::Response &req)
{
    req.id = 123;
    std_msgs::String msg;
    change_line_pub.publish(msg);
    upload_points_Date();
}

///////////////////////////////////////////////////////////////////////

const std::string merge_nameAndmodule(const cti_msgs::ErrorStatus &msg)
{
    std::string tranlation_error;
    if (msg.error_code < 10)
    {
        tranlation_error = msg.module_name + "000" + to_string(msg.error_code) + "E";
    }
    else if (msg.error_code < 100)
    {
        tranlation_error = msg.module_name + "00" + to_string(msg.error_code) + "E";
    }
    else
    {
        tranlation_error = msg.module_name + "0" + to_string(msg.error_code) + "E";
    }
    return tranlation_error;
}

std::queue<DIARY_JSON> readFileJson()
{
    Json::Reader reader;
    Json::Value root;
    ifstream ifs;
    std::queue<DIARY_JSON> diary_data_down;
    std::string read_file_path;
    read_file_path = ros::package::getPath("ctiwww") + "/www/js/diary/demo.json";
    std::cout << "Read file path: " << read_file_path << std::endl;
    ifs.open(read_file_path);
    // if (!ifs.is_open())
    // {
    //     ifs.open("/opt/cti/kinetic/share/ctiwww/www/js/diary/demo.json");
    // }
    if (!reader.parse(ifs, root))
    {
        std::cout << "读取json失败" << std::endl;
        return diary_data_down;
    }
    ifs.close();

    DIARY_JSON diary_json_down;
    int my_size = root.size();
    for (int a = 0; my_size > a; a++)
    {
        diary_json_down.time = root[a]["time"].asString();
        diary_json_down.message = root[a]["message"].asString();
        diary_json_down.error_code = root[a]["error_code"].asString();
        diary_data_down.push(diary_json_down);
        //std::cout<<root[a]["error_code"].asInt()<<std::endl;
    }

    return diary_data_down;
}

void writeFileJson(std::queue<DIARY_JSON> &diary_data_down, const int &record_numb)
{
    //子节点
    Json::Value root;
    int myqueue_size = diary_data_down.size();

    if (myqueue_size > record_numb)
    {
        diary_data_down.pop();
        myqueue_size--;
    }

    for (int i = 0; i < myqueue_size; i++)
    {
        Json::Value friends;
        friends["time"] = Json::Value(diary_data_down.front().time);
        friends["message"] = Json::Value(diary_data_down.front().message);
        friends["error_code"] = Json::Value(diary_data_down.front().error_code);
        diary_data_down.push(diary_data_down.front());
        diary_data_down.pop();
        root.append(friends);
        //std::cout<<diary_data_down.front().error_code<<std::endl;
    }

    //缩进输出
    Json::StyledWriter sw;
    //输出到文件
    ofstream os;
    std::string write_file_path;
    write_file_path = ros::package::getPath("ctiwww") + "/www/js/diary/demo.json";
    std::cout << "Write file path: " << write_file_path << std::endl;
    os.open(write_file_path);
    // if (!os.is_open())
    // {
    //     os.open("/opt/cti/kinetic/share/ctiwww/www/js/diary/demo.json");
    // }
    os << sw.write(root);
    os.close();
}

void MapSample::send_web_diary(std::queue<DIARY_JSON> diary_data_down)
{
    Json::Value root;
    Json::FastWriter writer;
    std_msgs::String web_msg;
    int myqueue_size_sum = diary_data_down.size();
    int myqueue_size = diary_data_down.size();
    if (myqueue_size > 50)
    {
        myqueue_size = 50;
    }

    for (int i = 50; i < myqueue_size_sum; myqueue_size_sum--)
    {
        diary_data_down.pop();
    }
    for (int i = 0; i < myqueue_size; i++)
    {
        Json::Value friends;
        friends["time"] = Json::Value(diary_data_down.front().time);
        friends["message"] = Json::Value(diary_data_down.front().message);
        friends["error_code"] = Json::Value(diary_data_down.front().error_code);
        diary_data_down.pop();
        root.append(friends);
    }

    web_msg.data = writer.write(root);
    diary_web_pub.publish(web_msg);
}

void MapSample::save_web_diary(const std::string &msg, const std::string &msgT, const std::string &msg_time)
{
    //std::string time = boost::lexical_cast<std::string>(boost::posix_time::second_clock::local_time());
    std::queue<DIARY_JSON> diary_data;
    DIARY_JSON diary_json;
    diary_json.message = msg;
    diary_json.time = msg_time;
    diary_json.error_code = msgT;
    int record_numb{600};
    diary_data = readFileJson();
    diary_data.push(diary_json);
    writeFileJson(diary_data, record_numb);
    send_web_diary(diary_data);
}

void MapSample::dynamic_save_web_diary(const cti_msgs::ErrorStatusArray &msg)
{
    std::string time = boost::lexical_cast<std::string>(boost::posix_time::second_clock::local_time());
    std::queue<DIARY_JSON> diary_data;
    DIARY_JSON diary_json;
    diary_data = readFileJson();
    for (auto msg_err_code : msg.data)
    {
        diary_json.message = "信息：" + msg_err_code.error_info + "      优先级：" + to_string(msg_err_code.level);
        diary_json.time = time;
        diary_json.error_code = merge_nameAndmodule(msg_err_code);
        diary_data.push(diary_json);
    }
    send_web_diary(diary_data);
}

void MapSample::error_msg_Callback(cti_msgs::ErrorStatusArray msg) //错误码收发
{
    tran_Err_code.data.clear();             //第一次清空处理空间
    tran_Err_code.data = cur_Err_code.data; //将现存数据放入处理空间中

    for (auto err_msg : msg.data) // 检测是否有新的数据传入，有则放入cur_Err_code
    {
        bool exsit_flag{false};
        for (auto tran_err_code : tran_Err_code.data)
        {
            if (tran_err_code.module_name == err_msg.module_name && tran_err_code.error_code == err_msg.error_code && tran_err_code.error_info == err_msg.error_info)
            {
                exsit_flag = true;
            }
        }
        if (exsit_flag == false)
        {
            err_msg.status_info = boost::lexical_cast<std::string>(boost::posix_time::second_clock::local_time());
            cur_Err_code.data.push_back(err_msg);
        }
    }

    tran_Err_code.data.clear();             //第二次清空处理空间
    tran_Err_code.data = cur_Err_code.data; //将现存数据放入处理空间中
    cur_Err_code.data.clear();              //清除当前存储容器，等待处理数据放入

    for (auto tran_err_code : tran_Err_code.data) //将恢复正常的数据，释放，并存在静态日志内
    {
        bool exsit_flag{false};
        for (auto err_msg : msg.data)
        {
            if (tran_err_code.module_name == err_msg.module_name && tran_err_code.error_code == err_msg.error_code && tran_err_code.error_info == err_msg.error_info)
            {
                exsit_flag = true;
            }
        }
        if (exsit_flag == true) //存在相同的，则放回当前存储容器
        {
            cur_Err_code.data.push_back(tran_err_code);
        }
        else //不存在，则放到存在静态日志
        {
            //relea_Err_code.data.push_back( tran_err_code );
            //save_web_diary(tran_err_code,merge_nameAndmodule(tran_err_code));
            save_web_diary("信息：" + tran_err_code.error_info + "    优先级：" + to_string(tran_err_code.level), merge_nameAndmodule(tran_err_code), tran_err_code.status_info);
            save_web_diary("信息：恢复正常", merge_nameAndmodule(tran_err_code));
        }
    }

    tran_Err_code.data.clear();             //第三次清空处理空间
    tran_Err_code.data = cur_Err_code.data; //将现存数据放入处理空间中
    rank_Err_code.data.clear();

    for (int rank = 0; rank <= 15; rank++) //将数据按危害等级排序，并发布到动态数据
    {
        for (auto tran_err_code : tran_Err_code.data)
        {
            if (tran_err_code.level == rank)
            {
                rank_Err_code.data.push_back(tran_err_code);
            }
        }
    }

    dynamic_save_web_diary(rank_Err_code); //发布动态错误代码

    err_end_flag = true;
    err_monitor_timer_ = ros::Time::now().toSec();
}

void MapSample::camera2_Callback(const sensor_msgs::Image &msg)
{
    static double start_time = ros::Time::now().toSec();
    if (camera2_flag == true && ros::Time::now().toSec() - start_time > 1)
    {
        std_msgs::String camera;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        cv::Mat img__ = cv_ptr->image;
        camera.data = "data:image/png;base64," + Mat2Base64(img__, "png");
        camera_image_encodebase64_pub.publish(camera);
        start_time = ros::Time::now().toSec();
    }
}

void MapSample::camera1_Callback(const sensor_msgs::Image &msg)
{
    if (camera1_flag == false)
    {
        std_msgs::String camera;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img__ = cv_ptr->image;
        camera.data = "data:image/png;base64," + Mat2Base64(img__, "png");
        camera_image_encodebase64_pub.publish(camera);
        camera1_flag = true;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////    箱子电量
void MapSample::dustboxbat_state_Callback(const cti_msgs::BatteryCellsState &msg)
{
    Json::Value arrayObj_top;
    Json::FastWriter writer;
    std_msgs::String web_msg;
    Json::Value item;
    int order = 1;

    item["Bat_Volt"] = Json::Value(msg.Volt_All);
    item["Bat_Soc"] = Json::Value(msg.Soc_All);
    item["name"] = Json::Value("总电池");
    arrayObj_top.append(item);

    for (auto bat : msg.BatCells)
    {
        item["Bat_Volt"] = Json::Value(bat.Bat_Volt);
        item["Bat_Soc"] = Json::Value(bat.Bat_Soc);
        item["name"] = Json::Value("分电池" + std::to_string(order) + "号");
        arrayObj_top.append(item);
        order++;
    }

    web_msg.data = writer.write(arrayObj_top);
    dustbox_battery_web_pub.publish(web_msg);
    order = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////    地盘电量
void MapSample::batcell_state_Callback(const cti_msgs::BatteryCellsState &msg)
{
    Json::Value arrayObj_top;
    Json::FastWriter writer;
    std_msgs::String web_msg;
    Json::Value item;
    int order = 1;

    item["Bat_Volt"] = Json::Value(msg.Volt_All);
    item["Bat_Soc"] = Json::Value(msg.Soc_All);
    item["name"] = Json::Value("总电池");
    arrayObj_top.append(item);

    for (auto bat : msg.BatCells)
    {
        item["Bat_Volt"] = Json::Value(bat.Bat_Volt);
        item["Bat_Soc"] = Json::Value(bat.Bat_Soc);
        item["name"] = Json::Value("分电池" + std::to_string(order) + "号");
        arrayObj_top.append(item);
        order++;
    }

    item["Bat_Volt"] = Json::Value(msg.Bat_backup_Volt);
    item["Bat_Soc"] = Json::Value("无");
    item["name"] = Json::Value("备用电池");
    arrayObj_top.append(item);

    web_msg.data = writer.write(arrayObj_top);
    vihicle_battery_web_pub.publish(web_msg);
    order = 0;
}

void MapSample::roadedge_point_Callback(const cti_vector_map_msgs::RoadEdgeArray &msg)
{
    Json::Value arrayObj_top;
    Json::FastWriter writer;
    std_msgs::String web_msg;
    Json::Value item;

    for (auto content : msg.data)
    {
        item["x"] = Json::Value(content.pose.position.x);
        item["y"] = Json::Value(content.pose.position.y);
        item["z"] = Json::Value(content.pose.position.z);
        item["type"] = Json::Value(content.ltype);
        item["numb"] = Json::Value(content.id);
        arrayObj_top.append(item);
    }
    web_msg.data = writer.write(arrayObj_top);

    roadedge_point_web_pub.publish(web_msg);
}

void MapSample::cleararea_point_Callback(const cti_vector_map_msgs::ClearAreaArray &msg)
{
    Json::Value arrayObj_top;
    Json::FastWriter writer;
    std_msgs::String web_msg;
    Json::Value item;

    for (auto content : msg.data)
    {
        item["x"] = Json::Value(content.pose.position.x);
        item["y"] = Json::Value(content.pose.position.y);
        item["z"] = Json::Value(content.pose.position.z);
        item["type"] = Json::Value(content.ltype);
        item["numb"] = Json::Value(content.id);
        arrayObj_top.append(item);
    }
    web_msg.data = writer.write(arrayObj_top);

    cleararea_point_web_pub.publish(web_msg);
}

////////////////////////////////////////////////////////////////////////////////////////////

void MapSample::Imu_statu_Callback(const sensor_msgs::Imu &msg)
{
    Quaternion quaternion;
    Coordination_pose Coordin_pose;
    Json::FastWriter writer;
    Json::Value item;
    std_msgs::String web_msg;

    quaternion.x = msg.orientation.x;
    quaternion.y = msg.orientation.y;
    quaternion.z = msg.orientation.z;
    quaternion.w = msg.orientation.w;

    Coordin_pose.angl = ToEulerAngles(quaternion);

    double pitch = Coordin_pose.angl.pitch * 180 / 3.14;
    double roll = Coordin_pose.angl.roll * 180 / 3.14;
    double yaw = Coordin_pose.angl.yaw * 180 / 3.14;

    item["pitch"] = Json::Value(pitch);
    item["roll"] = Json::Value(roll);
    item["yaw"] = Json::Value(yaw);

    web_msg.data = writer.write(item);
    Imu_web_pub.publish(web_msg);
}

/////////////////////////////////////////////////////////////////////////////////////前方施工

/////////////////////////////////////////////////////////////////////////////////////以下是发送摄像机第三视角坐标
void MapSample::ndt_pose_Callback(const geometry_msgs::PoseStamped &msg)
{

    //创建一个点，用来存放当前base_laser的点在base_link坐标系中的位置
    geometry_msgs::PointStamped base_point;

    tf::StampedTransform transform_base; //定义存放变换关系的变量
    //监听两个坐标系之间的变换
    try
    {
        tf_listen_.lookupTransform("/map", "/base_vihicle_seconcd_person_view", ros::Time(0), transform_base);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    base_point.point.x = transform_base.getOrigin().x();
    base_point.point.y = transform_base.getOrigin().y();
    base_point.point.z = transform_base.getOrigin().z();

    web_tf_person_view_pub.publish(base_point);
}
//////////////////////////////////////////////////////////////////////////////////////以上是发送摄像机第三视角坐标

//////////////////////////////////////////////////////////////////////////////////////

void MapSample::Check_Mapping_State()
{
    if (Mappint_last_time != -1)
    {
        if (ros::Time::now().toSec() - Mappint_last_time > 3)
        {
            // if (data.message != "建图异常，请处理...")
            // {
            //     save_web_diary("建图异常，请处理...");
            // }
            data.message = "建图异常，请处理...";
            Info("建图异常，请处理...!");
        }
        else
        {
            data.message = "初始化成功，建图正在进行中...";
            Mappint_last_time = ros::Time::now().toSec();
            Info("初始化成功，建图正在进行中...");
        }
    }
}

void MapSample::Mapping_statu_Callback(const std_msgs::Bool &msg)
{
    if (flag_Mapping_start == true)
    {
        Mappint_last_time = ros::Time::now().toSec();
    }
}

/////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////

void MapSample::NavDeliveryResp_Callback(const cti_rblite_msgs::NavDeliveryResp &msg)
{

    Json::Value arrayObj_top;
    Json::FastWriter writer;
    Json::Value item;
    std_msgs::String web_msg;

    int a = msg.code;

    item["message"] = Json::Value(msg.message);
    item["code"] = Json::Value(a);
    item["command_state"] = Json::Value(msg.nav_header.command_state);
    item["command_type"] = Json::Value(msg.nav_header.command_type);
    arrayObj_top.append(item);
    web_msg.data = writer.write(arrayObj_top);
    NavDeliveryResp_web_pub.publish(web_msg);
}

///////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////

void MapSample::box_msg_Callback(const cti_rblite_msgs::BoxAskResponse &msg)
{
    Json::Value arrayObj_top;
    Json::FastWriter writer;
    Json::Value item;
    std_msgs::String web_msg;

    for (auto state : msg.infos)
    {
        item["qr"] = Json::Value(state.qr);
        item["hive_type"] = Json::Value(state.hive_type);
        item["position"] = Json::Value(state.position);
        item["hive_device_type"] = Json::Value(state.hive_device_type);
        arrayObj_top.append(item);
    }
    web_msg.data = writer.write(arrayObj_top);
    box_msg_web_pub.publish(web_msg);
}

///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////以下超声波超时日志
void MapSample::ult1_state_Callback(const std_msgs::Float32MultiArray &msg)
{
    static double last_time = ros::Time::now().toSec();
    if (ros::Time::now().toSec() - last_time > 1)
    {
        static double last_time_record = ros::Time::now().toSec();
        static int flag = 1;
        if (ros::Time::now().toSec() - last_time_record > 1000 || flag == 1)
        {
            std::string alt_error_diary = "超声波模块1超时";
            pubErrorMsg("SENSOR", 3, 11, alt_error_diary);
            flag = 0;
            last_time_record = ros::Time::now().toSec();
        }
    }
    last_time = ros::Time::now().toSec();
}

void MapSample::ult2_state_Callback(const std_msgs::Float32MultiArray &msg)
{
    static double last_time = ros::Time::now().toSec();
    if (ros::Time::now().toSec() - last_time > 1)
    {
        static double last_time_record = ros::Time::now().toSec();
        static int flag = 1;
        if (ros::Time::now().toSec() - last_time_record > 1000 || flag == 1)
        {
            std::string alt_error_diary = "超声波模块2超时";
            pubErrorMsg("SENSOR", 3, 11, alt_error_diary);
            flag = 0;
            last_time_record = ros::Time::now().toSec();
        }
    }
    last_time = ros::Time::now().toSec();
}

void MapSample::ult0_state_Callback(const std_msgs::Float32MultiArray &msg)
{
    static double last_time = ros::Time::now().toSec();
    if (ros::Time::now().toSec() - last_time > 1)
    {
        static double last_time_record = ros::Time::now().toSec();
        static int flag = 1;
        if (ros::Time::now().toSec() - last_time_record > 1000 || flag == 1)
        {
            std::string alt_error_diary = "超声波模块0超时";
            pubErrorMsg("SENSOR", 3, 11, alt_error_diary);
            flag = 0;
            last_time_record = ros::Time::now().toSec();
        }
    }
    last_time = ros::Time::now().toSec();
}

///////////////////////////////////////////////////////////////////////////////////////////////以上是超声波超时日志

//////////////////////////////////////////////////////////////////////////////////////////////以下是节点日志

void MapSample::node_alive_Callback(const diagnostic_msgs::DiagnosticArray &msg)
{
    static diagnostic_msgs::DiagnosticArray last_msg = msg;
    static int flag{0};
    static int i{0};
    for (auto obj : msg.status)
    {
        if (flag == 0)
        {
            if (obj.level != 0)
            {
                std::string alt_error_diary = obj.message;
                pubErrorMsg("SYSTEM", 1, 12, alt_error_diary);
            }
        }
        else if (obj.level != last_msg.status[i].level)
        {
            std::string alt_error_diary = obj.message;
            pubErrorMsg("SYSTEM", 1, 12, alt_error_diary);
        }
        i++;
    }
    last_msg = msg;
    flag = 1;
    i = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////以上是节点日志

void MapSample::senfor_state_Callback(const cti_msgs::BoxState &msg)
{
    Json::Value arrayObj_top;
    Json::FastWriter writer;
    Json::Value item;
    std_msgs::String web_msg;
    static std::string last_data[20];
    static int flag{0};
    int i{0};

    for (auto state : msg.states)
    {
        item["name"] = Json::Value(state.name);
        item["statu"] = Json::Value(state.message);
        arrayObj_top.append(item);
        if (last_data[i] == state.message)
        {
            i++;
            continue;
        }
        if (flag == 0)
        {
            std::string message_state = " is normal";
            if (state.message == message_state)
            {
                last_data[i] = state.message;
                i++;
                continue;
            }
        }
        last_data[i] = state.message;
        i++;
    }
    flag = 1;
    web_msg.data = writer.write(arrayObj_top);
    sensor_status_web_pub.publish(web_msg);
}

void MapSample::points_data_Callback(const std_msgs::String &msg)
{
    std::string path_tosave_file;
    if (msg.data.substr(9, 1) == "1")
    {
        path_tosave_file = "/opt/cti/kinetic/share/ctiwww/www/js/design/line2.js";
    }
    else
    {
        path_tosave_file = "/opt/cti/kinetic/share/ctiwww/www/js/design/line3.js";
    }

    std::ofstream outFile;
    //打开文件
    outFile.open(path_tosave_file);

    //写入数据
    if (msg.data.substr(9, 1) == "1")
    {
        outFile << "var line_origal_roadage =" << msg.data;
    }
    else
    {
        outFile << "var line_origal_cleararea =" << msg.data;
    }

    //关闭文件
    outFile.close();

    std::cout << "success to save a document named line.js" << std::endl;
}

void MapSample::points_diary_Callback(const std_msgs::String &msg)
{
    std::ofstream outFile;
    std::string filename;
    std::string path;
    std::string time = boost::lexical_cast<std::string>(boost::posix_time::second_clock::local_time());

    filename = time + "points_diary.txt";
    path = "/opt/cti/kinetic/share/ctiwww/www/js/diary/" + filename;

    //打开文件
    outFile.open(path);
    //写入数据
    outFile << msg.data;
    //关闭文件
    outFile.close();

    std::cout << "success to save a document named points_diary.txt" << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////    日志系统

//////////////////////////////////////////////////////////////////////////////////////前方施工
void MapSample::mapToJpgCallback(const nav_msgs::OccupancyGrid &msg)
{
    int height, width;
    height = msg.info.height;
    width = msg.info.width;

    std::string name;

    std_msgs::String png_encode64_rosmsg;
    std::vector<cti_msgs::TabState> coordinations;

    Json::Value arrayObj_top;
    Json::FastWriter writer;
    Json::Value item;
    std_msgs::String web_msg;

    //构造一个H行W列的矩阵
    cv::Mat dst = cv::Mat(height, width, CV_8UC3);

    static int sum_j_w = 0, sum_i_h = 0, count_sum = 0;

    //ROS中的地图是串行数据，要将数据传输到Mat矩阵中
    //显示代价地图123456

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        { //////车子
            if (msg.data[i * width + j] == -1)
            {
                dst.at<cv::Vec3b>(height - i - 1, j)[0] = 0;
                dst.at<cv::Vec3b>(height - i - 1, j)[1] = 238;
                dst.at<cv::Vec3b>(height - i - 1, j)[2] = 238;
            }
            else if (msg.data[i * width + j] == 0) /////背景
            {
                dst.at<cv::Vec3b>(height - i - 1, j)[0] = 220;
                dst.at<cv::Vec3b>(height - i - 1, j)[1] = 220;
                dst.at<cv::Vec3b>(height - i - 1, j)[2] = 220;
            }
            else if (msg.data[i * width + j] == -5) /////     红路线
            {
                dst.at<cv::Vec3b>(height - i - 1, j)[0] = 0;
                dst.at<cv::Vec3b>(height - i - 1, j)[1] = 0;
                dst.at<cv::Vec3b>(height - i - 1, j)[2] = 255;
            }
            else if (msg.data[i * width + j] == -15 || msg.data[i * width + j] == -100) /////      车子坐标 及 路线  //绿色 局部路线
            {
                dst.at<cv::Vec3b>(height - i - 1, j)[0] = 0;
                dst.at<cv::Vec3b>(height - i - 1, j)[1] = 255;
                dst.at<cv::Vec3b>(height - i - 1, j)[2] = 0;
            }
            else if (msg.data[i * width + j] == -50) /////动态             //红色 全局路线
            {
                dst.at<cv::Vec3b>(height - i - 1, j)[0] = 255;
                dst.at<cv::Vec3b>(height - i - 1, j)[1] = 0;
                dst.at<cv::Vec3b>(height - i - 1, j)[2] = 0;
            }
            else if (msg.data[i * width + j] == 50) /////车上灰点
            {
                dst.at<cv::Vec3b>(height - i - 1, j)[0] = 220;
                dst.at<cv::Vec3b>(height - i - 1, j)[1] = 220;
                dst.at<cv::Vec3b>(height - i - 1, j)[2] = 220;
            }
            else if (msg.data[i * width + j] == 88 || msg.data[i * width + j] == 98 ||
                     msg.data[i * width + j] == 91 || msg.data[i * width + j] == 81) /////静态障碍物 黑边
            {
                dst.at<cv::Vec3b>(height - i - 1, j)[0] = 0;
                dst.at<cv::Vec3b>(height - i - 1, j)[1] = 0;
                dst.at<cv::Vec3b>(height - i - 1, j)[2] = 0;
            }
            else if (msg.data[i * width + j] == 100) ///// 路边线  紫色
            {
                dst.at<cv::Vec3b>(height - i - 1, j)[0] = 150;
                dst.at<cv::Vec3b>(height - i - 1, j)[1] = 0;
                dst.at<cv::Vec3b>(height - i - 1, j)[2] = 200;
            }
            else if (msg.data[i * width + j] == -2) ///// 车头显示
            {
                dst.at<cv::Vec3b>(height - i - 1, j)[0] = 150;
                dst.at<cv::Vec3b>(height - i - 1, j)[1] = 238;
                dst.at<cv::Vec3b>(height - i - 1, j)[2] = 238;
            }
        }
    }

    // //添加小车雷达坐标
    Coordination_pose coordination_pose;
    draw_coordinations(dst, coordination_pose, height, width);

    coordination_pose = add_coordinations("/base_link", "/rslidar", tf_listen_, coordinations);
    draw_coordinations(dst, coordination_pose, height, width);

    coordination_pose = add_coordinations("/base_link", "/livox_frame", tf_listen_, coordinations);
    draw_coordinations(dst, coordination_pose, height, width);

    coordination_pose = add_coordinations("/base_link", "/rsbpearl", tf_listen_, coordinations);
    draw_coordinations(dst, coordination_pose, height, width);

    coordination_pose = add_coordinations("/base_link", "/lh_back_laser_link", tf_listen_, coordinations);
    draw_coordinations(dst, coordination_pose, height, width);

    if (this->lidar_mode == -1)
    {
        //地图翻转270度
        dst = matRotateClockWise270(dst);
        png_encode64_rosmsg.data = "data:image/png;base64," + Mat2Base64(dst, "png");
        mapToPng_encodebase64_pub.publish(png_encode64_rosmsg);
        return;
    }
    else if (this->lidar_mode == 1)
    {
        //构造一个H行W列的矩阵
        //roi 是表示 img 中 Rect(10,10,100,100)区域的对象
        cv::Mat roi(dst, cv::Rect(70, 80, 60, 40)); //heigth width
        roi = matRotateClockWise270(roi);
        //imgType 包括png bmp jpg jpeg等opencv能够进行编码解码的文件
        png_encode64_rosmsg.data = "data:image/png;base64," + Mat2Base64(roi, "png");
        mapToPng_encodebase64_pub.publish(png_encode64_rosmsg);

        for (auto state : coordinations)
        {
            item["name"] = Json::Value(state.name);
            item["statu"] = Json::Value(state.message);
            arrayObj_top.append(item);
        }
        web_msg.data = writer.write(arrayObj_top);
        web_tf_msg_pub.publish(web_msg);
        return;
    }
}

void MapSample::rosweb_cmd_lidar(const std_msgs::Int32 &msg)
{
    //this->lidar_mode = msg.data;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////前方施工

void MapSample::sendWebDataState(bool trigger)
{
    static double start_time;
    if (((ros::Time::now().toSec() - start_time) > 0.5) || trigger)
    {
        if (mapsample.MapSampleState == MAPSAMPLE_STOP)
        {
            data.mapping_state = false;
        }
        //--
        
        std_msgs::String msg;
        Json::Value root;
        Json::FastWriter writer;
        root["longitude"] = Json::Value(data.lon);
        root["latitude"] = Json::Value(data.lat);
        root["altitude"] = Json::Value(data.alt);
        root["lat_err"] = Json::Value(data.lat_err);
        root["lon_err"] = Json::Value(data.lon_err);
        root["sats_used"] = Json::Value(data.sats_used);
        root["runstate"] = data.mapping_state ? Json::Value("建图中") : Json::Value("空闲");
        root["sensorstate"] = data.sensor_state ? Json::Value("正常") : Json::Value("异常");
        root["message"] = Json::Value(data.message);

        msg.data = writer.write(root);

        webstate_pub.publish(msg);
        start_time = ros::Time::now().toSec();
    }
}

void MapSample::thread_web_lidar_calibration(const std::string &msg)
{
    system(msg.c_str());
    //system("rosnode kill calibration");
}

void MapSample::rosWebCmdCallback(const std_msgs::Int32 &msg)
{
    std::cout << "web 接收：" << msg.data << std::endl;
    if (msg.data == 1)
    {
        flag_Mapping_start = true;
        if (!upload.enable)
        {
            if (mapsample.startMapping())
            {
                data.message = "现在开始建图,正在初始化...";
                Info("现在开始建图,正在初始化...");
            }
            else
            {
                data.message = "指令文件不存在...";
                Info("指令文件不存在!");
            }
        }
        else
        {
            data.message = "数据在上传中，无法开始建图...";
            Info("数据在上传中，无法开始建图...");
        }
    }
    else if (msg.data == 2)
    {
        Mappint_last_time = -1;
        flag_Mapping_start = false;
        if (!upload.enable)
        {
            if (mapsample.MapSampleState == MAPSAMPLE_START)
            {

                if (mapsample.stopMapping())
                {
                    data.message = "结束建图，数据处理完成...";
                    Info("结束建图，数据处理完成...");
                }
                else
                {
                    data.message = "指令文件不存在...";
                    Info("指令文件不存在!");
                }
            }
            else
            {
                data.message = "还未开始建图，无法结束建图...";
                Info("还未开始建图，无法结束建图...");
            }
        }
        else
        {
            data.message = "数据在上传中，不支持该功能...";
            Info("数据在上传中，不支持该功能...");
        }
    }
    else if (msg.data == 3)
    {
        if (upload.upload(mapsample.pathfile))
        {
            data.message = "数据在上传中,请等待...";
            Info("数据在上传中,请等待...");
        }
        else
        {
            data.message = "文件\"" + mapsample.pathfile + "\"不存在...";
            Info("文件\"" << mapsample.pathfile << "\"不存在...");
        }
    }
    else if (msg.data == -1)
    {
        std::cout << "-------------------shutdown------------" << std::endl;
        std::string cmd_str = shutdown_cmd_path + "shutdown_now.sh";
        std::system(cmd_str.c_str());
        Info("建图，关机");
    }
    else if (msg.data == 10)
    {
        std::cout << "-------------------screenshot-camera-picture--------------/cti/chassis_serial/sanitation_dustbox_state----" << std::endl;
        camera1_flag = false;
    }
    else if (msg.data == 11)
    {
        camera_cmd.data = "open camera";
        camera2_cmd_pub.publish(camera_cmd);
        std::cout << "-------------------open-camera2-------------------------------" << std::endl;
        Info("open-back_camera");
        camera2_flag = true;
    }
    else if (msg.data == 12)
    {
        camera_cmd.data = "calib finished";
        camera2_cmd_pub.publish(camera_cmd);
        Info("back_camera calib finished");
        std::cout << "-------------------close-camera2-------------------------------" << std::endl;
        camera2_flag = false;
    }
    else if (msg.data == 13)
    {
        camera_cmd.data = "calibrate";
        camera2_cmd_pub.publish(camera_cmd);
        Info("back_camera calibing");
        std::cout << "-------------------calibrate-camera2---------------------------" << std::endl;
    }
    else if (msg.data == 14)
    {
        camera_cmd.data = "RESET";
        camera2_cmd_pub.publish(camera_cmd);
        Info("back_camera RESET");
        std::cout << "-------------------reset-camera2--------------------------------" << std::endl;
    }
    else if (msg.data == 50)
    {
        rslidar_web_switch = !rslidar_web_switch;
        std::cout << "-------------------rslidar_web_switch--------------------------------" << std::endl;
        Info("rslidar_web_switch");
    }
    else if (msg.data == 51)
    {
        rslidar_Bpearl_web_switch = !rslidar_Bpearl_web_switch;
        std::cout << "-------------------rslidar_Bpearl_web_switch--------------------------------" << std::endl;
        Info("rslidar_Bpearl_web_switch");
    }
    else if (msg.data == 52)
    {
        rslidar_back_scan_web_switch = !rslidar_back_scan_web_switch;
        std::cout << "-------------------rslidar_back_scan_web_switch--------------------------------" << std::endl;
        Info("rslidar_back_scan_web_switch");
    }
    else if (msg.data == 53)
    {
        rslidar_mid_scan_switch = !rslidar_mid_scan_switch;
        std::cout << "-------------------rslidar_mid_scan_switch--------------------------------" << std::endl;
        Info("rslidar_mid_scan_switch");
    }
    else if (msg.data == 54)
    {
        rslidar_livox_web_switch = !rslidar_livox_web_switch;
        std::cout << "-------------------rslidar_livox_web_switch--------------------------------" << std::endl;
        Info("rslidar_livox_web_switch");
    }
    else if (msg.data == 71)
    {
        std::thread new_thread(&MapSample::thread_web_lidar_calibration, this, "roslaunch calibration calibration_v5.launch"); //子线程
        new_thread.detach();
        Info("switch on lidar calibrate_node v5 launch");
        std::cout << "-------------------switch on lidar calibrate_node v5 launch  --------------------------------" << std::endl;
    }
    else if (msg.data == 72)
    {
        std::thread new_thread(&MapSample::thread_web_lidar_calibration, this, "roslaunch calibration calibration_v6.launch"); //子线程
        new_thread.detach();
        Info("switch on lidar calibrate_node v6 launch");
        std::cout << "-------------------switch on lidar calibrate_node v6 launch  --------------------------------" << std::endl;
    }
    else if (msg.data == 73)
    {
        std::thread new_thread(&MapSample::thread_web_lidar_calibration, this, "roslaunch calibration calibration_v7.launch"); //子线程
        new_thread.detach();
        Info("switch on lidar calibrate_node v7 launch");
        std::cout << "-------------------switch on lidar calibrate_node v7 launch  --------------------------------" << std::endl;
    }
    else if (msg.data == 74)
    {
        std::thread new_thread(&MapSample::thread_web_lidar_calibration, this, "roslaunch calibration calibration_v8.launch"); //子线程
        new_thread.detach();
        Info("switch on lidar calibrate_node v8 launch");
        std::cout << "-------------------switch on lidar calibrate_node v8 launch  --------------------------------" << std::endl;
    }
    else if (msg.data == 90)
    {
        system("rosnode kill calibration");
        //system("rosnode kill cti_tf");
        rslidar_web_switch = 0;
        rslidar_Bpearl_web_switch = 0;
        rslidar_back_scan_web_switch = 0;
        std::cout << "------------------- try to kill calibrate and tf  --------------------------------" << std::endl;
        Info("try to kill calibrate and tf");
    }
    else if (msg.data == 91)
    {
        openAndsendtf_luanch();
        std::cout << "------------------- open tf.launch  --------------------------------" << std::endl;
        Info("open tf.launch");
    }
    else
    {
        std::cout << "web 指令无法识别" << std::endl;
    }
    sendWebDataState(true);
}

void MapSample::mappingStatusCallback(const std_msgs::Bool &msg)
{
    data.mapping_state = msg.data;
}

void MapSample::sensorStateCallback(const std_msgs::Bool &msg)
{
    data.sensor_state = msg.data;
}

void MapSample::err_monitor() //当无err回调时，清除所有异常,设定时间为20秒
{
    if (err_end_flag)
    {
        if (ros::Time::now().toSec() - err_monitor_timer_ > 20)
        {
            std::cout << "wojinlaile" << std::endl;
            cti_msgs::ErrorStatusArray msg;
            error_msg_Callback(msg);
            err_end_flag = false;
            err_monitor_timer_ = 0;
        }
    }
}

void MapSample::gnnsLocationCallback(const cti_msgs::GnssRTKConstPtr &nav_satfix)
{
    data.lat = nav_satfix->lat;
    data.lon = nav_satfix->lon;
    data.alt = nav_satfix->alt;
    data.lat_err = nav_satfix->lat_err;
    data.lon_err = nav_satfix->lon_err;
    data.sats_used = nav_satfix->sats_used;
}

void MapSample::pub_nav_version()
{
    std_msgs::String aa;
    aa.data = CTI_SW_VER;
    nav_version_web_pub.publish(aa);
}

void MapSample::pub_webTimer()
{
    std_msgs::String pose;
    web_timer_pub.publish(pose);
}

void MapSample::timercallback(const ros::TimerEvent &)
{
    sendWebDataState();
    // sendRoadedge();
    // sendCleararea();
    Check_Mapping_State();
    pub_nav_version();
    err_monitor();
    pub_webTimer();
    check_tf_exit_luanch();
    pub_web_red_err();
}

////////////////////////////////// original version
// void MapSample::error_msg_Callback( cti_msgs::ErrorStatusArray &msg)                               //错误码收发
// {
//     std::cout<<"kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk"<<std::endl;
//     std::cout<<msg.data.size()<<std::endl;
//     if(msg.data.size() == 0 )
//     {
//         for (auto rec_err_msg : rec_Err_msg.data)
//         {
//             bool flag_last{false};
//             for( cur_err_msg : cur_Err_msg.data)
//             {
//                 if ( rec_err_msg.error_code == cur_err_msg.error_code && rec_err_msg.module_name == cur_err_msg.module_name )
//                 {
//                     flag_last = true;
//                 }
//             }
//             if( flag_last == false )
//             {
//                 cur_Err_msg.data.push_back( rec_err_msg );
//             }
//         }

//         for( auto cur_err_msg : cur_Err_msg.data )                                                                          //存储恢复正常的错误码
//         {
//             save_web_diary("-----已经恢复正常last-----",merge_nameAndmodule(cur_err_msg) );
//             std::cout<<"22222222222222222222222222"<<std::endl;
//         }
//         rec_Err_msg.data.clear();
//         cur_Err_msg.data.clear();
//         err_end_flag = false;
//         return;
//     }

//     if( flag_first_in )                                                                                     // 第一次收到的信息直接存放
//     {
//         for( auto err_msg : msg.data )
//         {
//             rec_Err_msg.data.push_back(err_msg);
//             cur_Err_msg.data.push_back(err_msg);
//             save_web_diary(err_msg.error_info, merge_nameAndmodule(err_msg) );
//         }
//         flag_first_in = false;

//         err_end_flag = true;
//         err_monitor_timer_ = ros::Time::now().toSec();
//         return;
//     }

//     for( auto cur_err_msg : rec_Err_msg.data )                                                                            // 取出恢复正常的错误码
//     {
//         bool flag{false};
//         for (auto err_msg : msg.data)                                                                                     // 判断是否上一次发过的错误码，这次是否还发同样的错误码
//         {
//             if ( cur_err_msg.error_code == err_msg.error_code && cur_err_msg.module_name == err_msg.module_name )
//             {
//                 flag = true;
//             }
//         }
//         if(flag == false)
//         {
//             throw_Err_msg.data.push_back(cur_err_msg);
//         }ErrorStatusArray
//     }

//     for( auto cur_err_msg : throw_Err_msg.data )                                                                            //  剔除储存已经恢复正常的错误码
//     {
//         for (auto iter = rec_Err_msg.data.begin(); iter != rec_Err_msg.data.end(); ++iter)
//         {
//             if (iter->error_code == cur_err_msg.error_code && iter->module_name == cur_err_msg.module_name )
//             {
//                 std::cout<<iter->error_code<<"--------"<<iter->module_name<<std::endl;
//                 rec_Err_msg.data.erase(iter);
//                 break;
//             }
//         }
//     }

//     for( auto throw_err_msg : throw_Err_msg.data )                                                                          //存储恢复正常的错误码
//     {
//         save_web_diary("---已经恢复正常 doing----", merge_nameAndmodule(throw_err_msg)  );
//     }
//     throw_Err_msg.data.clear();

//     for( auto cur_err_msg : msg.data )                                                                              // 將重复的错误码储存起来
//     {
//         for( auto err_msg : cur_Err_msg.data  )
//         {
//             if( err_msg.module_name == cur_err_msg.module_name && err_msg.error_code == cur_err_msg.error_code )
//             {
//                 bool flag{0};
//                 for( auto rec_err_msg : rec_Err_msg.data )                                                                  //重复的不放入
//                 {
//                     if( rec_err_msg.module_name == err_msg.module_name && rec_err_msg.error_code == err_msg.error_code )
//                     {
//                         flag = true;
//                     }
//                 }
//                 if(flag == 0 )
//                 {
//                     rec_Err_msg.data.push_back(err_msg);
//                 }
//             }
//         }
//     }

//     for (auto cur_err_msg : rec_Err_msg.data)
//     {
//         cur_Err_msg.data.push_back( cur_err_msg );
//     }

//     for( auto cur_err_msg : msg.data )                                                                                       //  剔除重复错误码
//     {
//         bool flag{false};
//         for ( auto iter = cur_Err_msg.data.begin(); iter != cur_Err_msg.data.end(); ++iter )
//         {
//             if (iter->error_code == cur_err_msg.error_code && iter->module_name == cur_err_msg.module_name )
//             {
//                 flag = true;
//             }
//         }
//         if(flag == false)
//         {
//             cur2_Err_msg.data.push_back(cur_err_msg);
//         }
//     }

//     cur_Err_msg.data.clear();
//     cur_Err_msg.data = cur2_Err_msg.data;
//     cur2_Err_msg.data.clear();

//     for( auto cur_err_msg : cur_Err_msg.data )                                                                               //  存储最新的错误码
//     {
//         save_web_diary(cur_err_msg.error_info, merge_nameAndmodule(cur_err_msg) );
//     }

//     err_end_flag = true;
//     err_monitor_timer_ = ros::Time::now().toSec();

// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////ftp
// //上传打印线程
// int upload_print_thread(){
// //获取本地文件大小
// double local_file_size = 0.0f;
// struct stat statbuf;
// if(stat(local_file_path.c_str(),&statbuf) == 0){
//     local_file_size = statbuf.st_size;
//     printf("local file size>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>%0.fKB\n",local_file_size);
// }else{
//     ROS_ERROR("%s does not exist!\n",local_file_path.c_str());
//     print_thread_finished = true;
//     return -1;
// }

// //获取远程文件大小
// double remote_file_size = 0.0f;
// embeddedmz::CFTPClient cti_CFTPClient_print;
// //cti_CFTPClient_print.m_oLog = std::move(print_log);
// if(!cti_CFTPClient_print.InitSession(ftp_host, ftp_port, ftp_user, ftp_pwd,embeddedmz::CFTPClient::FTP_PROTOCOL::FTP,embeddedmz::CFTPClient::SettingsFlag::ALL_FLAGS))
// {
//     ROS_ERROR("Can not init ftp session!");
//     print_thread_finished = true;
//     return -1;
// }

// //打印
// embeddedmz::CFTPClient::FileInfo ResFileInfo = {0,0.0};
// printf("\033[?25l");//隐藏光标
// while (ros_ok){
//     sleep(1);
//     static double old_remote_file_size = 0.0f;
//     bool bRes =false;
//     while(!bRes){
//         bRes = cti_CFTPClient_print.Info(remote_file_path,ResFileInfo);
//     }
//     remote_file_size = ResFileInfo.dFileSize;
//     int upload_speed = (remote_file_size - old_remote_file_size) / 1024 / 1;
//     old_remote_file_size = remote_file_size;
//     int upload_percent = remote_file_size * 100 / local_file_size;
//     int upload_time_rest = (local_file_size - remote_file_size) / (upload_speed*1024);
//     int upload_time_rest_min = upload_time_rest / 60;
//     int upload_time_rest_sec = upload_time_rest % 60;
//     if(remote_file_size < 1024*1024*10){
//         if(upload_speed < 1024){
//             printf("%s  %d%%  %.0fKB   %dKB/s  %02d:%02d \033[K \n",file_name.c_str(),upload_percent,remote_file_size/1024,upload_speed,upload_time_rest_min,upload_time_rest_sec);
//         }else{
//             printf("%s  %d%%  %.0fKB   %dMB/s  %02d:%02d \033[K \n",file_name.c_str(),upload_percent,remote_file_size/1024,upload_speed/1024,upload_time_rest_min,upload_time_rest_sec);
//         }
//     }else{
//         if(upload_speed < 1024){
//             printf("%s  %d%%  %.0fMB   %dKB/s  %02d:%02d \033[K \n",file_name.c_str(),upload_percent,remote_file_size/(1024*1024),upload_speed,upload_time_rest_min,upload_time_rest_sec);
//         }else{
//             printf("%s  %d%%  %.0fMB   %dMB/s  %02d:%02d \033[K \n",file_name.c_str(),upload_percent,remote_file_size/(1024*1024),upload_speed/1024,upload_time_rest_min,upload_time_rest_sec);
//         }
//     }
//     if(local_file_size <= remote_file_size){
//         break;
//     }else if(local_file_size > remote_file_size){
//         printf("\r\033[4A ");
//     }
// }
// cti_CFTPClient_print.CleanupSession();
// print_thread_finished = true;

// typedef enum module_error_enum{
// motion_control_board = 1,
// left_front_driver = 2,
// right_front_driver = 3,
// left_rear_driver = 4,
// right_rear_driver = 5,
// front_turn_driver = 6,
// rear_turn_driver = 7,
// front_brake_driver = 8,
// rear_brake_driver = 9,
// battery_board = 10,
// }module_error_t;

// switch (msg->data[0])
// {
//     case motion_control_board:
//     Info(" ERR_MC: " << (int)msg->data[1]);
//     break;
//     case battery_board:
//     Info(" ERR_BAT: " << (int)msg->data[1]);
//     break;
//     case left_front_driver:
//     Info(" ERR_LFD: " << (int)msg->data[1]);
//     break;
//     case right_front_driver:
//     Info(" ERR_RFD: " << (int)msg->data[1]);
//     break;
//     case left_rear_driver:
//     Info(" ERR_LRD: " << (int)msg->data[1]);
//     break;
//     case right_rear_driver:
//     Info(" ERR_RRD: " << (int)msg->data[1]);
//     break;
//     case front_turn_driver:
//     Info(" ERR_FTD:: " << (int)msg->data[1]);
//     break;
//     case rear_turn_driver:
//     Info(" ERR_RTD: " << (int)msg->data[1]);
//     break;
//     case front_brake_driver:
//     Info(" ERR_RBD: " << (int)msg->data[1]);
//     break;
//     case rear_brake_driver:
//     Info(" ERR_RBD: " << (int)msg->data[1]);
//     break;
//     default:
//     break;
// }

// ros::Subscriber subImage_ = nh_.subscribe(image_topic_name_, 30, &Calibration::image_callback, this);

// void Calibration::image_callback(const sensor_msgs::ImageConstPtr &msg) {//////////////////////////////////ros image
//     cv::Mat img;
//     cv_bridge::CvImagePtr cv_ptr;
//     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     cv_ptr->image.copyTo(img);
//     //img.copyTo(new_img_);
//     new_img_t_ = msg->header.stamp.toSec();
// }

// // static int i{0};

// // i++;
// // if(msg.data.size() == 0 )
// // {
// //     for( auto cur_err_msg : rec_Err_msg.data )                                                                          //存储恢复正常的错误码
// //     {
// //         save_web_diary(cur_err_msg.module_name +"---已经恢复正常",cur_err_msg.error_code );
// //     }
// //     rec_Err_msg.data.clear();
// //     return;
// // }

// // for( auto err_msg : msg.data )
// // {
// //     bool exsit_same{false};
// //     if( rec_Err_msg.data.size() == 0 )
// //     {
// //         //std::cout<<"ssssssssssssssssssssssssssssssss"<<std::endl;
// //         rec_Err_msg.data.push_back(err_msg);
// //         save_web_diary(err_msg.module_name +"---"+to_string(err_msg.error_code) +"---"+ err_msg.error_info, err_msg.error_code );
// //         return;
// //     }

// //     for( rec_err_msg : rec_Err_msg.data )
// //     {
// //         if(rec_err_msg.error_code == err_msg.error_code && rec_err_msg.module_name == err_msg.module_name)
// //         {
// //             std::cout<<i<<"ssssssssssssssssssssssssssssssss"<<std::endl;
// //             exsit_same = true;
// //         }
// //     }

// //     if( exsit_same == false )
// //     {
// //         std::cout<<i<<"wwwwwwwwwwwwwwwwwwwwwwwwww"<<std::endl;
// //         mid_translation_msg.data.push_back(err_msg);
// //         save_web_diary(err_msg.module_name +"---"+to_string(err_msg.error_code) +"---"+ err_msg.error_info, err_msg.error_code );
// //     }
// // }
// // // for( auto err_msg : rec_Err_msg.data )
// // // {
// // //     bool exsit_same{false};
// // //     for( msg_data : msg.data )
// // //     {
// // //         if(msg_data.error_code == err_msg.error_code && msg_data.module_name == err_msg.module_name)
// // //         {
// // //             exsit_same = true;
// // //         }
// // //     }
// // //     if( exsit_same == false )
// // //     {
// // //         save_web_diary(err_msg.module_name +"---已经恢复正常",err_msg.error_code );
// // //     }
// // // }
// // if( mid_translation_msg.data.size() != 0 )
// // {
// //     rec_Err_msg.data.clear();
// //     rec_Err_msg.data = mid_translation_msg.data;
// //     mid_translation_msg.data.clear();
// // }

// // if( flag_first_in )                                                                                     // 第一次收到的信息直接存放
// // {
// //     for( auto err_msg : msg.data )
// //     {
// //         rec_Err_msg.data.push_back(err_msg);
// //         cur_Err_msg.data.push_back(err_msg);
// //         save_web_diary(err_msg.module_name +"---"+to_string(err_msg.error_code) +"---"+ err_msg.error_info, err_msg.error_code );
// //     }
// //     flag_first_in = false;
// //     return;
// // }

// // for( auto cur_err_msg : rec_Err_msg.data )                                                                            // 取出恢复正常的错误码
// // {
// //     bool flag{false};
// //     for (auto err_msg : msg.data)                                                                                     // 判断是否上一次发过的错误码，这次是否还发同样的错误码
// //     {
// //         if ( cur_err_msg.error_code == err_msg.error_code && cur_err_msg.module_name == err_msg.module_name )
// //         {
// //             flag = true;
// //         }
// //     }
// //     if(flag == false)
// //     {
// //         throw_Err_msg.data.push_back(cur_err_msg);
// //     }
// // }

// // for( auto cur_err_msg : throw_Err_msg.data )                                                                            //  剔除储存已经恢复正常的错误码
// // {
// //     for (auto iter = rec_Err_msg.data.begin(); iter != rec_Err_msg.data.end(); ++iter)
// //     {
// //         if (iter->error_code == cur_err_msg.error_code && iter->module_name == cur_err_msg.module_name )
// //         {
// //             rec_Err_msg.data.erase(iter);
// //             break;
// //         }
// //     }
// // }

// // for( auto throw_err_msg : throw_Err_msg.data )                                                                          //存储恢复正常的错误码
// // {
// //     save_web_diary(throw_err_msg.module_name +"---"+to_string(throw_err_msg.error_code) +"---已经恢复正常", throw_err_msg.error_code );
// // }
// // throw_Err_msg.data.clear();

// // for( auto cur_err_msg : msg.data )                                                                              // 將重复的错误码储存起来
// // {
// //     for( auto err_msg : cur_Err_msg.data  )
// //     {
// //         if( err_msg.module_name == cur_err_msg.module_name && err_msg.error_code == cur_err_msg.error_code )
// //         {
// //             bool flag{0};
// //             for( auto rec_err_msg : rec_Err_msg.data )                                                                  //重复的不放入
// //             {
// //                 if( rec_err_msg.module_name == err_msg.module_name && rec_err_msg.error_code == err_msg.error_code )
// //                 {
// //                     flag = true;
// //                 }
// //             }
// //             if(flag == 0 )
// //             {
// //                 rec_Err_msg.data.push_back(err_msg);
// //             }
// //         }
// //     }
// // }

// // for (auto cur_err_msg : rec_Err_msg.data)
// // {
// //     cur_Err_msg.data.push_back( cur_err_msg );
// // }

// // for( auto cur_err_msg : msg.data )                                                                                       //  剔除重复错误码
// // {
// //     bool flag{false};
// //     for ( auto iter = cur_Err_msg.data.begin(); iter != cur_Err_msg.data.end(); ++iter )
// //     {
// //         if (iter->error_code == cur_err_msg.error_code && iter->module_name == cur_err_msg.module_name )
// //         {
// //             flag = true;
// //         }
// //     }
// //     if(flag == false)
// //     {
// //         cur2_Err_msg.data.push_back(cur_err_msg);
// //     }
// // }

// // cur_Err_msg.data.clear();
// // cur_Err_msg.data = cur2_Err_msg.data;
// // cur2_Err_msg.data.clear();

// // for( auto cur_err_msg : cur_Err_msg.data )                                                                               //  存储最新的错误码
// // {
// //     save_web_diary(cur_err_msg.module_name +"---"+to_string(cur_err_msg.error_code) +"---"+ cur_err_msg.error_info, cur_err_msg.error_code );
// // }
// err_end_flag = true;
// err_monitor_timer_ = ros::Time::now().toSec();

// ros::Subscriber clean_function_test_sub = nh.subscribe("/cti/chassis_serial/clean_function_test", 1, cleanFunctionTest_Callback); //环卫车清扫功能测试

// //清扫功能测试回调函数
// void cleanFunctionTest_Callback(const cti_msgs::DataArray::ConstPtr &msg)
// {
//     if (stm32_update_flag)
//     {
//         return;
//     }
//     //数据解析
//     for (int i = 0; i < msg->datas.size(); i++)
//     {
//         cti_msgs::Data info_msg = msg->datas[i];
//         if (info_msg.name == "spray_motor")
//             //喷水电机 uint8_t 1/0    0
//             send_to_clean_cmd_new.spray_motor = atoi(info_msg.data.c_str());
//         else if (info_msg.name == "dam_board")
//             //挡板控制 uint8_t 0   1  2 bankai 3 kaihe
//             send_to_clean_cmd_new.dam_board = atoi(info_msg.data.c_str());
//         else if (info_msg.name == "sidebrush_transform")
//             //边刷伸展 uint8_t 100   0
//             send_to_clean_cmd_new.side_brush_transform = atoi(info_msg.data.c_str());
//         else if (info_msg.name == "sidebrush_speed")
//             //边刷转速 uint8_t 100  0
//             send_to_clean_cmd_new.side_brush_speed = atoi(info_msg.data.c_str());
//         else
//             continue;
//     }

// clean_function_test_report_pub = nh.advertise<cti_msgs::DataArray>("/cti/chassis_serial/clean_function_test_report", 10); //清扫功能测试结果发布

// cti_msgs::DataArray cleanFuncTestReports;
// cleanFuncTestReports.header.stamp = ros::Time::now();
// cleanFuncTestReports.header.frame_id = "clean_function_test_report";
// cti_msgs::Data cleanFuncTestReport;

// //挡板状态数值
// cleanFuncTestReport.name = "damboard_state_value";
// cleanFuncTestReport.data = std::to_string(clean_function_test_state.recv_damboard_status);
// cleanFuncTestReport.type = cti_msgs::Data::TYPE_UINT8;
// cleanFuncTestReports.datas.push_back(cleanFuncTestReport);
// //挡板测试结果
// cleanFuncTestReport.name = "damboard_state_result";
// cleanFuncTestReport.type = cti_msgs::Data::TYPE_STRING;
// if (clean_function_test_state.recv_damboard_status == clean_function_test_state.damboard_status)
// {
//     cleanFuncTestReport.data = "PASS";
// }
// else
// {
//     cleanFuncTestReport.data = "NG";
// }
// cleanFuncTestReports.datas.push_back(cleanFuncTestReport);

// //边刷伸展
// cleanFuncTestReport.name = "sidebrush_transform_value";
// cleanFuncTestReport.data = std::to_string(clean_function_test_state.recv_side_brush_transform_state);
// cleanFuncTestReport.type = cti_msgs::Data::TYPE_UINT8;
// cleanFuncTestReports.datas.push_back(cleanFuncTestReport);
// //边刷伸展结果
// cleanFuncTestReport.name = "sidebrush_transform_result";
// cleanFuncTestReport.type = cti_msgs::Data::TYPE_STRING;
// if (abs(clean_function_test_state.recv_side_brush_transform_state - clean_function_test_state.side_brush_transform_state) <= 3)
// {
//     cleanFuncTestReport.data = "PASS";
// }
// else
// {
//     cleanFuncTestReport.data = "NG";
// }
// cleanFuncTestReports.datas.push_back(cleanFuncTestReport);

// //边刷转速
// cleanFuncTestReport.name = "side_brush_speed_value";
// cleanFuncTestReport.data = std::to_string(clean_function_test_state.recv_side_brush_speed);
// cleanFuncTestReport.type = cti_msgs::Data::TYPE_UINT8;
// cleanFuncTestReports.datas.push_back(cleanFuncTestReport);
// //边刷转速结果
// cleanFuncTestReport.name = "side_brush_speed_report";
// cleanFuncTestReport.type = cti_msgs::Data::TYPE_STRING;
// if (clean_function_test_state.recv_side_brush_speed == clean_function_test_state.side_brush_speed)
// {
//     cleanFuncTestReport.data = "PASS";
// }
// else
// {
//     cleanFuncTestReport.data = "NG";
// }
// cleanFuncTestReports.datas.push_back(cleanFuncTestReport);

// //喷水电机
// cleanFuncTestReport.name = "spray_motor_value";
// cleanFuncTestReport.data = std::to_string(clean_function_test_state.recv_spray_motor);
// cleanFuncTestReport.type = cti_msgs::Data::TYPE_UINT8;
// cleanFuncTestReports.datas.push_back(cleanFuncTestReport);
// ////喷水电机结果
// cleanFuncTestReport.name = "spray_motor_report";
// cleanFuncTestReport.type = cti_msgs::Data::TYPE_STRING;
// if (clean_function_test_state.recv_spray_motor == clean_function_test_state.spray_motor)
// {
//     cleanFuncTestReport.data = "PASS";
// }
// else
// {
//     cleanFuncTestReport.data = "NG";
// }
// cleanFuncTestReports.datas.push_back(cleanFuncTestReport);
// //发布测试结果
// clean_function_test_report_pub.publish(cleanFuncTestReports);
// //状态重置
// clean_function_test_state.reset();
