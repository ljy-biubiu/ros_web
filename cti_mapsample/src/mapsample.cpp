#include "mapsample/mapsample.h"
#include "FTPClient.h"
#include <jsoncpp/json/json.h>

using namespace embeddedmz;

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
    std::string gpsTopic;
    std::string status_topic;
    std::string sensor_state_topic;
    std::string roswebCmd_topic;
    std::string roswebState_topic;
    pnh_.param<std::string>("gpsTopic", gpsTopic, "/ublox_gps/fix");
    pnh_.param<std::string>("startMappingCmd", mapsample.startMappingCmd, "");
    pnh_.param<std::string>("stopMappingCmd", mapsample.stopMappingCmd, "");
    pnh_.param<std::string>("mapping_status_topic", status_topic, "/mapping_status");
    pnh_.param<std::string>("sensor_state_topic", sensor_state_topic, "/cti/status_monitor/sensor_state");
    pnh_.param<std::string>("rosweb_cmd_topic", roswebCmd_topic, "/cti/rosweb/cmd");
    pnh_.param<std::string>("rosweb_state_topic", roswebState_topic,"/cti/rosweb/state");

    webstate_pub = nh.advertise<std_msgs::String>(roswebState_topic,1);
    webcmd_sub = nh.subscribe(roswebCmd_topic, 10,&MapSample::rosWebCmdCallback, this);
    status_sub = nh.subscribe(status_topic, 10,&MapSample::mappingStatusCallback, this);
    gps_sub = nh.subscribe(gpsTopic, 10, &MapSample::gnnsLocationCallback, this);
    sensorState_sub = nh.subscribe(sensor_state_topic, 10, &MapSample::sensorStateCallback, this);
    timer = nh.createTimer(ros::Duration(0.5), &MapSample::timercallback,this);
}

void MapSample::run()
{
    std::thread dowork_thread(boost::bind(&MapSample::doWork, this));
    ros::spin();
}

void MapSample::doWork()
{
    int waitcnt=0;
    while(ros::ok())
    {
        //--delay--
        ros::Duration(0.1).sleep();
        if(upload.enable)
        {
            CFTPClient FTPClient([](const std::string& strLogMsg) {
                  std::cout << strLogMsg << std::endl;
                });
            if(FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp001", "Ftp001@2020"))
            {
                upload.upLoadState = UPLOAD_STATE::UPLOAD_START;
                FTPClient.RemoveFile(upload.remotefile);
                bool upload_result = FTPClient.UploadFile(upload.localfile, upload.remotefile);
                std::cout << "uploading..." << std::endl;
                if(upload_result){
                    upload.upLoadState = UPLOAD_STATE::UPLOAD_SUCCESS;
                    std::cout << "success" << std::endl;
                }else{
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
            if(waitcnt > 20)
            {
                upload.upLoadState = UPLOAD_FAILED;
                upload.enable = false;
                std::cout << "upload failed" << std::endl;
            }
            sendWebDataState(true);
        }else{
            waitcnt = 0;
        }
    }
}

void MapSample::sendWebDataState(bool trigger)
{
    static double start_time;
    if(((ros::Time::now().toSec() - start_time) > 0.5) || trigger)
    {
        if(mapsample.MapSampleState == MAPSAMPLE_STOP)
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

void MapSample::rosWebCmdCallback(const std_msgs::Int32 &msg)
{
    std::cout << "web 接收：" << msg.data << std::endl;
    if(msg.data == 1)
    {
        if(!upload.enable)
        {

            if(mapsample.startMapping())
            {
                data.message = "现在开始建图...";
            }
            else
            {
                data.message = "指令文件不存在...";
            }
        }
        else
        {
            data.message = "数据在上传中，无法开始建图...";
        }
    }
    else if(msg.data == 2)
    {
        if(!upload.enable)
        {
            if(mapsample.MapSampleState == MAPSAMPLE_START)
            {

                if(mapsample.stopMapping())
                {
                    data.message = "结束建图，数据处理完成...";
                }
                else
                {
                    data.message = "指令文件不存在...";
                }
            }
            else
            {
                data.message = "还未开始建图，无法结束建图...";
            }
        }
        else
        {
            data.message = "数据在上传中，不支持该功能...";
        }
    }
    else if(msg.data == 3)
    {
        if(upload.upload(mapsample.pathfile))
        {
            data.message = "数据在上传中,请等待...";
        }
        else
        {
            data.message = "文件\"" + mapsample.pathfile + "\"不存在...";
        }
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

void MapSample::gnnsLocationCallback(const cti_msgs::GnssRTKConstPtr &nav_satfix)
{
    data.lat = nav_satfix->lat;
    data.lon = nav_satfix->lon;
    data.alt = nav_satfix->alt;
    data.lat_err = nav_satfix->lat_err;
    data.lon_err = nav_satfix->lon_err;
    data.sats_used = nav_satfix->sats_used;
}

void MapSample::timercallback(const ros::TimerEvent&)
{
    sendWebDataState();
}
