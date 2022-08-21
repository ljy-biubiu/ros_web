#include "mapsample/mapsample.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

#include "FTPClient.h"
#include "matToPng_base64.h"
// #include "ros2_packages_/ros2_packages_params_.h"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"

using namespace embeddedmz;

using std::placeholders::_1;


template <class T,class Y>
void SensorState<T,Y>::init() {
  if (!data_.topic.empty()) {
    if(this->getType() == "sensor_msgs::msg::PointCloud2")
    {
      sub_ = node_->template create_subscription<T>(
      data_.topic, rclcpp::QoS{1}.best_effort(),
      std::bind(&SensorState::callback, this, std::placeholders::_1));
    }
    else
    {
      sub_ = node_->template create_subscription<T>(
      data_.topic, rclcpp::QoS{1},
      std::bind(&SensorState::callback, this, std::placeholders::_1));
    }
  }
}


template <class T,class Y>
int SensorState<T, Y>::getState() {
  int stat = StateType::ERROR;
  if (data_.enable) {
    if (timer_.state) {
      if (timer_.getDuration(node_->now().seconds()) > data_.timeout) {
        stat = StateType::OVERTIME;
      } else {
        stat = StateType::NORMAL;
      }
    }
  } else {
    stat = StateType::NORMAL;
  }
  return stat;
}

template <class T,class Y>
std::string SensorState<T, Y>::getTopic() {
  return data_.topic;
}

template <class T,class Y>
std::string SensorState<T, Y>::getName() {
  return data_.name;
}

template <class T,class Y>
std::string SensorState<T, Y>::getType() {
  return data_.type;
}

template <class T,class Y>
float SensorState<T, Y>::getTimeout() {
  return data_.timeout;
}

template <class T,class Y>
bool SensorState<T, Y>::getEnable() {
  return data_.enable;
}

template <class T,class Y>
int SensorState<T, Y>::getMul() {
  return data_.mul;
}

template <class T,class Y>
void SensorState<T, Y>::callback(const DataTypeSharedPtr msg) {
  if (data_.mul == 0) {
    monitor_exist_lidar(data_.topic);
  }
  timer_.set();
}

template <class T,class Y>
void SensorState<T, Y>::monitor_exist_lidar(const std::string &str) {
  std::ofstream fout;
  YAML::Node config = YAML::LoadFile(topic_config_path_);
  fout.open(topic_config_path_, std::ios::out);
  if (!fout.is_open()) {
    std::cout << "can't open " + topic_config_path_ << std::endl;
  }

  fout << "topic_state:" << std::endl;

  for (int i = 0; config["topic_state"].size() > i; i++) {
    std::string value_enable{"true"};
    std::string value_mul{config["topic_state"][i]["mul"].as<std::string>()};
    if (config["topic_state"][i]["enable"].as<bool>() == 0) {
      value_enable = "false";
    }
    if (str == config["topic_state"][i]["topic"].as<std::string>()) {
      value_mul = "1";
    }

    fout << " "
         << " - { topic: \""
         << config["topic_state"][i]["topic"].as<std::string>()
         << "\", type: \"" << config["topic_state"][i]["type"].as<std::string>()
         << "\", name: \"" << config["topic_state"][i]["name"].as<std::string>()
         << "\", timeout: " << config["topic_state"][i]["timeout"].as<double>()
         << ", enable: " << value_enable << ", mul: " << value_mul << "}"
         << std::endl;
  }
  fout.close();
}


void CalibLidar::init()
{
  if (!sub_topic_.empty() && !pub_topic_.empty())
  {
    calibration_lidar_sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        sub_topic_, rclcpp::QoS{10}.transient_local(),
        std::bind(&CalibLidar::callback, this, std::placeholders::_1));
    calibration_lidar_pub = node_->create_publisher<sensor_msgs::msg::PointCloud>(
        pub_topic_, rclcpp::QoS{10});
  }
}

void CalibLidar::setState()
{
  rslidar_web_switch = !rslidar_web_switch;
  // rslidar_web_switch == 1 ? rslidar_web_switch = 0 : rslidar_web_switch = 1;
}

const std::string CalibLidar::getName()
{
  return name_;
}

const int CalibLidar::getRank()
{
  return rank_;
}

void CalibLidar::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::cout << " i am out name " << getName() << std::endl;
  if (rslidar_web_switch == 1)
  {
    std::cout << " i am in name " << getName() << std::endl;
    sensor_msgs::msg::PointCloud out_pointcloud;
    sensor_msgs::msg::PointCloud out_pointcloud_aft;
    geometry_msgs::msg::Point32 point_;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pointcloud);
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
    calibration_lidar_pub->publish(out_pointcloud_aft);
  }
}

MapSample::MapSample() : Node("cti_mapsample_node")
{
  init();
  run();
}

MapSample::~MapSample() {}

void MapSample::init()
{

  calibration_lidar_config_path_ = this->declare_parameter<std::string>("CALIBRATION_LIDAR_CONFIG_PATH", "");
  calibration_launch_path_ = this->declare_parameter<std::string>("calibration_launch_path", "");
  mapsample.startMappingCmd = this->declare_parameter<std::string>("startMappingCmd", "");
  mapsample.stopMappingCmd = this->declare_parameter<std::string>("stopMappingCmd", "");
  cti_topic_config_path_ =
      this->declare_parameter<std::string>("CTI_TOPIC_CONFIG_FILE_PATH", "");

  //获取参数
  CTI_SW_VER = "null";
  //订阅话题

  calibration_camera_sub = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera_calibrator/image", rclcpp::QoS{1},
      std::bind(&MapSample::calibrationCameraCallback, this, std::placeholders::_1));

  error_msg_sub = this->create_subscription<cti_msgs::msg::ErrorStatusArray>(
      "/cti/error_status/get", rclcpp::QoS{1},
      std::bind(&MapSample::error_msg_Callback, this, std::placeholders::_1));

  dustboxbat_state_sub =
      this->create_subscription<cti_msgs::msg::BatteryCellsState>( //none
          "/cti/chassis_serial/dustbox_batcell_state",
          rclcpp::QoS{1},
          std::bind(&MapSample::dustboxbat_state_Callback, this,
                    std::placeholders::_1));

  batcell_state_sub =
      this->create_subscription<cti_msgs::msg::BatteryCellsState>(
          "/cti/cti_fpga/batcell_state", rclcpp::QoS{1},
          std::bind(&MapSample::batcell_state_Callback, this,
                    std::placeholders::_1));

  Imu_statu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "/cti/fpga_serial/imu", rclcpp::QoS{1},
      std::bind(&MapSample::Imu_statu_Callback, this, std::placeholders::_1));

  Mapping_statu_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/mapping_status", rclcpp::QoS{1},
      std::bind(&MapSample::Mapping_statu_Callback, this,
                std::placeholders::_1));

  NavDeliveryResp_msg_sub =
      this->create_subscription<cti_rblite_msgs::msg::NavDeliveryResp>(
          "rblite/navdeliveryresp", rclcpp::QoS{1},
          std::bind(&MapSample::NavDeliveryResp_Callback, this,
                    std::placeholders::_1));

  box_msg_sub = this->create_subscription<cti_rblite_msgs::msg::BoxAskResponse>(
      "/cloud_scheduling_node/response/ask/boxinfo",
      rclcpp::QoS{1},
      std::bind(&MapSample::box_msg_Callback, this, std::placeholders::_1));

  node_alive_sub =
      this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>( ///been
          "/cti/monitor/node_alive", rclcpp::QoS{10},
          std::bind(&MapSample::node_alive_Callback, this,
                    std::placeholders::_1));

  ult1_state_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/cti/ultrasonic/data_1", rclcpp::QoS{10},
      std::bind(&MapSample::ult1_state_Callback, this, std::placeholders::_1));

  ult2_state_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/cti/ultrasonic/data_2", rclcpp::QoS{10},
      std::bind(&MapSample::ult2_state_Callback, this, std::placeholders::_1));

  ult0_state_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/cti/ultrasonic/data_0", rclcpp::QoS{10},
      std::bind(&MapSample::ult0_state_Callback, this, std::placeholders::_1));

  sensor_state_sub = this->create_subscription<cti_msgs::msg::BoxState>(
      "/cti/robot_config/sensorState", rclcpp::QoS{10},
      std::bind(&MapSample::senfor_state_Callback, this,
                std::placeholders::_1));

  webcmd_sub = this->create_subscription<std_msgs::msg::Int32>(
      "/cti/rosweb/cmd", rclcpp::QoS{10},
      std::bind(&MapSample::rosWebCmdCallback, this, std::placeholders::_1));

  status_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/mapping_status", rclcpp::QoS{10},
      std::bind(&MapSample::mappingStatusCallback, this,
                std::placeholders::_1));

  gps_sub = this->create_subscription<cti_msgs::msg::GnssRTK>(
      "/cti/chassis_serial/gnss", rclcpp::QoS{10},
      std::bind(&MapSample::gnnsLocationCallback, this, std::placeholders::_1));

  sensorState_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/cti/status_monitor/sensor_state", rclcpp::QoS{10},
      std::bind(&MapSample::sensorStateCallback, this, std::placeholders::_1));

  mapToJpg_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/cti/semantics/object_map/test_costmap",
      rclcpp::QoS{10},
      std::bind(&MapSample::mapToJpgCallback, this, std::placeholders::_1));

  parameter_server_sub = this->create_subscription<std_msgs::msg::String>(
      "/robot_config/parameter_server",
      rclcpp::QoS{10}.transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg)
      { parameterServerCallback(msg); });

  mapping_state_sub = this->create_subscription<std_msgs::msg::String>(
      "/platform_communication/buildmapcommand",
      rclcpp::QoS{10},
      [this](const std_msgs::msg::String::SharedPtr msg)
      { mappingCallback(msg); });

  calibration_lidar2_sub = this->create_subscription<std_msgs::msg::String>(
      "/cti/web/calibration_lidar2_cmd",
      rclcpp::QoS{2},
      [this](const std_msgs::msg::String::SharedPtr msg)
      { calibrationLidar2Callback(msg); });

  web_send_calibration_tf_launch = this->create_publisher<std_msgs::msg::String>(
      "/cti/web/tf_launch", rclcpp::QoS{1});
  web_tf_msg_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/rosweb/pub_web_tf_msg", rclcpp::QoS{1});
  web_tf_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/rosweb/pub_web_tf", rclcpp::QoS{1});
  webstate_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/rosweb/state", rclcpp::QoS{1});
  mapToPng_encodebase64_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/cti_mapsample/mapToPng_encodebase64_topic", rclcpp::QoS{1});
  sensor_status_web_pub = this->create_publisher<std_msgs::msg::String>(
      "cti/robot_config/sensorState_web", rclcpp::QoS{1});
  box_msg_web_pub = this->create_publisher<std_msgs::msg::String>(
      "cti/web/box_msg", rclcpp::QoS{1});
  NavDeliveryResp_web_pub = this->create_publisher<std_msgs::msg::String>(
      "cti/web/NavDeliveryResp", rclcpp::QoS{1}); /// TURE
  diary_web_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/web/diary", rclcpp::QoS{1}); /// TURE
  Imu_web_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/web/imu", rclcpp::QoS{1}); /// TURE
  vihicle_battery_web_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/web/vihicle_battery", rclcpp::QoS{1}); /// TURE
  dustbox_battery_web_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/web/dustbox_battery", rclcpp::QoS{1}); /// TURE
  odometer_web_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/web/odometer", rclcpp::QoS{1}); /// TURE
  nav_version_web_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/web/nav_version", rclcpp::QoS{1}); /// TURE
  error_publisher = this->create_publisher<cti_msgs::msg::ErrorStatus>(
      "/cti/error_status/set", rclcpp::QoS{20}); /// TURE
  web_timer_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/web/timer", rclcpp::QoS{1}); /// TURE
  check_tf_file_publisher = this->create_publisher<std_msgs::msg::Bool>(
      "/cti/web/tf_exit", rclcpp::QoS{1}); /// TURE
  mappingstate_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/rblite/navdeliveryresp", rclcpp::QoS{1});
  camera_image_encodebase64_pub = this->create_publisher<std_msgs::msg::String>(
      "/cti/web/calibration_camera_3_0", rclcpp::QoS{1}); /// TURE
  mappingstate_TEST_pub = this->create_publisher<std_msgs::msg::String>(
      "/platform_communication/buildmapcommand", rclcpp::QoS{1});

  //定时器
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                   [this]()
                                   { this->timercallback(); });

  //--读取参数--
  std::vector<TopicState> topicStates;
  getTopicStateParams(topicStates);
  
  sensor_states_.clear();
  for (auto &topicState : topicStates) {
    if (topicState.type == "sensor_msgs::msg::PointCloud2") {
      std::shared_ptr<StateBase> ptr =
          std::make_shared<SensorState<sensor_msgs::msg::PointCloud2,int>>(
              this, topicState, cti_topic_config_path_);
      sensor_states_.emplace_back(ptr);
    } 
    else if (topicState.type == "sensor_msgs::msg::LaserScan") {
      std::shared_ptr<StateBase> ptr =
          std::make_shared<SensorState<sensor_msgs::msg::LaserScan,int>>(
              this, topicState, cti_topic_config_path_);
      sensor_states_.emplace_back(ptr);
    } else if (topicState.type == "sensor_msgs::msg::Imu") {
      std::shared_ptr<StateBase> ptr =
          std::make_shared<SensorState<sensor_msgs::msg::Imu,int>>(
              this, topicState, cti_topic_config_path_);
      sensor_states_.emplace_back(ptr);
    } else if (topicState.type == "sensor_msgs::msg::Image") {
      std::shared_ptr<StateBase> ptr =
          std::make_shared<SensorState<sensor_msgs::msg::Image,int>>(
              this, topicState, cti_topic_config_path_);
      sensor_states_.emplace_back(ptr);
    } 
    else if (topicState.type == "nav_msgs::msg::Odometry") {
      std::shared_ptr<StateBase> ptr =
          std::make_shared<SensorState<nav_msgs::msg::Odometry,int>>(
              this, topicState, cti_topic_config_path_);
      sensor_states_.emplace_back(ptr);
    } 
    // else if (topicState.type == "tf2_msgs::msg::TFMessage") {
    //   std::shared_ptr<StateBase> ptr =
    //       std::make_shared<SensorState<tf2_msgs::msg::TFMessage>>(
    //           this, topicState, cti_topic_config_path_);
    //   sensor_states_.emplace_back(ptr);
    // }
    else{
      std::cout<<"cant find fit type for topic_state.yaml "<<std::endl;
    }
  }

  save_web_diary("------开机------");
}


//发布所有传感器状态信息
void MapSample::monitorMapBuildSensorState() {

  int sensor_state{0};

  for (const auto &state : sensor_states_) {
    if (state->getMul() == 0) {
      continue;
    }

    cti_msgs::msg::TabState tabstate;
    tabstate.status = state->getState();
    tabstate.name = state->getName();

    if (state->getState() == StateType::ERROR) {
      // RCLCPP_INFO_STREAM(
      //     get_logger(),
      //     "name: "+state->getName()+" no init  statu:no init getTopic: "+state->getTopic()+" getType: "+state->getType());
    } else if (state->getState() == StateType::OVERTIME) {
      // RCLCPP_INFO_STREAM(
      //     get_logger(),
      //     "name: "+state->getName()+" over time statu:over time");
    } else {
      sensor_state++;
    }
  }

  // std::cout<<"sensor_states_.size(); "<<sensor_states_.size()<<std::endl;
  if(sensor_state == sensor_states_.size())
  {
   data.sensor_state = true; 
  //  std::cout<<"data.sensor_state = true; "<<std::endl;
  }
  else
  {
    std::cout<<"data.sensor_state = false; "<<std::endl;
    data.sensor_state = false;
  }
  // data.sensor_state = true;
  
}


void MapSample::getTopicStateParams(std::vector<TopicState> &topicStates) {
  try {
    auto yaml_node = YAML::LoadFile(cti_topic_config_path_);
    auto configs = yaml_node["topic_state"];
    topicStates.clear();
    for (const auto &config : configs) {
      TopicState state;
      state.topic = config["topic"].as<std::string>();
      state.name = config["name"].as<std::string>();
      state.type = config["type"].as<std::string>();
      state.timeout = config["timeout"].as<double>();
      state.enable = config["enable"].as<bool>();
      state.mul = config["mul"].as<int>();
      topicStates.emplace_back(state);

    }

  } catch (const std::exception &ex) {
    RCLCPP_ERROR_STREAM(get_logger(), ex.what());
  }

  for(int i = 0 ; topicStates.size()>i ;i++ )
  {
    std::cout<<"------------------------"<<std::endl;
    std::cout<<topicStates.at(i).topic<<std::endl;
    std::cout<<topicStates.at(i).type<<std::endl;
  }
}

void MapSample::calibrationLidar2Callback(const std_msgs::msg::String::SharedPtr msg)
{
  if(msg->data == "start")
  {
    caliblidar2.startCalibLidar2();
    std::cout<<"startCalibLidar2"<<std::endl;
  }else if(msg->data == "close")
  {
    caliblidar2.stopCalibLidar2();
    std::cout<<"closeCalibLidar2"<<std::endl;
  }
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
    std_msgs::msg::Bool pub_data;
    read_file_path = home_str + "/cti-config/calibration/tf.launch.py";
    ifs.open(read_file_path);
    if (!ifs.is_open())
    {
      pub_data.data = 0;
      check_tf_file_publisher->publish(pub_data);
      ifs.close();
      return;
    }
    pub_data.data = 1;
    check_tf_file_publisher->publish(pub_data);
    ifs.close();
    return;
  }
  count--;
  if (count == 0)
  {
    count = 100;
  }
}

void MapSample::initCalibrationLidarParam()
{
  //读取需要校准的雷达信息 || 初始化雷达校准配置
  if (getLidarTopicStateParams())
  {
    for (auto child : calibLidarTopicStates)
    {
      auto calibLidar = std::make_shared<CalibLidar>(this,
                                                     child.pub_topic, child.sub_topic, child.name, child.rank);
      calibLidars.emplace_back(calibLidar);
    }
  }
}

void MapSample::parameterServerCallback(const std_msgs::msg::String::SharedPtr msg)
{
  static bool init_flag{false};
  if (init_flag == true)
  {
    return;
  }

  Json::Value parameter_server;
  Json::Reader reader;
  reader.parse(msg->data, parameter_server);
  vehicle_type = parameter_server["CTI_RUN_VER"].asString();

  //读取需要校准的雷达信息 || 初始化雷达校准配置
  initCalibrationLidarParam();
  init_flag = true;
  // std::cout << "读取需要校准的雷达信息 || 初始化雷达校准配置" << std::endl;
  // std::cout << vehicle_type << std::endl;
}

const bool MapSample::getLidarTopicStateParams()
{
  try
  {
    std::cout << "read calibration_lidar_config_path_ :" << calibration_lidar_config_path_ << std::endl;
    auto yaml_node = YAML::LoadFile(calibration_lidar_config_path_);

    //lidar data
    auto configs = yaml_node["topic_state"];
    calibLidarTopicStates.clear();
    for (const auto &config : configs)
    {
      CalibLidarTopicState state;
      state.sub_topic = config["sub_topic"].as<std::string>();
      state.pub_topic = config["pub_topic"].as<std::string>();
      state.name = config["name"].as<std::string>();
      state.rank = config["rank"].as<int>();
      calibLidarTopicStates.emplace_back(state);
    }
    //launch data
    auto configs_ = yaml_node["calibration_launch"];
    for (const auto &config : configs_)
    {

      if (vehicle_type == config["vehicle_type"].as<std::string>())
      {
        calibLidarLaunch = config["file"].as<std::string>();
      }
    }
  }
  catch (const std::exception &ex)
  {
    std::cout << "讀取校準雷達配置文件失敗！" << std::endl;
    RCLCPP_ERROR_STREAM(get_logger(), ex.what());
    return false;
  }
  return true;
}

void MapSample::run()
{
  std::thread dowork_thread(boost::bind(&MapSample::doWork, this));
  // rclcpp::spin(node_ftp_uploadMap);
  rclcpp::spin(this->get_node_base_interface());
  rclcpp::shutdown();
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
  rclcpp::Rate rate(0.1);
  int waitcnt = 0;
  while (rclcpp::ok())
  {
    //--delay--
    rate.sleep();
    if (upload.enable)
    {
      CFTPClient FTPClient([](const std::string &strLogMsg)
                           { std::cout << strLogMsg << std::endl; });

      if (FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp001",
                                "Ftp001@2020"))
      {
        upload.upLoadState = UPLOAD_STATE::UPLOAD_START;
        FTPClient.RemoveFile(upload.remotefile);
        bool upload_result =
            FTPClient.UploadFile(upload.localfile, upload.remotefile);
        std::cout << "uploading..." << std::endl;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (stat(upload.localfile.c_str(), &statbuf) == 0 && flag == false)
        {
          flag = true;
          local_file_size = statbuf.st_size;
          printf("local file size>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>%0.fKB\n",
                 local_file_size);
        }
        while (!bRes)
        {
          bRes = FTPClient.Info(upload.remotefile, ResFileInfo);
        }
        remote_file_size = ResFileInfo.dFileSize;
        old_remote_file_size = remote_file_size;
        int upload_percent = remote_file_size * 100 / local_file_size;
        data.message = "当前上传进度为： " +
                       boost::lexical_cast<std::string>(upload_percent) + "%";
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
        mapping_cmnc_type.command_state = "COMPLETED";
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
      sendAppBuildState(true);
    }
    else
    {
      waitcnt = 0;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////imu数据处理玉发布

////////////////////////////////////////////////////////////////////////////////////////////////线路发布
/// RoadEdgeArray

void MapSample::calibrationCameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  static int delay{2};
  static bool fir_flag{true};

  if(delay == 0 || fir_flag)
  {
    fir_flag = false;
    delay = 2;
    std_msgs::msg::String camera;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img__ = cv_ptr->image;
    cv::Mat shrink;
    cv::resize (img__, shrink, cv::Size (img__.cols/4, img__.rows/4));

    camera.data = "data:image/png;base64," + Mat2Base64(shrink, "png");
    camera_image_encodebase64_pub->publish(camera);
  }
  delay--;
}

void MapSample::robot_env_sub(const std_msgs::msg::String::SharedPtr msg)
{
  robot_env = msg->data;
}

void MapSample::pubErrorMsg(const std::string &modular_name,
                            const uint &error_code, const uint &error_level,
                            const std::string &error_msg)
{
  cti_msgs::msg::ErrorStatus error_msgs;
  error_msgs.stamp = this->now();
  ;
  error_msgs.module_name = modular_name;
  error_msgs.error_code = error_code;
  error_msgs.error_info = error_msg;
  error_msgs.level = error_level;
  error_publisher->publish(error_msgs);
}

void MapSample::openAndsendtf_luanch()
{

  ifstream ifs;
  std::string read_file_path;
  std_msgs::msg::String pub_data;
  const std::string home_str = std::string(std::getenv("HOME"));
  read_file_path = home_str + "/cti-config/calibration/tf.launch.py";
  std::cout << "Read file path: " << read_file_path << std::endl;
  ifs.open(read_file_path);
  if (!ifs.is_open())
  {
    pub_data.data = " 问题巨大，没有找到tf.launch文件 ";
    web_send_calibration_tf_launch->publish(pub_data);
    std::cout << " 问题巨大，没有找到tf.launch文件 " << std::endl;
    return;
  }

  std::string str((std::istreambuf_iterator<char>(ifs)),
                  std::istreambuf_iterator<char>());
  pub_data.data = str;
  web_send_calibration_tf_launch->publish(pub_data);

  ifs.close();
}

///////////////////////////////////////////////////////////////////////

const std::string MapSample::merge_nameAndmodule(
    const cti_msgs::msg::ErrorStatus msg)
{
  std::string tranlation_error;
  if (msg.error_code < 10)
  {
    tranlation_error =
        msg.module_name + "000" + to_string(msg.error_code) + "E";
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
  read_file_path = ament_index_cpp::get_package_share_directory("ctiwww") +
                   "/www/js/diary/demo.json";
  //read_file_path = "/home/ljy/ros2_ws/src/cti_data_viewer/cti_www/www/js/diary/demo.json";

  //std::cout << "Read file path: " << read_file_path << std::endl;
  ifs.open(read_file_path);
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
    // std::cout<<root[a]["error_code"].asInt()<<std::endl;
  }

  return diary_data_down;
}

void writeFileJson(std::queue<DIARY_JSON> &diary_data_down,
                   const int &record_numb)
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
  }

  //缩进输出
  Json::StyledWriter sw;
  //输出到文件
  ofstream os;
  std::string write_file_path;
  // write_file_path = ament_index_cpp::get_package_share_directory("ctiwww") +
  //                   "/www/js/diary/demo.json";
  write_file_path = "/home/ljy/ros2_ws/src/cti_data_viewer/cti_www/www/js/diary/demo.json";
  //std::cout << "Write file path: " << write_file_path << std::endl;
  os.open(write_file_path);
  os << sw.write(root);
  os.close();
}

void MapSample::send_web_diary(std::queue<DIARY_JSON> diary_data_down)
{
  Json::Value root;
  Json::FastWriter writer;
  std_msgs::msg::String web_msg;
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
  diary_web_pub->publish(web_msg);
}

void MapSample::save_web_diary(const std::string &msg,
                               const std::string &msgT)
{
  std::string time = boost::lexical_cast<std::string>(
      boost::posix_time::second_clock::local_time());
  std::queue<DIARY_JSON> diary_data;
  DIARY_JSON diary_json;
  diary_json.message = msg;
  diary_json.time = time;
  diary_json.error_code = msgT;
  int record_numb{600};
  diary_data = readFileJson();
  diary_data.push(diary_json);
  writeFileJson(diary_data, record_numb);
  send_web_diary(diary_data);
}

void MapSample::dynamic_save_web_diary(
    const cti_msgs::msg::ErrorStatusArray msg)
{
  std::string time = boost::lexical_cast<std::string>(
      boost::posix_time::second_clock::local_time());
  std::queue<DIARY_JSON> diary_data;
  DIARY_JSON diary_json;
  diary_data = readFileJson();
  for (auto msg_err_code : msg.data)
  {
    diary_json.message = "信息：" + msg_err_code.error_info + "      优先级：" +
                         to_string(msg_err_code.level);
    diary_json.time = time;
    diary_json.error_code = merge_nameAndmodule(msg_err_code);
    diary_data.push(diary_json);
  }
  send_web_diary(diary_data);
}

//
void MapSample::error_msg_Callback(
    const cti_msgs::msg::ErrorStatusArray::SharedPtr msg) //错误码收发
{
  tran_Err_code.data.clear();             //第一次清空处理空间
  tran_Err_code.data = cur_Err_code.data; //将现存数据放入处理空间中

  for (auto err_msg :
       msg->data) // 检测是否有新的数据传入，有则放入cur_Err_code
  {
    if (msg->data[0].module_name == "END")
      break;
    bool exsit_flag{false};
    for (auto tran_err_code : tran_Err_code.data)
    {
      if (tran_err_code.module_name == err_msg.module_name &&
          tran_err_code.error_code == err_msg.error_code &&
          tran_err_code.error_info == err_msg.error_info)
      {
        exsit_flag = true;
      }
    }
    if (exsit_flag == false)
    {
      cur_Err_code.data.push_back(err_msg);
    }
  }

  tran_Err_code.data.clear();             //第二次清空处理空间
  tran_Err_code.data = cur_Err_code.data; //将现存数据放入处理空间中
  cur_Err_code.data.clear();              //清除当前存储容器，等待处理数据放入
  // relea_Err_code.data.clear();

  for (auto tran_err_code :
       tran_Err_code.data) //将恢复正常的数据，释放，并存在静态日志内
  {
    bool exsit_flag{false};
    for (auto err_msg : msg->data)
    {
      if (msg->data[0].module_name == "END")
        break;
      if (tran_err_code.module_name == err_msg.module_name &&
          tran_err_code.error_code == err_msg.error_code &&
          tran_err_code.error_info == err_msg.error_info)
      {
        exsit_flag = true;
      }
    }
    if (exsit_flag == true) //存在相同的，则放回当前存储容器
    {
      cur_Err_code.data.push_back(tran_err_code);
    }
    else //不存在，则放到存在静态日志.,
    {
      save_web_diary("信息：" + tran_err_code.error_info + "    优先级：" +
                         to_string(tran_err_code.level),
                     merge_nameAndmodule(tran_err_code));
      save_web_diary("信息：恢复正常", merge_nameAndmodule(tran_err_code));
    }
  }

  tran_Err_code.data.clear();             //第三次清空处理空间
  tran_Err_code.data = cur_Err_code.data; //将现存数据放入处理空间中
  rank_Err_code.data.clear();

  for (int rank = 0; rank <= 15;
       rank++) //将数据按危害等级排序，并发布到动态数据
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
  err_monitor_timer_ = this->now().seconds();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
///箱子电量
void MapSample::dustboxbat_state_Callback(
    const cti_msgs::msg::BatteryCellsState::SharedPtr msg)
{
  Json::Value arrayObj_top;
  Json::FastWriter writer;
  std_msgs::msg::String web_msg;
  Json::Value item;
  int order = 1;

  item["bat_volt"] = Json::Value(msg->volt_all);
  item["bat_soc"] = Json::Value(msg->soc_all);
  item["name"] = Json::Value("总电池");
  arrayObj_top.append(item);

  for (auto bat : msg->batcells)
  {
    item["bat_volt"] = Json::Value(bat.bat_volt);
    item["bat_soc"] = Json::Value(bat.bat_soc);
    item["name"] = Json::Value("分电池" + std::to_string(order) + "号");
    arrayObj_top.append(item);
    order++;
  }

  web_msg.data = writer.write(arrayObj_top);
  dustbox_battery_web_pub->publish(web_msg);
  order = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
///地盘电量
void MapSample::batcell_state_Callback(
    const cti_msgs::msg::BatteryCellsState::SharedPtr msg)
{
  Json::Value arrayObj_top;
  Json::FastWriter writer;
  std_msgs::msg::String web_msg;
  Json::Value item;
  int order = 1;

  item["bat_volt"] = Json::Value(msg->volt_all);
  item["bat_soc"] = Json::Value(msg->soc_all);
  item["name"] = Json::Value("总电池");
  arrayObj_top.append(item);

  for (auto bat : msg->batcells)
  {
    item["bat_volt"] = Json::Value(bat.bat_volt);
    item["bat_soc"] = Json::Value(bat.bat_soc);
    item["name"] = Json::Value("分电池" + std::to_string(order) + "号");
    arrayObj_top.append(item);
    order++;
    //std::cout<<"---------"<<std::endl;
  }

  item["bat_volt"] = Json::Value(msg->bat_backup_volt);
  item["bat_soc"] = Json::Value("无");
  item["name"] = Json::Value("备用电池");
  arrayObj_top.append(item);

  web_msg.data = writer.write(item);
  Imu_web_pub->publish(web_msg);
  vihicle_battery_web_pub->publish(web_msg);
  order = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////

void MapSample::Imu_statu_Callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  Quaternion quaternion;
  Coordination_pose Coordin_pose;
  Json::FastWriter writer;
  Json::Value item;
  std_msgs::msg::String web_msg;

  quaternion.x = msg->orientation.x;
  quaternion.y = msg->orientation.y;
  quaternion.z = msg->orientation.z;
  quaternion.w = msg->orientation.w;

  Coordin_pose.angl = ToEulerAngles(quaternion);

  double pitch = Coordin_pose.angl.pitch * 180 / 3.14;
  double roll = Coordin_pose.angl.roll * 180 / 3.14;
  double yaw = Coordin_pose.angl.yaw * 180 / 3.14;

  item["pitch"] = Json::Value(pitch);
  item["roll"] = Json::Value(roll);
  item["yaw"] = Json::Value(yaw);

  web_msg.data = writer.write(item);
  Imu_web_pub->publish(web_msg);
}

/////////////////////////////////////////////////////////////////////////////////////前方施工

//////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////

void MapSample::box_msg_Callback(
    const cti_rblite_msgs::msg::BoxAskResponse::SharedPtr msg)
{
  Json::Value arrayObj_top;
  Json::FastWriter writer;
  Json::Value item;
  std_msgs::msg::String web_msg;

  for (auto state : msg->infos)
  {
    item["qr"] = Json::Value(state.qr);
    item["hive_type"] = Json::Value(state.hive_type);
    item["position"] = Json::Value(state.position);
    item["hive_device_type"] = Json::Value(state.hive_device_type);
    arrayObj_top.append(item);
  }
  web_msg.data = writer.write(arrayObj_top);
  box_msg_web_pub->publish(web_msg);
}

///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////以下超声波超时日志
void MapSample::ult1_state_Callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  static double last_time = this->now().seconds();
  if (this->now().seconds() - last_time > 1)
  {
    static double last_time_record = this->now().seconds();
    static int flag = 1;
    if (this->now().seconds() - last_time_record > 300 || flag == 1)
    {
      std::string alt_error_diary = "超声波模块1超时";
      pubErrorMsg("SENSOR", 3, 11, alt_error_diary);
      flag = 0;
      last_time_record = this->now().seconds();
    }
  }
  last_time = this->now().seconds();
}

void MapSample::ult2_state_Callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  static double last_time = this->now().seconds();
  if (this->now().seconds() - last_time > 1)
  {
    static double last_time_record = this->now().seconds();
    static int flag = 1;
    if (this->now().seconds() - last_time_record > 300 || flag == 1)
    {
      std::string alt_error_diary = "超声波模块2超时";
      pubErrorMsg("SENSOR", 3, 11, alt_error_diary);
      flag = 0;
      last_time_record = this->now().seconds();
    }
  }
  last_time = this->now().seconds();
}

void MapSample::ult0_state_Callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  static double last_time = this->now().seconds();
  if (this->now().seconds() - last_time > 1)
  {
    static double last_time_record = this->now().seconds();
    static int flag = 1;
    if (this->now().seconds() - last_time_record > 300 || flag == 1)
    {
      std::string alt_error_diary = "超声波模块0超时";
      pubErrorMsg("SENSOR", 3, 11, alt_error_diary);
      flag = 0;
      last_time_record = this->now().seconds();
    }
  }
  last_time = this->now().seconds();
}

///////////////////////////////////////////////////////////////////////////////////////////////以上是超声波超时日志

//////////////////////////////////////////////////////////////////////////////////////////////以下是节点日志

void MapSample::node_alive_Callback(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  static diagnostic_msgs::msg::DiagnosticArray last_msg = *msg;
  static int flag{0};
  static int i{0};
  for (auto obj : msg->status)
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
  last_msg = *msg;
  flag = 1;
  i = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////以上是节点日志

void MapSample::senfor_state_Callback(
    const cti_msgs::msg::BoxState::SharedPtr msg)
{
  Json::Value arrayObj_top;
  Json::FastWriter writer;
  Json::Value item;
  std_msgs::msg::String web_msg;
  static std::string last_data[20];
  static int flag{0};
  int i{0};

  for (auto state : msg->states)
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
    save_web_diary(state.name + state.message);
    last_data[i] = state.message;
    i++;
  }
  flag = 1;
  web_msg.data = writer.write(arrayObj_top);
  sensor_status_web_pub->publish(web_msg);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///日志系统

//////////////////////////////////////////////////////////////////////////////////////前方施工
void MapSample::mapToJpgCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  int height, width;
  height = msg->info.height;
  width = msg->info.width;

  std::string name;

  std_msgs::msg::String png_encode64_rosmsg;
  std::vector<cti_msgs::msg::TabState> coordinations;

  Json::Value arrayObj_top;
  Json::FastWriter writer;
  Json::Value item;
  std_msgs::msg::String web_msg;

  //构造一个H行W列的矩阵
  cv::Mat dst = cv::Mat(height, width, CV_8UC3);

  static int sum_j_w = 0, sum_i_h = 0, count_sum = 0;

  // ROS中的地图是串行数据，要将数据传输到Mat矩阵中
  //显示代价地图123456

  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    { //////车子
      if (msg->data[i * width + j] == -1)
      {
        dst.at<cv::Vec3b>(height - i - 1, j)[0] = 0;
        dst.at<cv::Vec3b>(height - i - 1, j)[1] = 238;
        dst.at<cv::Vec3b>(height - i - 1, j)[2] = 238;
      }
      else if (msg->data[i * width + j] == 0)
      {
        dst.at<cv::Vec3b>(height - i - 1, j)[0] = 220;
        dst.at<cv::Vec3b>(height - i - 1, j)[1] = 220;
        dst.at<cv::Vec3b>(height - i - 1, j)[2] = 220;
      }
      else if (msg->data[i * width + j] == -5) /////     红路线
      {
        dst.at<cv::Vec3b>(height - i - 1, j)[0] = 0;
        dst.at<cv::Vec3b>(height - i - 1, j)[1] = 0;
        dst.at<cv::Vec3b>(height - i - 1, j)[2] = 255;
      }
      else if (msg->data[i * width + j] == -15 ||
               msg->data[i * width + j] ==
                   -100) /////      车子坐标 及 路线  //绿色 局部路线
      {
        dst.at<cv::Vec3b>(height - i - 1, j)[0] = 0;
        dst.at<cv::Vec3b>(height - i - 1, j)[1] = 255;
        dst.at<cv::Vec3b>(height - i - 1, j)[2] = 0;
      }
      else if (msg->data[i * width + j] == -50) /////动态 //红色 全局路线
      {
        dst.at<cv::Vec3b>(height - i - 1, j)[0] = 255;
        dst.at<cv::Vec3b>(height - i - 1, j)[1] = 0;
        dst.at<cv::Vec3b>(height - i - 1, j)[2] = 0;
      }
      else if (msg->data[i * width + j] == 50) /////车上灰点
      {
        dst.at<cv::Vec3b>(height - i - 1, j)[0] = 220;
        dst.at<cv::Vec3b>(height - i - 1, j)[1] = 220;
        dst.at<cv::Vec3b>(height - i - 1, j)[2] = 220;
      }
      else if (msg->data[i * width + j] == 88 ||
               msg->data[i * width + j] == 98 ||
               msg->data[i * width + j] == 91 ||
               msg->data[i * width + j] == 81) /////静态障碍物 黑边
      {
        dst.at<cv::Vec3b>(height - i - 1, j)[0] = 0;
        dst.at<cv::Vec3b>(height - i - 1, j)[1] = 0;
        dst.at<cv::Vec3b>(height - i - 1, j)[2] = 0;
      }
      else if (msg->data[i * width + j] == 100) ///// 路边线  紫色
      {
        dst.at<cv::Vec3b>(height - i - 1, j)[0] = 150;
        dst.at<cv::Vec3b>(height - i - 1, j)[1] = 0;
        dst.at<cv::Vec3b>(height - i - 1, j)[2] = 200;
      }
      else if (msg->data[i * width + j] == -2) ///// 车头显示
      {
        dst.at<cv::Vec3b>(height - i - 1, j)[0] = 150;
        dst.at<cv::Vec3b>(height - i - 1, j)[1] = 238;
        dst.at<cv::Vec3b>(height - i - 1, j)[2] = 238;
      }
    }
  }

  //地图翻转270度
  dst = matRotateClockWise270(dst);
  png_encode64_rosmsg.data = "data:image/png;base64," + Mat2Base64(dst,
                                                                   "png");
  mapToPng_encodebase64_pub->publish(png_encode64_rosmsg);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////前方施工

void MapSample::sendAppBuildState(bool trigger)
{

  static double start_time{0};

  if (((this->now().seconds() - start_time) > 10) || trigger)
  {
    if (mapsample.MapSampleState == MAPSAMPLE_STOP)
    {
      data.mapping_state = false;
    }
    //--
    if(!trigger)
    {
      if(mapping_cmnc_type.msg_id.empty() || mapping_cmnc_type.command_state == "COMPLETED"|| mapping_cmnc_type.command_state == "FAILED")
      {
        return;
      }
    }

    data.whole_message = data.message + (data.sensor_state ? " 传感器状态：正常 " : " 传感器状态：异常 ");
    
    if(!trigger)
    {
      if(!data.sensor_state)
      {
        mapping_cmnc_type.command_state = "FAILED";
        std::cout<<" data.sensor_state : false "<<std::endl;
      }
    }

    std_msgs::msg::String msg;
    Json::Value root;
    Json::FastWriter writer;

    root["command_mode"] = "app";
    root["msg_id"] = mapping_cmnc_type.msg_id;
    root["command_state"] = mapping_cmnc_type.command_state;
    root["command_type"] = mapping_cmnc_type.command_type;
    root["info"] = data.whole_message;
    root["message"] = data.sensor_state ? "传感器状态：正常" : "传感器状态：异常";
    msg.data = writer.write(root);

    mappingstate_pub->publish(msg);
    start_time = this->now().seconds();

    if(!trigger)
    {
      if(!data.sensor_state)
      {
        mapsample.stopMapping();
        exit(0); //异常重启程序
      }
    }

  }
}

//////////////////////////////////////////////////////////////////////////////////////

void MapSample::Check_Mapping_State()
{
  if (Mappint_last_time != -1)
  {
    if (this->now().seconds() - Mappint_last_time > 3)
    {
      if (data.message != "建图异常，请处理...")
      {
        save_web_diary("建图异常，请处理...");
      }
      data.message = "建图异常，请处理...";
      mapping_cmnc_type.command_state = "FAILED";
    }
    else
    {
      data.message = "初始化成功，建图正在进行中...";
      mapping_cmnc_type.command_state = "IN_PROGRESS";
      Mappint_last_time = this->now().seconds();
    }
  }
}

void MapSample::mappingCallback(const std_msgs::msg::String::SharedPtr msg)
{
  Json::Value mapping_state;
  Json::Reader reader;
  reader.parse(msg->data, mapping_state);
  static std::string start_build_msg_id{""};
  static std::string end_build_msg_id{""};

  std::cout << "建图指令 : " << mapping_state["header"]["command_type"].asString() << std::endl;
  
  mapping_cmnc_type.msg_id = mapping_state["header"]["msg_id"].asString();             //id
  mapping_cmnc_type.command_type = mapping_state["header"]["command_type"].asString(); //指令类型

  if (mapping_cmnc_type.command_type == "BUILD_MAP_START")
  {
    flag_Mapping_start = true;
    start_build_msg_id = mapping_cmnc_type.msg_id;
    if (!upload.enable)
    {
      if (mapsample.startMapping())
      {
        data.message = "现在开始建图,正在初始化...";
        mapping_cmnc_type.command_state = "IN_PROGRESS";
      }
      else
      {
        data.message = "指令文件不存在...";
        mapping_cmnc_type.command_state = "FAILED";
      }
    }
    else
    {
      data.message = "数据在上传中，无法开始建图...";
      mapping_cmnc_type.command_state = "FAILED";
    }
  }
  else if (mapping_cmnc_type.command_type == "BUILD_MAP_END")
  {
    Mappint_last_time = -1;
    flag_Mapping_start = false;
    end_build_msg_id = mapping_cmnc_type.msg_id;

    if (!upload.enable)
    {
      data.message = "建图完成！";
      mapping_cmnc_type.command_state = "COMPLETED";
      mapping_cmnc_type.msg_id = start_build_msg_id;
      sendAppBuildState(true);
      
      if (mapsample.MapSampleState == MAPSAMPLE_START)
      {
        if (mapsample.stopMapping())
        {
          data.message = "结束建图，数据处理完成...";
          mapping_cmnc_type.command_state = "COMPLETED";
          mapping_cmnc_type.msg_id = end_build_msg_id;
        }
        else
        {
          data.message = "指令文件不存在...";
          mapping_cmnc_type.command_state = "FAILED";
        }
      }
      else
      {
        data.message = "还未开始建图，无法结束建图...";
        mapping_cmnc_type.command_state = "FAILED";
      }
    }
    else
    {
      data.message = "数据在上传中，不支持该功能...";
      mapping_cmnc_type.command_state = "FAILED";
    }
  }
  else if (mapping_cmnc_type.command_type == "BUILD_MAP_UPLOAD")
  {
    if (upload.upload(mapsample.pathfile))
    {
      data.message = "数据在上传中,请等待...";
      mapping_cmnc_type.command_state = "IN_PROGRESS";
    }
    else
    {
      data.message = "文件\"" + mapsample.pathfile + "\"不存在...";
      mapping_cmnc_type.command_state = "FAILED";
    }
  }
  else
  {
    std::cout << "建图指令无法识别" << std::endl;
    mapping_cmnc_type.command_state = "FAILED";
  }
  sendAppBuildState(true);
}

void MapSample::Mapping_statu_Callback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
  if (flag_Mapping_start == true)
  {
    Mappint_last_time = this->now().seconds();
  }
}

/////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////

void MapSample::NavDeliveryResp_Callback(
    const cti_rblite_msgs::msg::NavDeliveryResp::SharedPtr msg)
{
  Json::Value arrayObj_top;
  Json::FastWriter writer;
  Json::Value item;
  std_msgs::msg::String web_msg;
  int a = msg->code;

  item["message"] = Json::Value(msg->message);
  item["code"] = Json::Value(a);
  item["command_state"] = Json::Value(msg->nav_header.command_state);
  item["command_type"] = Json::Value(msg->nav_header.command_type);
  arrayObj_top.append(item);
  web_msg.data = writer.write(arrayObj_top);
  NavDeliveryResp_web_pub->publish(web_msg);
}

///////////////////////////////////////////////////////////////////////////////////////
void MapSample::rosWebCmdCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::cout << "web 接收：" << msg->data << std::endl;
  if (70 > msg->data && msg->data >= 50) //lidr显示开关
  {

    for (auto state : calibLidars)
    {
      if (state->getRank() == msg->data)
      {
        state->setState();
        std::cout << "-------------------" << state->getName() << " switch -------------------" << std::endl;
      }
    }
  }
  else if (msg->data == 70)
  {
    if (calibLidarLaunch == "")
    {
      std::cout << "无校准启动文件，请检查yaml文件是否有配置！" << std::endl;
      return;
    }

    std::string cmd = "ros2 launch " + calibration_launch_path_ + calibLidarLaunch;
    std::thread new_thread(
        &MapSample::thread_web_lidar_calibration, this, cmd); //子线程
    new_thread.detach();
    std::cout << "switch on lidar calibrate_node" + calibLidarLaunch + " launch  " << std::endl;
  }
  else if (msg->data == 90)
  {
    std::string cmd_kill_process = {"ps -ef | grep " + calibLidarLaunch + " | awk '{print $2}' | xargs kill -9"};
    system(cmd_kill_process.c_str());
    std::cout << "----------------------- try to kill calibrate node --------------------------------" << std::endl;
  }
  else if (msg->data == 91)
  {
    openAndsendtf_luanch();
    std::cout << "------------------------------- open tf.launch --------------------------------" << std::endl;
  }
}

void MapSample::mappingStatusCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
  data.mapping_state = msg->data;
}

void MapSample::thread_web_lidar_calibration(const std::string &msg)
{
  system(msg.c_str());
  // system("rosnode kill calibration");
}

void MapSample::sensorStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  //data.sensor_state = msg->data;
}

void MapSample::err_monitor() //当无err回调时，清除所有异常,设定时间为20秒
{
  if (err_end_flag)
  {
    if (this->now().seconds() - err_monitor_timer_ > 20)
    {
      pubErrorMsg("END", 0, 0, "NONE");
      err_end_flag = false;
      err_monitor_timer_ = 0;
    }
  }
}

void MapSample::gnnsLocationCallback(
    const cti_msgs::msg::GnssRTK::SharedPtr nav_satfix)
{
  data.lat = nav_satfix->lat;
  data.lon = nav_satfix->lon;
  data.alt = nav_satfix->alt;
  data.lat_err = nav_satfix->lat_err;
  data.lon_err = nav_satfix->lon_err;
  data.sats_used = nav_satfix->sats_used;

  data.sensor_state = true;
}

void MapSample::pub_nav_version()
{
  std_msgs::msg::String Data;
  Data.data = CTI_SW_VER;
  nav_version_web_pub->publish(Data);
}

void MapSample::pub_webTimer()
{
  std_msgs::msg::String pose;
  web_timer_pub->publish(pose);
}

void MapSample::timercallback()
{
  monitorMapBuildSensorState();
  sendAppBuildState();
  Check_Mapping_State();
  pub_nav_version();
  err_monitor();
  pub_webTimer();
  check_tf_exit_luanch();
}
