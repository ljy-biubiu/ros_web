#ifndef __MY_CLOUD_H__
#define __MY_CLOUD_H__

#include <cv_bridge/cv_bridge.h> //cv_bridge
#include <jsoncpp/json/json.h>
#include <math.h>

#include <yaml-cpp/yaml.h>

#include <boost/cast.hpp>
#include <boost/date_time.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <cstdlib>
#include <cti_msgs/msg/gnss_rtk.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <queue>
#include <regex>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <tf2_msgs/msg/tfmessage.hpp>
#include <set>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <streambuf>
#include <string>
#include <thread>
#include <typeinfo>

#include "boost/shared_ptr.hpp"
#include "cti_msgs/msg/battery_cells_state.hpp"
#include "cti_msgs/msg/box_state.hpp"
#include "cti_msgs/msg/error_status.hpp"
#include "cti_msgs/msg/error_status_array.hpp"
#include "cti_msgs/msg/tab_state.hpp"
#include "cti_msgs/msg/vehicle_ctl_run_info.hpp"
#include "cti_rblite_msgs/msg/box_ask_response.hpp"
#include "cti_rblite_msgs/msg/box_info.hpp"
#include "cti_rblite_msgs/msg/nav_delivery_resp.hpp"
#include "cti_rblite_msgs/msg/nav_header.hpp"
#include "cti_vector_map_msgs/msg/clear_area.hpp"
#include "cti_vector_map_msgs/msg/road_edge_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"

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

enum StateType
{
  ERROR = -1,   //错误，开机未读取到数据
  NORMAL = 0,   //正常
  OVERTIME = 1, //超时
};

struct TopicState
{
  std::string name;
  std::string topic;
  std::string type;
  float timeout{0};
  bool enable{false};
  int mul{0};
};

class StateBase
{
  struct StateTimer
  {
    explicit StateTimer(rclcpp::Node *node_) : node__(node_)
    {
    }
    // rclcpp::Time start_time = clock.now();

    void set()
    {
      state = true;
      start_timer = node__->now().seconds();
    }

    [[nodiscard]] double getDuration(double now) const
    {
      return now - start_timer;
    }
    bool state{false};
    double start_timer{0};
    rclcpp::Node *node__;
  };

public:
  explicit StateBase(rclcpp::Node *node)
      : node_(node), timer_(node) {}
  virtual ~StateBase() {}
  virtual void init() = 0;
  virtual int getState() = 0;
  virtual std::string getTopic() = 0;
  virtual std::string getName() = 0;
  virtual std::string getType() = 0;
  virtual float getTimeout() = 0;
  virtual bool getEnable() = 0;
  virtual int getMul() = 0;
  virtual void monitor_exist_lidar(const std::string &str) = 0;

protected:
  rclcpp::Node *node_;
  StateTimer timer_;
};

template <class T, class Y>
class SensorState : public StateBase
{
  using DataTypeSharedPtr = typename T::SharedPtr;
  using SubscriptionType = typename rclcpp::Subscription<T>::SharedPtr;

public:
  SensorState(rclcpp::Node *node, TopicState &state, std::string topic_config_path)
      : StateBase(node), data_(state), topic_config_path_(topic_config_path)
  {
    init();
  }
  ~SensorState() override = default;

public:
  virtual void init() override;
  virtual int getState() override;
  virtual std::string getTopic() override;
  virtual std::string getName() override;
  std::string getType() override;
  virtual float getTimeout() override;
  virtual bool getEnable() override;
  virtual int getMul() override;
  virtual void monitor_exist_lidar(const std::string &str) override;
  void callback(const DataTypeSharedPtr msg);

private:
  SubscriptionType sub_;
  TopicState data_;
  std::string topic_config_path_;
};

////////////////////////////////////////////////////////////// 建图通信信息结构体
struct MappingCmncType
{
  string msg_id;        //id
  string command_state; //指令状态
  string command_type;  //指令类型
  string command_mode;  //设备ID
  string entityId;      //机器id
  string stamp;         //
};

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
  std::vector<std::string> s_split(const std::string &in,
                                   const std::string &delim)
  {
    std::regex re{delim};
    return std::vector<std::string>{
        std::sregex_token_iterator(in.begin(), in.end(), re, -1),
        std::sregex_token_iterator()};
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
               localctm.tm_year + 1900, localctm.tm_mon + 1, localctm.tm_mday,
               localctm.tm_hour, localctm.tm_min, localctm.tm_sec);
    return buf;
  }

  bool startMapping()
  {
    bool ret = false;
    std::cout << "ssssssssssssssssssssss " << startMappingCmd << std::endl;
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
  bool sensor_state{true};
  std::string message{""};
  std::string whole_message{""};
};

////////////////////////////////////////////////////////////////雷达校准
// enum CALIBRATION_LIDAR_SWITCH
// {
//   UPLOAD_IDLE = 50,
//   UPLOAD_START = 51,
//   UPLOAD_SUCCESS = 52,
//   UPLOAD_FAILED = 53
// };

struct CalibLidar2
{
  std::string start_calibrate_lidar2{"/opt/ros/humble/share/lidar2lidar_calibration/scripts/start"};
  std::string stop_calibrate_lidar2{"/opt/ros/humble/share/lidar2lidar_calibration/scripts/stop"};

  bool startCalibLidar2()
  {
    bool ret = false;
    if (access(start_calibrate_lidar2.c_str(), F_OK) == 0)
    {
      std::system(start_calibrate_lidar2.c_str());
      std::cout << "start calibrate_lidar " << std::endl;
      ret = true;
    }
    return ret;
  }

  bool stopCalibLidar2()
  {
    bool ret = false;
    if (access(stop_calibrate_lidar2.c_str(), F_OK) == 0)
    {
      std::system(stop_calibrate_lidar2.c_str());
      std::cout << "stop calibrate_lidar" << std::endl;
      ret = true;
    }
    return ret;
  }
};

struct CalibLidarTopicState
{
  std::string name;
  std::string sub_topic;
  std::string pub_topic;
  int rank;
};

class CalibLidar
{
public:
  explicit CalibLidar(rclcpp::Node *node, std::string pub_topic,
                      std::string sub_topic, std::string name, int rank) : node_(node),
                                                                           pub_topic_(pub_topic),
                                                                           sub_topic_(sub_topic),
                                                                           name_(name),
                                                                           rank_(rank) { init(); }

  void init();
  void setState();
  const int getRank();
  const std::string getName();
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  rclcpp::Node *node_;
  bool rslidar_web_switch{false};
  std::string sub_topic_;
  std::string pub_topic_;
  std::string name_;
  int rank_{0};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr calibration_lidar_pub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr calibration_lidar_sub;
};

//////////////////////////////////////////////////////////////

class MapSample : public rclcpp::Node
{
public:
  MapSample();
  ~MapSample();
  void run();
  void doWork();
  void upLoad();
  void Check_Mapping_State();
  void sendAppBuildState(bool trigger = false);
  void pub_nav_version();

private:
  void init();

  void pubErrorMsg(const std::string &modular_name, const uint &error_code,
                   const uint &error_level, const std::string &error_msg);
  void gnnsLocationCallback(const cti_msgs::msg::GnssRTK::SharedPtr nav_satfix);
  void mappingStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void sensorStateCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void rosWebCmdCallback(const std_msgs::msg::Int32::SharedPtr msg);

  void mapToJpgCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void senfor_state_Callback(const cti_msgs::msg::BoxState::SharedPtr msg);
  void save_web_diary(const std::string &msg, const std::string &msgT = "无");

  void calibrationCameraCallback(
      const sensor_msgs::msg::Image::SharedPtr msg);

  void calibrationLidar2Callback(const std_msgs::msg::String::SharedPtr msg);
  void parameterServerCallback(
      const std_msgs::msg::String::SharedPtr msg);
  void dynamic_save_web_diary(
      const cti_msgs::msg::ErrorStatusArray msg);
  void ult1_state_Callback(
      const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void ult2_state_Callback(
      const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void ult0_state_Callback(
      const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void node_alive_Callback(
      const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void box_msg_Callback(
      const cti_rblite_msgs::msg::BoxAskResponse::SharedPtr msg);
  void NavDeliveryResp_Callback(
      const cti_rblite_msgs::msg::NavDeliveryResp::SharedPtr msg);
  void send_web_diary(std::queue<DIARY_JSON> diary_data_down);
  void Mapping_statu_Callback(const std_msgs::msg::Bool::SharedPtr msg);
  void Imu_statu_Callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void batcell_state_Callback(
      const cti_msgs::msg::BatteryCellsState::SharedPtr msg);
  void dustboxbat_state_Callback(
      const cti_msgs::msg::BatteryCellsState::SharedPtr msg);
  void error_msg_Callback(const cti_msgs::msg::ErrorStatusArray::SharedPtr msg);
  const std::string merge_nameAndmodule(const cti_msgs::msg::ErrorStatus msg);
  void thread_web_lidar_calibration(const std::string &msg);
  void robot_env_sub(const std_msgs::msg::String::SharedPtr msg);
  void mappingCallback(const std_msgs::msg::String::SharedPtr msg);
  void pub_webTimer();
  void err_monitor();
  void openAndsendtf_luanch();
  void upload_points_Date();
  void timercallback();
  const bool getLidarTopicStateParams();
  void initCalibrationLidarParam();
  void check_tf_exit_luanch();
  void getTopicStateParams(std::vector<TopicState> &topicStates);
  void monitorMapBuildSensorState();

private:
  //定时器
  rclcpp::TimerBase::SharedPtr timer_;

  //订阅者

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr parameter_server_sub;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub;

  rclcpp::Subscription<cti_msgs::msg::GnssRTK>::SharedPtr gps_sub;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensorState_sub;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr webcmd_sub;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapToJpg_sub;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr ndt_pose_sub;

  rclcpp::Subscription<cti_msgs::msg::BoxState>::SharedPtr sensor_state_sub;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      ult1_state_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      ult2_state_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      ult0_state_sub;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      node_alive_sub;

  rclcpp::Subscription<cti_rblite_msgs::msg::BoxAskResponse>::SharedPtr
      box_msg_sub;

  rclcpp::Subscription<cti_rblite_msgs::msg::NavDeliveryResp>::SharedPtr
      NavDeliveryResp_msg_sub;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Mapping_statu_sub;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr Imu_statu_sub;

  rclcpp::Subscription<cti_msgs::msg::BatteryCellsState>::SharedPtr
      batcell_state_sub;

  rclcpp::Subscription<cti_msgs::msg::BatteryCellsState>::SharedPtr
      dustboxbat_state_sub;

  rclcpp::Subscription<cti_msgs::msg::ErrorStatusArray>::SharedPtr
      error_msg_sub;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr temp_use_rob_env_sub;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapping_state_sub;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr calibration_camera_sub;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr calibration_lidar2_sub;

  // publiser
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr check_tf_file_publisher;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr web_timer_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_version_web_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr odometer_web_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dustbox_battery_web_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vihicle_battery_web_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Imu_web_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diary_web_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr NavDeliveryResp_web_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr box_msg_web_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensor_status_web_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr webstate_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr web_tf_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr web_tf_msg_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapToPng_encodebase64_pub;

  rclcpp::Publisher<cti_msgs::msg::ErrorStatus>::SharedPtr error_publisher;

  rclcpp::Publisher<cti_msgs::msg::ErrorStatus>::SharedPtr pub_error_msg;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr web_send_calibration_tf_launch;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mappingstate_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_image_encodebase64_pub;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mappingstate_TEST_pub;

  const std::shared_ptr<rclcpp::Node> node_param_ =
      rclcpp::Node::make_shared("mapsample_params_");

  const std::shared_ptr<rclcpp::Node> node_ftp_uploadMap =
      rclcpp::Node::make_shared("node_ftp_uploadMap");

  double Mappint_last_time{-1};
  bool flag_Mapping_start{false};
  bool err_end_flag{0};

  std::string robot_env;

  double err_monitor_timer_{0};

  std::string lidar_png_out_path;
  std::string shutdown_cmd_path;
  std::string check_pointdata_cmd_path;
  std::string CTI_SW_VER;

  cti_msgs::msg::ErrorStatusArray cur_Err_code;
  cti_msgs::msg::ErrorStatusArray tran_Err_code;
  cti_msgs::msg::ErrorStatusArray relea_Err_code;
  cti_msgs::msg::ErrorStatusArray rank_Err_code;

  MapSampleData data;
  UpLoadManage upload;
  MapSampleManage mapsample;
  MappingCmncType mapping_cmnc_type;

  std::vector<CalibLidarTopicState> calibLidarTopicStates;
  std::vector<std::shared_ptr<CalibLidar>> calibLidars;
  std::string calibration_lidar_config_path_;
  std::string calibration_launch_path_;

  std::string vehicle_type;
  std::string calibLidarLaunch;

  std::vector<std::shared_ptr<StateBase>> sensor_states_;
  std::string cti_topic_config_path_;

  CalibLidar2 caliblidar2;
};

#endif
