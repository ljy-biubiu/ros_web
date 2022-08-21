#ifndef __MATTOPNG_BASE64_H__
#define __MATTOPNG_BASE64_H__

#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "cti_msgs/msg/tab_state.hpp"

// namespace wz{
using namespace boost::archive::iterators;
#define half_of_PI 1.57079

//这里的代码可以存取png图片
// std::vector<int> compression_params;
// compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
// //PNG格式图片的压缩级别 compression_params.push_back(2);
// //这里设置保存的图像质量级别
// //std::cout<<lidar_png_out_path+"lidar_map.png"<<std::endl;
// imwrite(lidar_png_out_path+"lidar_map.png", dst);

struct Quaternion {
  double w, x, y, z;
};

struct EulerAngles {
  double roll, pitch, yaw;
};

struct Coordination_pose {
  Coordination_pose() {
    x = 0, y = 0, z = 0;
    angl.roll = 0, angl.pitch = 0, angl.yaw = 0;
  };
  double x, y, z;
  EulerAngles angl;
};

std::string base64Encode(const unsigned char *Data, int DataByte) {
  //编码表
  const char EncodeTable[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  //返回值
  std::string strEncode;
  unsigned char Tmp[4] = {0};
  int LineLength = 0;
  for (int i = 0; i < (int)(DataByte / 3); i++) {
    Tmp[1] = *Data++;
    Tmp[2] = *Data++;
    Tmp[3] = *Data++;
    strEncode += EncodeTable[Tmp[1] >> 2];
    strEncode += EncodeTable[((Tmp[1] << 4) | (Tmp[2] >> 4)) & 0x3F];
    strEncode += EncodeTable[((Tmp[2] << 2) | (Tmp[3] >> 6)) & 0x3F];
    strEncode += EncodeTable[Tmp[3] & 0x3F];
    if (LineLength += 4, LineLength == 76) {
      strEncode += "\r\n";
      LineLength = 0;
    }
  }
  //对剩余数据进行编码
  int Mod = DataByte % 3;
  if (Mod == 1) {
    Tmp[1] = *Data++;
    strEncode += EncodeTable[(Tmp[1] & 0xFC) >> 2];
    strEncode += EncodeTable[((Tmp[1] & 0x03) << 4)];
    strEncode += "==";
  } else if (Mod == 2) {
    Tmp[1] = *Data++;
    Tmp[2] = *Data++;
    strEncode += EncodeTable[(Tmp[1] & 0xFC) >> 2];
    strEncode += EncodeTable[((Tmp[1] & 0x03) << 4) | ((Tmp[2] & 0xF0) >> 4)];
    strEncode += EncodeTable[((Tmp[2] & 0x0F) << 2)];
    strEncode += "=";
  }

  return strEncode;
}

// imgType 包括png bmp jpg jpeg等opencv能够进行编码解码的文件
std::string Mat2Base64(const cv::Mat &img, std::string imgType) {
  // Mat转base64
  std::string img_data;
  std::vector<uchar> vecImg;
  std::vector<int> vecCompression_params;
  vecCompression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  vecCompression_params.push_back(90);
  imgType = "." + imgType;
  cv::imencode(imgType, img, vecImg, vecCompression_params);
  img_data = base64Encode(vecImg.data(), vecImg.size());
  return img_data;
}

cv::Mat matRotateClockWise270(cv::Mat src)  //顺时针270
{
  if (src.empty()) {
    // qDebug() << "RorateMat src is empty!";
  }
  // 矩阵转置
  // transpose(src, src);
  // 0: 沿X轴翻转； >0: 沿Y轴翻转； <0: 沿X轴和Y轴翻转
  transpose(
      src,
      src);  // 翻转模式，flipCode ==
             // 0垂直翻转（沿X轴翻转），flipCode>0水平翻转（沿Y轴翻转），flipCode<0水平垂直翻转（先沿X轴翻转，再沿Y轴翻转，等价于旋转180°）
  flip(src, src, 0);
  return src;
}

EulerAngles ToEulerAngles(Quaternion q) {
  EulerAngles angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  angles.roll = std::atan2(sinr_cosp, cosr_cosp);
  // std::cout<<"roll"<<angles.roll<<endl;

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1)
    angles.pitch =
        std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    angles.pitch = std::asin(sinp);
  // std::cout<<"pitch"<<angles.pitch<<endl;

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.yaw = std::atan2(siny_cosp, cosy_cosp);
  // std::cout<<"yaw"<<angles.yaw<<endl;

  return angles;
}

inline double setPercision(double num) {
  return num = double(int(num * 1000)) / 1000;
}

// }
// Coordination_pose add_coordinations(const std::string &frame, const
// std::string &child_frame, const tf::TransformListener &tf_listen_,
// std::vector<cti_msgs::TabState> &coordinations)
// {

//     Quaternion quaternion;
//     Coordination_pose Coordin_pose;
//     cti_msgs::TabState tabstate;
//     tf::StampedTransform transform_base; //定义存放变换关系的变量

//     // nav_msgs::Odometry position;
//     // tf::Quaternion q;
//     // double roll,pitch,yaw;

//     //监听两个坐标系之间的变换
//     try
//     {
//         tf_listen_.waitForTransform(frame, child_frame, ros::Time(0),
//         ros::Duration(0.01)); tf_listen_.lookupTransform(frame, child_frame,
//         ros::Time(0), transform_base);
//     }
//     catch (tf::TransformException ex)
//     {
//         ROS_ERROR("%s", ex.what());
//         return Coordin_pose;
//     }

//     quaternion.x = transform_base.getRotation().getX();
//     quaternion.y = transform_base.getRotation().getY();
//     quaternion.z = transform_base.getRotation().getZ();
//     quaternion.w = transform_base.getRotation().getW();

//     Coordin_pose.x = transform_base.getOrigin().x();
//     Coordin_pose.y = transform_base.getOrigin().y();
//     Coordin_pose.z = transform_base.getOrigin().z();

//     // position.pose.pose.orientation.w =
//     transform_base.getRotation().getW();
//     // position.pose.pose.orientation.z =
//     transform_base.getRotation().getZ();
//     // position.pose.pose.orientation.y =
//     transform_base.getRotation().getY();
//     // position.pose.pose.orientation.x =
//     transform_base.getRotation().getX();
//     // tf::quaternionMsgToTF(position.pose.pose.orientation,q);
//     // tf::Matrix3x3(q).getRPY(roll,pitch,yaw);

//     // Coordin_pose.angl.yaw=yaw;
//     // Coordin_pose.angl.roll=roll;
//     // Coordin_pose.angl.pitch=pitch;
//     Coordin_pose.angl = ToEulerAngles(quaternion);

//     std::string combine_message;
//     stringstream ss;
//     ss << "yaw:" << setPercision(Coordin_pose.angl.yaw) << "  roll:" <<
//     setPercision(Coordin_pose.angl.roll) << "  pitch" <<
//     setPercision(Coordin_pose.angl.pitch) << "  x:" <<
//     setPercision(Coordin_pose.x) << "  y:" << setPercision(Coordin_pose.y) <<
//     "  z:" << setPercision(Coordin_pose.z);
//     //std::cout<<setPercision(Coordin_pose.angl.yaw)<<setPercision(Coordin_pose.angl.roll)<<setPercision(Coordin_pose.angl.pitch)<<setPercision(Coordin_pose.x)<<setPercision(Coordin_pose.y)<<setPercision(Coordin_pose.z)<<endl;
//     combine_message = ss.str();
//     tabstate.name = child_frame;
//     tabstate.message = combine_message;
//     coordinations.push_back(tabstate);
//     return Coordin_pose;
// }

// void draw_coordinations(cv::Mat &dst, const Coordination_pose
// &coordination_pose, const int &height, const int &width)
// {

//     int Pj = 10 * coordination_pose.x + 100;
//     int Pi = 10 * coordination_pose.y + 100;
//     dst.at<cv::Vec3b>(height - Pi - 1, Pj)[0] = 0;
//     dst.at<cv::Vec3b>(height - Pi - 1, Pj)[1] = 0;
//     dst.at<cv::Vec3b>(height - Pi - 1, Pj)[2] Quaternion quaternion;
//     {
//         Pj = cos(coordination_pose.angl.yaw) * i;
//         Pj = cos(coordination_pose.angl.pitch) * Pj; //改变投影值
//         Pj = 10 * coordination_pose.x + 100 + Pj;
//         Pi = sin(coordination_pose.angl.yaw) * i;
//         Pi = cos(coordination_pose.angl.pitch) * Pi; //改变投影值
//         Pi = 10 * coordination_pose.y + 100 + Pi;
//         dst.at<cv::Vec3b>(height - Pi - 1, Pj)[0] = 0;
//         dst.at<cv::Vec3b>(height - Pi - 1, Pj)[1] = 0;
//         dst.at<cv::Vec3b>(height - Pi - 1, Pj)[2] = 255;
//     }

//     for (int i = 1; i < 7; i++)
//     {
//         Pj = cos(coordination_pose.angl.yaw + half_of_PI) * i;
//         Pj = cos(coordination_pose.angl.roll) * Pj; //改变投影值
//         //std::cout<<"Pj"<<Pj<<std::endl;
//         Pj = 10 * coordination_pose.x + 100 + Pj;
//         Pi = sin(coordination_pose.angl.yaw + half_of_PI) * i;
//         Pi = cos(coordination_pose.angl.roll) * Pi; //改变投影值
//         //std::cout<<"Pi"<<Pi<<std::endl;
//         Pi = 10 * coordination_pose.y + 100 + Pi;
//         dst.at<cv::Vec3b>(height - Pi - 1, Pj)[0] = 0;
//         dst.at<cv::Vec3b>(height - Pi - 1, Pj)[1] = 255;
//         dst.at<cv::Vec3b>(height - Pi - 1, Pj)[2] = 0;
//     }
// }

#endif
