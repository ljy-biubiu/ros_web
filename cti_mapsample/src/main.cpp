#include <ros/ros.h>
#include "mapsample/mapsample.h"
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <vector>
#include "dif_area_translate.h"
#include "core.h"
//#include "FTPClient.h"
// #include <pcl/filters/voxel_grid.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/point_cloud.h>


// #include<string>
// #include<fstream>
// #include <iostream>


using namespace std;


int main(int argc, char* argv[])
{
    // std::vector<int> las;
    // las.push_back(1);
    // las.push_back(12);
    // las.push_back(123);
    // las[0] = 11;
    // std::cout<<"ssssssssssssssssssss"<<std::endl;


    ros::init(argc, argv, "cti_mapsample_node");

    if (setUnlimit() == -1) 
    {
        return -1;
    }
    MapSample sample;
    sample.run();
    return 0;
}

    ///////////////////////////////////////////////////////////////////////////////////////     FTP简单版   注意文件太大，要加个线程
        // embeddedmz::CFTPClient FTPClient([](const std::string& strLogMsg) {                             
        //               std::cout << strLogMsg << std::endl;
        //             });

        // FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp001", "Ftp001@2020"); 
        // FTPClient.RemoveFile("/home/ljy/tf.launch");
        // bool upload_result = FTPClient.UploadFile("/home/ljy/tf.launch", "/ljy/test/tf.launch");    /////////文件 文件
        // std::cout << "uploading..." << std::endl;
        // if(upload_result){
        //     std::cout << "success" << std::endl;
        // }else{
        //     std::cout << "failed" << std::endl;
        // }
        // FTPClient.CleanupSession();



    // ////////////////////////////////////////////////////////////////////////////////////將点读入容器
    // double a[][2] = 
    // {
    //     {189.5276,-132.9335},
    //     {189.1371,-109.5281},
    //     {189.0580,-101.7255},
    //     {188.6901,-74.3415},
    //     {188.3283,-47.2433},
    //     {188.0530,-33.0026},
    //     {187.3584,7.3233},
    //     {173.6895,7.0752},
    //     {173.8806,-0.7651},
    //     {173.8888,-3.4505},
    //     {173.1444,-3.5350},
    //     {173.3725,-13.7794},
    //     {173.8499,-13.8105},
    //     {173.8191,-15.9533},
    //     {173.3076,-15.9364},
    //     {173.4321,-22.2391},
    //     {173.5844,-28.2867},
    //     {178.3149,-28.2869},
    //     {178.3760,-33.2774},
    //     {178.6955,-45.5889},
    //     {179.0118,-52.4118},
    //     {179.0017,-71.8231},
    //     {179.2010,-78.0767},
    //     {179.6417,-101.6711},
    //     {174.7380,-101.7123},
    //     {175.1119,-118.1936},
    //     {175.7136,-118.2392},
    //     {175.8832,-130.6808},
    //     {180.1441,-130.4284},
    //     {180.3578,-135.9514},
    //     {178.8033,-135.8158},
    //     {178.9378,-140.9341},
    //     {180.4030,-140.9418},
    //     {180.5294,-149.6139},
    //     {189.8913,-149.5871},
    //     {189.5081,-132.8568}
    // };

    // std::list<std::map<std::string,double>> points;
    // std::list<std::map<std::string,double>> points_cp;                              //为遍历不相交点建立
    // int size = sizeof ( a ) /  sizeof ( a[0] );

    // for(int i = 0 ;size > i;i++)                                                     ///////將数据读取进容器
    // {
    //     std::map<std::string,double> x;
    //     for(int j = 0 ;2 > j;j++)
    //     {
    //         if(j == 0)
    //         {
    //             x.insert(pair<string, double>("x",a[i][j]));
    //         }
    //         else
    //         {
    //             x.insert(pair<string, double>("y",a[i][j])); 
    //         }
    //     }
    //     points.push_back(x);
    //     points_cp.push_back(x);   
    // }

    // double fillter_precise{1};
    // divide_area_first(points,fillter_precise,1);

    
    
    // std::vector<string> aaa{"1","2","3","4","5"};

    // for (auto iter = aaa.begin(); iter != aaa.end();++iter) 
    // {
    //     std::cout<<"111111111111222222222222222222223333333333333"<<std::endl;
    //     if ( *iter == "3" ) 
    //     {
    //         // std::cout<<"ssssssssssssssssssssssssssssssssssssss"<<std::endl;
    //         // printf("%u",iter);
    //         // std::cout<<"ssssssssssssssssssssssssssssssssssssss"<<std::endl;
    //         aaa.erase(iter);
    //         // continue;
    //     }
    //     std::cout<<"======================================"<<std::endl;
    //     std::cout<<*iter<<std::endl;
    //     std::cout<<"======================================"<<std::endl;       
    // }

    // for (auto iter = aaa.begin(); iter != aaa.end();++iter) 
    // {
    //     std::cout<<"======================================"<<std::endl;
    //     std::cout<<*iter<<std::endl;
    //     printf("%u",iter);
    //     std::cout<<"======================================"<<std::endl;
    // }











   // for(iter =points.begin(); iter!=points.end();iter++)
    // {
    //     std::cout<<" x : "<<iter->find("x")->second<<std::endl;
    //     std::cout<<" y : "<<iter->find("y")->second<<std::endl;
    //     std::cout<<"===================================="<<std::endl;
    // }                                   
    // std::cout<<"***********************************************"<<std::endl;

    // points.push_back(points.front());
    // points.pop_front();

    // std::cout<<"***********************************************"<<std::endl;
    // for(iter =points.begin(); iter!=points.end();iter++)
    // {
    //     std::cout<<" x : "<<iter->find("x")->second<<std::endl;
    //     std::cout<<" y : "<<iter->find("y")->second<<std::endl;
    //     std::cout<<"===================================="<<std::endl;
    // }                                   
    // std::cout<<"***********************************************"<<std::endl;



    //points.back().find("x")->second<<"   y:"<<points.back().find("y")->second<<std::endl;
    // point_c ass;
    // ass.x = -10;
    // ass.y = 6;
    // point_c bss;
    // bss.x = -5;
    // bss.y = 3;
    // point_c css;
    // css.x = -10;
    // css.y = -1;
    // point_c dss;
    // dss.x = -6;
    // dss.y = 3;


    // point_c css = get_two_points_K_B_V(ass,bss);
    // std::cout<<"=========================================================================="<<std::endl;
    // std::cout<<"=========================================================================="<<std::endl;
    // std::cout<<"y"<<css.y<<"x"<<css.x<<std::endl;
    // std::cout<<"=========================================================================="<<std::endl;
    // std::cout<<"=========================================================================="<<std::endl;
    
    // std::cout<<"bool:"<<isintersected(ass,bss,css,dss)<<std::endl;


    // int SIZE = points.size();
    // for( int i = 0;SIZE>i;i++ )
    // {
    //     //std::cout<<points.front().find("x")->first<<points.front().find("x")->second<<std::endl;
    //     points.pop();
    // }
    // if()

// list的正向与反向遍历
// for(riter =points.rbegin(); riter!=points.rend();riter++)
// {
//     std::cout<<"===================================="<<std::endl;
//     std::cout<<" x : "<<riter->find("x")->second<<std::endl;
//     std::cout<<" y : "<<riter->find("y")->second<<std::endl;
//     std::cout<<"===================================="<<std::endl;
// }

// for(iter =points.begin(); iter!=points.end();iter++)
// {
//     std::cout<<"===================================="<<std::endl;
//     std::cout<<" x : "<<iter->find("x")->second<<std::endl;
//     std::cout<<" y : "<<iter->find("y")->second<<std::endl;
//     std::cout<<"===================================="<<std::endl;
// }












// void writeFileJson()
// {
// 	//根节点  
// 	Json::Value root;
// 	//根节点属性  
// 	root["name"] = Json::Value("shuiyixin");
// 	root["age"] = Json::Value(21);
// 	root["sex"ssss] = Json::Value("man");
 
// 	//子节点  
// 	Json::Value friends;
//     friends.append(root);
//     friends.append(root);
//     friends.append(root);

// 	//缩进输出
// 	Json::StyledWriter sw;
// 	cout << sw.write(friends) << endl << endl;
 
// 	//输出到文件  
// 	ofstream os;
// 	os.open("/home/ljy/dev/demo.json", std::ios::out | std::ios::app);
// 	if (!os.is_open())
// 	cout << "error：can not find or create the file which named \" demo.json\"." << endl;
// 	os << sw.write(friends);
// 	os.close();
 
// }

// void readFileJson()
// {
// 	Json::Reader reader;
// 	Json::Value root;
//     Json::Value friends;
//     ifstream ifs;
//     ifs.open("/home/ljy/dev/demo.json");

//     if (!reader.parse(ifs, root))
//     {
//         std::cout<<"读取json失败"<<std::endl;
//     }
//     ifs.close();
//     std::cout<<root<<std::endl;
//     for (auto it = root.begin();it!=root.end();it++)
//     {
//         for (auto sit = it->begin();sit != it->end();sit++)
//         {
//             std::cout<<sit.key()<<"\t"<<sit.name()<<std::endl;
//         }
//     }
// }




    // std::string finname = "/home/ljy/dev/catkin_wx/src/cti_data_viewer/cti_mapsample/tayg.pcd";

    // //创建点云指针
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // //直接读取pcd文件到cloud中，并判断是否读取正确
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(finname, *cloud) == -1) //* load the file
    // {
    //     PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    //     return (-1);
    // }
    //                                                                                                             // //控制台输出点云宽高(无序点云为总数)
    //                                                                                                             // std::cout << "Loaded "
    //                                                                                                             //           << cloud->width * cloud->height
    //                                                                                                             //           << " data points from test_pcd.pcd with the following fields: "
    //                                                                                                             //           << std::endl;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	// cloud2->width = cloud->width;
	// cloud2->height = 1;	
	// cloud2->is_dense = false;
	// cloud2->resize(cloud2->width * cloud2->height);

    // std::cout<<cloud->width<<"----------------WH---------------------"<<cloud->height<<std::endl;

    // //遍历点云输出坐标
    // for (const auto &point : *cloud)
    // {
        
    //     if(point.z<20)
    //     {
    //         static int i =0;
    //         cloud2->points[i].x =point.x;
    //         cloud2->points[i].y =point.y;
    //         cloud2->points[i].z =point.z;
    //         i++;

    //     }
    //     //std::cout<<i<<"----------------WH---------------------"<<std::endl;

    // }
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::VoxelGrid<pcl::PointXYZ> voxel;
    // voxel.setInputCloud(cloud2);
    // voxel.setLeafSize(0.9f,0.9f,0.9f);//体素大小设置为30*30*30cm  //越趋紧1越少
    // voxel.filter(*cloud_voxel_filtered);
    // std::cout<<cloud->width<<"----------------WH---------------------"<<cloud->height<<std::endl;

    // pcl::io::savePCDFile("/home/ljy/dev/catkin_wx/src/cti_data_viewer/cti_mapsample/pcd1011.pcd", *cloud_voxel_filtered);


    // std::string aaa ="var line_origal_roadage = 113241654dsadasd";
    // std::cout<<"-----------------------------------------------"<<std::endl;
    // std::cout<<aaa.substr(4,19)<<std::endl;
    // if(aaa.substr(4,19)=="line_origal_roadage")
    // {
    //     std::cout<<"--------------------65465465456----------------------"<<std::endl;
    // }
 


    // std::ofstream file;
    // if (file.bad())
    // {
    //     std::cout << "cannot open file" << std::endl;
    // }
    // file.open("/home/ljy/dev/ljy.txt", std::ios::app);
    // file << "541654564" << "\n";
    // file << "541654564" << "\n";
    // file << "541654564" << "\n";
    // file << "541654564" << "\n";
    // file.close();

    // std::queue<std::string> words;
    // std::string asda{"sdada"};

    // for(int i=0;i<10;i++)
    // {
    //     std::string ss = boost::lexical_cast<string>(i);
    //     std::string aa = "dasdasd"+":"+ss;
    //     words.push(aa);
    // }

    // int myqueue_size = words.size();
    // for(int i = 0; i < myqueue_size; i++) {   //myqueue_size 必须是固定值
    //   cout << words.front() << endl;
    //   words.push(words.front());
    //   words.pop();
    // } 
    // writeFileJson();
    // readFileJson();
    // cti_msgs::ErrorStatusArray cur;
    // cti_msgs::ErrorStatusArray rec;
    // cti_msgs::ErrorStatus s;
    // cti_msgs::ErrorStatus ss;
    // cti_msgs::ErrorStatus sss;
    // s.error_code = 100;
    // ss.error_code = 1000;
    // sss.error_code = 10000;
    // cur.data.push_back(s);
    // cur.data.push_back(ss);
    // cur.data.push_back(sss);
    // for(auto qwer : cur.data)
    // {
    //     std::cout<<"555555555555555555555555555555555555555555"<<std::endl;
    // }
    // for (auto iter = cur.data.begin(); iter != cur.data.end(); ++iter) {
    // if (iter->error_code == 100 ) {
    //    cur.data.erase(iter);
    // }
    // }
    // for(auto qwer : cur.data)
    // {
    //     std::cout<<"555555555555555555555555555555555555555555"<<std::endl;
    // }

    // std::queue<std::map<std::string,double>> points;                      //////////////使用queue容器储存数据
    // int size = sizeof ( a ) /  sizeof ( a[0] );
    // for(int i = 0 ;size > i;i++)                     ///////將数据读取进容器
    // {
    //     std::map<std::string,double> x;
    //     for(int j = 0 ;2 > j;j++)
    //     {
    //         if(j == 0)
    //         {
    //             x.insert(pair<string, double>("x",a[i][j]));
    //         }
    //         else
    //         {
    //             x.insert(pair<string, double>("y",a[i][j])); 
    //         }
    //     }
    //     points.push(x);   
    // }



    // pcl::PointCloud<PointT>::Ptr cloud_voxel_filtered(new pcl::PointCloud<PointT>());
    // pcl::VoxelGrid<PointT> voxel;
    // voxel.setInputCloud(cloud_filtered);
    // voxel.setLeafSize(0.2f, 0.2f, 0.2f);//体素大小设置为30*30*30cm
    // voxel.filter(*cloud_voxel_filtered);
