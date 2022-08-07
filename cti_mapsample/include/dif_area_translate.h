#ifndef __DIF_AREA_TRANSLATE_H__
#define __DIF_AREA_TRANSLATE_H__


#include <queue>
#include <map>
#include <list>
#include <deque>
#include <math.h>
#include <iostream>
#include <fstream> // c++文件操作
struct point_c
{
    double x,y;
};

point_c point_last_2;
point_c point_last;
point_c point_cur;
point_c point_next;

point_c point_mid;
point_c point_beam;

point_c tranfrom_point_one;
point_c tranfrom_point_two;

int cor_numb{0};                                            //此段为二次塞选使用
int key_que[] ={0,1,0};
bool flag_is_special_polygen{0};
std::deque<int> content_queue{0,1,0};
std::queue<int> if_special_polygen(content_queue);

bool reserve_act{0};
bool if_hollow_point;
bool start_flag{false};
bool if_point_in_triangle{0};
static point_c mark_same_point;
bool success_calibration_flag{0};

int mark_count{0};
int odd_even_numb{0};

std::vector<std::vector<point_c> > vec_new_polygon;
std::list<std::map<std::string,double> >::iterator iter;
std::list<std::map<std::string,double> >::reverse_iterator riter;





/////////////////////////////////////////////////////////////////////////////////////旁断2个线段是否相交
    double multi(point_c a,point_c b,point_c c)//abxac,ab(b.x-a.x,b.y-a.y),ac(c.x-a.x,c.y-a.y)
    {
        return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
    }

    bool isintersected(point_c a,point_c b,point_c c,point_c d)//判断两条线段是否相交
    {
        double u,v,w,z;
        u=multi(a,b,c);
        v=multi(a,b,d);
        w=multi(c,d,b);
        z=multi(c,d,a);
        return (u*v<=0.00000001&&w*z<=0.00000001);//1相交0不相交
    }
/////////////////////////////////////////////////////////////////////////////////////旁断2个线段是否相交

/////////////////////////////////////////////////////////////////////////////////////判断俩点的中点

    point_c get_two_points_mid(point_c a,point_c b)
    {
        point_c cur_data;
        cur_data.x = (a.x + b.x)/2;
        cur_data.y = (a.y + b.y)/2;
        return cur_data;
    }

/////////////////////////////////////////////////////////////////////////////////////判断俩点的中点

/////////////////////////////////////////////////////////////////////////////////////获取射线的另外一个点(俩点的线段斜率,b,射线方向）

    point_c get_two_points_K_B_V(point_c a,point_c b)
    {
        point_c cur_data;
        double k = (a.y-b.y)/(a.x-b.x);
        double B = a.y - k*a.x;
        double v_x = a.x - b.x;
        if(v_x>0)
        {
            cur_data.x = -10000;
        }
        else
        {
            cur_data.x = 10000;
        }
        cur_data.y = k*cur_data.x + B;
        //std::cout<<"y = "<<k<<"x +"<<B<<std::endl;
        return cur_data;
    }

/////////////////////////////////////////////////////////////////////////////////////获取射线的另外一个点(俩点的线段斜率,b,射线方向）

// 3D vector      
class Vector3
{
public:
    Vector3(double fx, double fy, double fz)
        :x(fx), y(fy), z(fz)
    {
    }

    // Subtract
    Vector3 operator - (const Vector3& v) const
    {
        return Vector3(x - v.x, y - v.y, z - v.z) ;
    }

    // Dot product
    double Dot(const Vector3& v) const
    {
        return x * v.x + y * v.y + z * v.z ;
    }

    // Cross product
    Vector3 Cross(const Vector3& v) const
    {
        return Vector3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x ) ;
    }

public:
    double x, y, z ;
};

// Determine whether point P in triangle ABC
bool PointinTriangle(Vector3 A, Vector3 B, Vector3 C, Vector3 P)
{
    Vector3 v0 = C - A ;
    Vector3 v1 = B - A ;
    Vector3 v2 = P - A ;

    double dot00 = v0.Dot(v0) ;
    double dot01 = v0.Dot(v1) ;
    double dot02 = v0.Dot(v2) ;
    double dot11 = v1.Dot(v1) ;
    double dot12 = v1.Dot(v2) ;

    double inverDeno = 1 / (dot00 * dot11 - dot01 * dot01) ;

    double u = (dot11 * dot02 - dot01 * dot12) * inverDeno ;
    if (u < 0 || u > 1) // if u out of range, return directly
    {
        return false ;
    }

    double v = (dot00 * dot12 - dot01 * dot02) * inverDeno ;
    if (v < 0 || v > 1) // if v out of range, return directly
    {
        return false ;
    }

    return u + v <= 1 ;
}

///////////////////////////////////////////////////////////////////////////////////////////写入csv文件
void write_into_file(std::string path)
{

    ofstream ofile;                                                         //定义输出文件
    ofile.open(path);       //作为输出文件打开

    std::cout<<"*******************************"<<std::endl;
    std::cout<<vec_new_polygon.size()<<std::endl;
    int i{0};
    for( auto Bv : vec_new_polygon )
    {
        for(auto Sv : Bv)
        {
            ofile<<"9,"<<i<<","<<Sv.x<<","<<Sv.y<<",0,0,0,0,0,0,0,0,0,0,0,0"<<std::endl;
            std::cout<<Sv.x<<"========"<<Sv.y<<std::endl;
        }
        i++;
    }
    ofile.close(); 
    ///////////////////////////////////////////////////////////////////////////////////////////写入csv文件
}

///////////////////////////////////////////////////////////////////////////////////////////////////  过滤 无效点
void fillter_useless_points( std::list<std::map<std::string,double> > &points, double fillter_precise )
{
    int points_size = points.size();                                    //  过滤精度
    for(iter = points.begin();points_size != 0;points_size--,iter++)
    {
        bool x_err_first = ( abs(iter->find("x")->second - (++iter)->find("x")->second) < fillter_precise );  
        bool x_err_second = (abs(iter->find("x")->second - (++iter)->find("x")->second) < fillter_precise );

        bool y_err_first = (abs(iter->find("y")->second - (--iter)->find("y")->second) < fillter_precise );
        bool y_err_second = (abs(iter->find("y")->second - (--iter)->find("y")->second) < fillter_precise );

        if( (x_err_first && x_err_second) || (y_err_first && y_err_second) )
        {
            iter++;
            points.erase(iter);   
            iter = points.begin();
            iter--;
            points_size = points.size();
        }
        
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////  过滤 无效点


void divide_area_first(std::list<std::map<std::string,double> > &points , double fillter_precise ,bool is_save_file = 0 ,std::string path = "/home/ljy/dev/map_vmap/cti_vmap/wkyc/cleararea.csv")    /////  第一轮处理     缺漏：无法处理俩边凹进去，中间凸起来的几何
{
    fillter_useless_points(points,fillter_precise);

    while(1)                                                      
    {
        if(start_flag == false)                                     //对循环开始进行标志   循环到相同的点就退出break  说明无法再分割
        {
            mark_same_point.x = points.back().find("x")->second;
            mark_same_point.y = points.back().find("y")->second;
            start_flag = true;
        }

        iter = points.begin();                                      //获取三个相邻点
        iter++;

        point_last.x = points.back().find("x")->second;
        point_last.y = points.back().find("y")->second;
        point_cur.x = points.front().find("x")->second;
        point_cur.y = points.front().find("y")->second;
        point_next.x = iter->find("x")->second;
        point_next.y = iter->find("y")->second;

        point_mid = get_two_points_mid(point_last,point_next);                                 //返回线段中点
        point_beam = get_two_points_K_B_V(point_cur,point_mid);                                //返回截取射线的点
        odd_even_numb = 1;


        for(iter = points.begin(); iter != points.end(); iter++)                               //遍历全部点  判断是不是凹点
        {
            tranfrom_point_one.x = iter->find("x")->second;
            tranfrom_point_one.y = iter->find("y")->second;

            if( iter ==  points.begin())
            {
                continue;
            }
            else if(++iter == points.end())
            {
                break;
            }
            else
            {
                tranfrom_point_two.x = iter->find("x")->second;
                tranfrom_point_two.y = iter->find("y")->second;
                iter--;
            }

            if_hollow_point = isintersected(tranfrom_point_one,tranfrom_point_two,point_cur,point_beam);

            if(if_hollow_point == true)
            {
                odd_even_numb++;
            }
        }

        if(odd_even_numb % 2 == 0)                                                      //偶数为在外边 奇数在里边
        {
            mark_count++;
            if(mark_count >=2 )
            {
                success_calibration_flag = true;
            }
        }
        else
        {
            mark_count = 0;
        }
        //std::cout<<"*****************3333333333333333333****************"<<std::endl;
        if(success_calibration_flag == true)                                            //旁断点是否在三角形里面 在的话 不能要（二次塞选）
        {
            riter = points.rbegin();
            riter++;
            point_last_2.x = riter -> find("x")->second;
            point_last_2.y = riter -> find("y")->second;

            Vector3 point_Last{point_last_2.x,point_last_2.y,0};
            Vector3 point_Cur {point_last.x,point_last.y,0};
            Vector3 point_Next{point_cur.x,point_cur.y,0};
            
            if_point_in_triangle = false;                                                                   
            for(iter = points.begin(); iter != points.end(); iter++)
            {
                if((iter->find("x")->second == point_last_2.x && iter->find("y")->second == point_last_2.y) 
                || (iter->find("x")->second == point_last.x && iter->find("y")->second == point_last.y) 
                || (iter->find("x")->second == point_cur.x && iter->find("y")->second == point_cur.y))
                {
                    continue;
                }
                else
                {   
                    Vector3 point_Act{iter->find("x")->second,iter->find("y")->second,0};
                    if(PointinTriangle(point_Last,point_Cur,point_Next,point_Act))
                    {
                        if_point_in_triangle = true;
                    }
                }
            }
        }
        
        // std::cout<<"****************44444444444444444444444****************"<<std::endl;
        if(success_calibration_flag == true && if_point_in_triangle == false)       //俩轮塞选
        {
            std::vector<point_c> new_triangle;                                      //加入的新三角形
            new_triangle.push_back(point_last_2);
            new_triangle.push_back(point_last);
            new_triangle.push_back(point_cur);
            vec_new_polygon.push_back(new_triangle);

            points.pop_back();                                                      //將分割点移除
            mark_count = 0 ;                                                        //各种标志置零
            success_calibration_flag = false;
            if_point_in_triangle = false;
            start_flag = false;
        }

        if(mark_count < 2 || if_point_in_triangle == true)                             ///没出现内凹现象就移动容器
        {
            points.push_back(points.front());
            points.pop_front();
        }

        riter = points.rbegin();
        if(points.size() == 3 || ((mark_same_point.x == riter -> find("x") -> second && mark_same_point.y == riter -> find("y") -> second) && reserve_act != true ) )
        {
            points.reverse() ;
            start_flag = false ;
            reserve_act = true ;
            continue;
        }

        if(points.size() == 3 || ((mark_same_point.x == riter -> find("x") -> second && mark_same_point.y == riter -> find("y") -> second) && reserve_act ) )
        {
            std::vector<point_c> new_triangle;                                                   //加入的新多边形 

            for(iter = points.begin();iter != points.end();iter++)                               //將剩下的点放进一个容器
            {
                point_c new_point;
                new_point.x = iter->find("x")->second;
                new_point.y = iter->find("y")->second;
                new_triangle.push_back(new_point);
            }
            vec_new_polygon.push_back(new_triangle);
            if(is_save_file)
            {
                write_into_file(path);
            }
            break;
        }
    }
}


#endif




//////////////////////////////////////////////////////////////////////////////////////////////////   第一轮处理


    //////////////////////////////////////////////////////////////////////////////////////////////////   第二轮处理  



    // start_flag = false;
    // int circle_num = points.size();
    
    
    // while(1)
    // {

    //     if(start_flag == false)                                     //对循环开始进行标志   循环到相同的点就退出break  说明无法再分割
    //     {
    //         circle_num = 2 * (points.size());  
    //         start_flag = true;
    //     }

    //     iter = points.begin();                                      //获取三个相邻点
    //     iter++;


    //     point_last.x = points.back().find("x")->second;
    //     point_last.y = points.back().find("y")->second;
    //     point_cur.x = points.front().find("x")->second;
    //     point_cur.y = points.front().find("y")->second;
    //     point_next.x = iter->find("x")->second;
    //     point_next.y = iter->find("y")->second;

    //     point_c tranfrom_point_one;
    //     point_c tranfrom_point_two;

    //     point_mid = get_two_points_mid(point_last,point_next);                                 //返回线段中点
    //     point_beam = get_two_points_K_B_V(point_cur,point_mid);                                //返回截取射线的点
    //     odd_even_numb = 1;


    //     for(iter = points.begin(); iter != points.end(); iter++)                               //遍历全部点  判断是不是凹点
    //     {
    //         tranfrom_point_one.x = iter->find("x")->second;
    //         tranfrom_point_one.y = iter->find("y")->second;

    //         if( iter ==  points.begin())
    //         {
    //             continue;
    //         }
    //         else if(++iter == points.end())
    //         {
    //             break;
    //         }
    //         else
    //         {
    //             tranfrom_point_two.x = iter->find("x")->second;
    //             tranfrom_point_two.y = iter->find("y")->second;
    //             iter--;
    //         }

    //         if_hollow_point = isintersected(tranfrom_point_one,tranfrom_point_two,point_cur,point_beam);

    //         if(if_hollow_point == true)
    //         {
    //             odd_even_numb++;
    //         }
    //     }

    //     if(odd_even_numb % 2 == 0)                                                      //偶数为在外边 奇数在里边
    //     {
    //         if_special_polygen.pop();
    //         if_special_polygen.push(0);
    //     }
    //     else
    //     {
    //         if_special_polygen.pop();
    //         if_special_polygen.push(1);
    //     }


    //     if(true)                                            //旁断点是否在三角形里面 在的话 不能要（二次塞选）
    //     {
    //         riter = points.rbegin();
    //         riter++;
    //         point_last_2.x = riter -> find("x")->second;
    //         point_last_2.y = riter -> find("y")->second;

    //         Vector3 point_Last{point_last_2.x,point_last_2.y,0};
    //         Vector3 point_Cur {point_last.x,point_last.y,0};
    //         Vector3 point_Next{point_cur.x,point_cur.y,0};
            
    //         if_point_in_triangle = false;                                                                   
    //         for(iter = points_cp.begin(); iter != points_cp.end(); iter++)
    //         {
    //             if((iter->find("x")->second == point_last_2.x && iter->find("y")->second == point_last_2.y) 
    //             || (iter->find("x")->second == point_last.x && iter->find("y")->second == point_last.y) 
    //             || (iter->find("x")->second == point_cur.x && iter->find("y")->second == point_cur.y))
    //             {
    //                 continue;
    //             }
    //             else
    //             {   
    //                 Vector3 point_Act{iter->find("x")->second,iter->find("y")->second,0};
    //                 if(PointinTriangle(point_Last,point_Cur,point_Next,point_Act))
    //                 {
    //                     if_point_in_triangle = true;
    //                 }
    //             }
    //         }
    //     }

    //     std::cout<<if_point_in_triangle<<std::endl;

    //     cor_numb = 0;
    //     int myqueue_size = if_special_polygen.size();
    //     for(int i = 0; i < myqueue_size; i++) 
    //     {   
    //         if( if_special_polygen.front() == key_que[i] )
    //         {
    //             cor_numb++;
    //         }
    //         if_special_polygen.push(if_special_polygen.front());
    //         if_special_polygen.pop();        
    //     }

    //     if( cor_numb == 3 && if_point_in_triangle == false)
    //     {
    //         std::cout<<"***************sssssssssssssssssssssssssssssssssssssssssss****************"<<std::endl;
    //         std::vector<point_c> new_triangle;                                      //加入的新三角形

    //         riter =points.rbegin();
    //         riter++;

    //         point_last_2.x = riter->find("x")->second;
    //         point_last_2.y = riter->find("y")->second;

    //         new_triangle.push_back(point_last_2);
    //         new_triangle.push_back(point_last);
    //         new_triangle.push_back(point_cur);
    //         vec_new_polygon.push_back(new_triangle);

    //         points.pop_front();
    //         start_flag = false;
            
    //     }

    //     ///没出现内凹现象就移动容器
    //     points.push_back(points.front());
    //     points.pop_front();
        
    //     circle_num--;

    //     if(circle_num == 0)
    //     {
    //         std::cout<<"****************77777777777777777777777777****************"<<std::endl;
    //         std::vector<point_c> new_triangle;                                                   //加入的新多边形 

    //         for(iter = points.begin();iter != points.end();iter++)                               //將剩下的点放进一个容器
    //         {
    //             point_c new_point;
    //             new_point.x = iter->find("x")->second;
    //             new_point.y = iter->find("y")->second;
    //             new_triangle.push_back(new_point);
    //         }
    //         vec_new_polygon.push_back(new_triangle);
    //         break;
    //     }
    // }


    //////////////////////////////////////////////////////////////////////////////////////////////////   第二轮处理