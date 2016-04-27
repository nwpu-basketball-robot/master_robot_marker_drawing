#ifndef BASKETBALL_DRAW_MARKER
#define BASKETBALL_DRAW_MARKER

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <stack>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <string>

using namespace std ;

class BasketballDrawMarker
{
public:
    BasketballDrawMarker() ;
    ~BasketballDrawMarker() ;

    void setTrajectoryScale(float x_scale =0.02, float y_scale =0.02, float z_scale=0.02) ;
    void setBallScale(float x_scale = 0.2 , float y_scale = 0.2, float z_scale =0.2) ;
    void setCylinderScale(float x_scale =0.5, float y_scale =0.5, float z_scale = 1.0) ;
    void setBallColor(float r = 1.0, float g =0.0, float b =0.0, float a = 1.0) ;
    void setCylinderColor(float r = 0.0, float g =0.0, float b = 1.0, float a = 1.0) ;
    void setTrajectoryColor(float r =0.0, float g =1.0, float b =0.0, float a = 1.0) ;

    void drawBall(const geometry_msgs::Point &ball_point , string ball_description , bool need_del_last = false) ;
    void drawTrajectory(const nav_msgs::Path &trajectory) ;
    void drawCylinder(const geometry_msgs::Point &cylinder_point ,string cylinder_description ,bool need_del_last = false) ;

    //将marker_queue中的marker绘制在rviz中
    void drawAll() ;
    //清除所有已经画过的东西
    void clearAll() ;
protected:
private:
    void delBall(int id) ;
    void delCylinder(int id) ;
private:
    ros::NodeHandle nh_ ;

    ros::Publisher marker_pub_ ;
    ros::Publisher markarray_pub_ ;

    visualization_msgs::MarkerArray all_marker_array_ ;
    visualization_msgs::MarkerArray marker_array_ ;
    visualization_msgs::Marker ball_marker_ ;
    visualization_msgs::Marker cylinder_marker_ ;
    visualization_msgs::Marker trajectory_marker_ ;
    uint64_t marker_id_ ;

    stack<int> ball_drawed_id_stack_ ;
    stack<int> cylinder_drawed_id_stack_ ;


    string marker_array_topic_name_ ;
    string marker_topic_name_ ;
    string marker_namespace_ ;
    ros::Time last_trajectory_poiont_time_ ;
} ;

#endif // BASKETBALL_DRAW_MARKER
