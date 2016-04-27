#ifndef BASKETBALL_MARK_SRV
#define BASKETBALL_MARK_SRV

#include <basketball_maker_drawing/basketball_draw_marker.h>
#include <basketball_msgs/basketball_position.h>
#include <basketball_msgs/mark_post_position.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class RobotMarker
{
public:
    RobotMarker(ros::NodeHandle &node) ;
    ~RobotMarker() ;
protected:
private:
    //rviz更新定时器，回调时发送marker到rviz
    void drawMarkerTimerCallBack(const ros::TimerEvent &event) ;
    //画路径的回调函数
    void pathCallBack(const nav_msgs::PathConstPtr &ptr) ;
    //画球的回调函数
    void ballPositionCallBack(const basketball_msgs::basketball_positionConstPtr &ptr) ;
    //画柱子的回调函数
    void cylinderPositionCallBack(const basketball_msgs::mark_post_positionConstPtr &ptr) ;
    //坐标转换
    void transformPoint(const geometry_msgs::Point &source_point ,
                         geometry_msgs::Point &target_point) ;

    void waitForTF() ;
private:
    BasketballDrawMarker *draw_marker_ ;

    ros::Subscriber path_sub_ ;
    ros::Subscriber ball_pos_sub_ ;
    ros::Subscriber cylinder_sub_ ;
    ros::Timer draw_mark_timer_ ;

    ros::NodeHandle nh_ ;

    geometry_msgs::PoseStamped current_robot_pos_ ;

    tf::TransformListener tf_ ;

    string target_frame_ ;
    string source_frame_ ;
    double draw_rate_ ;
} ;


#endif // BASKETBALL_MARK_SRV
