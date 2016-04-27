//author : rescuer liao
//https://github.com/rescuer-liao
//date : 2016 - 1 - 23
//Team Explorer(rescue robot)
//Team Unware (NWPU Basketball robot)
//this program is used to draw something on the rviz detected by the sensor
//you'd better use this package when debug or in match

#include <basketball_maker_drawing/basketball_mark_srv.h>
#include <ros/ros.h>
//#define DEBUG
RobotMarker::RobotMarker(ros::NodeHandle &node)
    :nh_(node),
      draw_marker_(new BasketballDrawMarker())
{
    nh_.param("target_frame_name" , target_frame_ , string("/odom")) ;
    nh_.param("source_frame_name",source_frame_ , string("/base_link")) ;
    nh_.param("draw_marke_rate" , draw_rate_,0.25) ;
    current_robot_pos_.header.frame_id = source_frame_ ;
    current_robot_pos_.pose.orientation.w = 1.0 ;

    waitForTF();

    draw_mark_timer_ = nh_.createTimer(ros::Duration(1/draw_rate_),&RobotMarker::drawMarkerTimerCallBack,this,false) ;
    path_sub_ = nh_.subscribe("/moved_trajectory",1,&RobotMarker::pathCallBack,this) ;
    ball_pos_sub_ = nh_.subscribe("/mark_ball",2,&RobotMarker::ballPositionCallBack , this) ;
    cylinder_sub_ = nh_.subscribe("/mark_cylinder" ,1 , &RobotMarker::cylinderPositionCallBack,this) ;

#ifdef DEBUG
    //this is the test mark ball and mark cylinder
        geometry_msgs::Point p ;
        p.x = 1.0 ;
        p.y = 1.0 ;
        p.z = 1.0 ;
        draw_marker_->drawBall(p,"ds");

        p.x = 0.5 ;
        p.y = 2.0 ;
        p.z = 0.0 ;
        draw_marker_->drawCylinder(p,"dsad");
#endif
}

RobotMarker::~RobotMarker()
{
    delete draw_marker_ ;
    nh_.shutdown();
}

void RobotMarker::waitForTF()
{
    ros::Time start_time = ros::Time::now() ;

    ROS_INFO("[basketball_draw_maker] wait for the transformation between %s and %s become available",
             target_frame_.c_str() , source_frame_.c_str()) ;

    bool get_transform = false ;
    while(!get_transform)
    {
        get_transform = tf_.canTransform(target_frame_,source_frame_,ros::Time()) ;
        if(get_transform)break ;
        ros::Time now = ros::Time::now() ;

        if((now-start_time).toSec() > 20.0)
        {
            ROS_WARN_ONCE("[basketball_draw_maker] no transform frame between %s and %s find after %f seconds of waiting this warning only print once",
                          source_frame_.c_str() , target_frame_.c_str(),(now-start_time).toSec()) ;
        }
    }
    ROS_INFO("[basketball_draw_maker] get the transforom between %s and %s" , source_frame_.c_str() , target_frame_.c_str()) ;
}

void RobotMarker::pathCallBack(const nav_msgs::PathConstPtr &ptr)
{
#ifdef DEBUG
    ROS_INFO("[basketball_draw_maker] get new trajectory") ;
#endif
    draw_marker_->drawTrajectory(*(ptr.get()));

}

void RobotMarker::ballPositionCallBack(const basketball_msgs::basketball_positionConstPtr &ptr)
{
#ifdef DEBUG
    ROS_INFO("[basketball_draw_maker] get ball pub") ;
#endif
    geometry_msgs::Point global_ball_pos ;
    transformPoint(ptr->basketball_position , global_ball_pos);
    draw_marker_->drawBall(global_ball_pos,"test ball");
}

void RobotMarker::cylinderPositionCallBack(const basketball_msgs::mark_post_positionConstPtr &ptr)
{
#ifdef DEBUG
    ROS_INFO("[basketball_draw_maker] get cylinder pub") ;
#endif
    geometry_msgs::Point global_cylinder_pos ;
    transformPoint(ptr->mark_post_pose , global_cylinder_pos);
    draw_marker_->drawCylinder(global_cylinder_pos,"test cylinder");
}

void RobotMarker::transformPoint(const geometry_msgs::Point &source_point,  geometry_msgs::Point &target_point)
{
    geometry_msgs::PoseStamped p_out ;
    current_robot_pos_.header.stamp = ros::Time(0) ;

    tf_.transformPose(target_frame_,current_robot_pos_,p_out) ;

    target_point.x = p_out.pose.position.x+source_point.x ;
    target_point.y = p_out.pose.position.y+source_point.y ;
    target_point.z = p_out.pose.position.z+source_point.z ;
}

void RobotMarker::drawMarkerTimerCallBack(const ros::TimerEvent &event)
{
    draw_marker_->drawAll();
}


int main(int argc , char **argv)
{
    ros::init(argc , argv , "basketball_mark_draw") ;
    ros::NodeHandle node ;
    RobotMarker robot_mark_draw(node) ;
    ros::spin() ;
    return 0 ;
}
