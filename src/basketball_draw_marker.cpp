//author : rescuer liao
//https://github.com/rescuer-liao
//date : 2016 - 1 - 23
//Team Explorer(rescue robot)
//Team Unware (NWPU Basketball robot)
//this program is used to draw something on the rviz detected by the sensor
//you'd better use this package when debug or in match

#define DEBUG
#include <basketball_maker_drawing/basketball_draw_marker.h>

BasketballDrawMarker::BasketballDrawMarker()
    :nh_("~"),
     marker_id_(0)
{
    nh_.param("basketball_draw_marker_array_topic_name" , marker_array_topic_name_ , string("basketball_marker_array_draw")) ;
    nh_.param("basketball_draw_marker_topic_name" , marker_topic_name_ , string("basketball_marker_draw")) ;
    nh_.param("basketball_draw_marker_namespace" , marker_namespace_ , string("basketball_marker_namespace")) ;

    ball_marker_.ns = marker_namespace_ ;
    trajectory_marker_.ns = marker_namespace_ ;
    cylinder_marker_.ns = marker_namespace_ ;


    ball_marker_.header.frame_id = "odom" ;
    cylinder_marker_.header.frame_id = "odom" ;
    trajectory_marker_.header.frame_id = "odom" ;

    ball_marker_.type = visualization_msgs::Marker::SPHERE ;
    trajectory_marker_.type = visualization_msgs::Marker::LINE_STRIP ;
    cylinder_marker_.type = visualization_msgs::Marker::CYLINDER ;

    ball_marker_.pose.orientation.w = 1.0 ;
    cylinder_marker_.pose.orientation.w = 1.0 ;
    trajectory_marker_.pose.orientation.w = 1.0 ;

    setBallColor();
    setBallScale();
    setCylinderColor();
    setCylinderScale();
    setTrajectoryColor();
    setTrajectoryScale();

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_name_ , 10) ;
    markarray_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_array_topic_name_,10) ;

}

BasketballDrawMarker::~BasketballDrawMarker()
{
	//to do : delete all the maker has been drawn in rviz
}

void BasketballDrawMarker::setTrajectoryScale(float x_scale, float y_scale, float z_scale)
{
    trajectory_marker_.scale.x = x_scale ;
    trajectory_marker_.scale.y = y_scale ;
    trajectory_marker_.scale.z = z_scale ;
}

void BasketballDrawMarker::setTrajectoryColor(float r, float g, float b, float a)
{
    trajectory_marker_.color.r = r ;
    trajectory_marker_.color.g = g ;
    trajectory_marker_.color.b = b ;
    trajectory_marker_.color.a = a ;
}

void BasketballDrawMarker::setBallColor(float r, float g, float b, float a)
{
    ball_marker_.color.r = r ;
    ball_marker_.color.g = g ;
    ball_marker_.color.b = b ;
    ball_marker_.color.a = a ;
}

void BasketballDrawMarker::setBallScale(float x_scale, float y_scale, float z_scale)
{
    ball_marker_.scale.x = x_scale ;
    ball_marker_.scale.y = y_scale ;
    ball_marker_.scale.z = z_scale ;
}

void BasketballDrawMarker::setCylinderColor(float r, float g, float b, float a)
{
    cylinder_marker_.color.r = r ;
    cylinder_marker_.color.g = g ;
    cylinder_marker_.color.b = b ;
    cylinder_marker_.color.a = a ;
}

void BasketballDrawMarker::setCylinderScale(float x_scale, float y_scale, float z_scale)
{
    cylinder_marker_.scale.x = x_scale ;
    cylinder_marker_.scale.y = y_scale ;
    cylinder_marker_.scale.z = z_scale ;
}


void BasketballDrawMarker::drawBall(const geometry_msgs::Point &ball_point, string ball_description, bool need_del_last)
{
	//only for test ~~
    if((need_del_last)&&(!ball_drawed_id_stack_.empty()))
    {
        int del_id = ball_drawed_id_stack_.top() ;
        ball_drawed_id_stack_.pop();
        delBall(del_id);
    }
    marker_id_++ ;
    ball_drawed_id_stack_.push(marker_id_);
    ball_marker_.id = marker_id_ ;
    ball_marker_.header.stamp = ros::Time::now() ;
    ball_marker_.action = visualization_msgs::Marker::ADD ;
    ball_marker_.text = ball_description ;
    ball_marker_.pose.position = ball_point ;
    marker_array_.markers.push_back(ball_marker_);
    ROS_INFO("draw ball") ;
}

void BasketballDrawMarker::drawCylinder(const geometry_msgs::Point &cylinder_point, string cylinder_description, bool need_del_last)
{
	//only for test ~~
    if((need_del_last)&&(!cylinder_drawed_id_stack_.empty()))
    {
        int del_id = cylinder_drawed_id_stack_.top() ;
        cylinder_drawed_id_stack_.pop();
        delCylinder(del_id);
    }
    marker_id_++ ;
    cylinder_drawed_id_stack_.push(marker_id_);
    cylinder_marker_.id = marker_id_ ;
    cylinder_marker_.header.stamp = ros::Time::now() ;
    cylinder_marker_.action = visualization_msgs::Marker::ADD ;
    cylinder_marker_.text = cylinder_description ;
    cylinder_marker_.pose.position = cylinder_point ;
    marker_array_.markers.push_back(cylinder_marker_) ;
    ROS_INFO("draw cylinder") ;
}

void BasketballDrawMarker::drawTrajectory(const nav_msgs::Path &trajectory)
{
    marker_id_++ ;
    trajectory_marker_.id = marker_id_ ;
    trajectory_marker_.action = visualization_msgs::Marker::ADD ;
    trajectory_marker_.points.clear() ;
    for(size_t i = 0 ; i < trajectory.poses.size() ; i++)
    {
	if(trajectory.poses.back().header.stamp.toSec() >= last_trajectory_poiont_time_.toSec())
	{
        	trajectory_marker_.points.push_back(trajectory.poses[i].pose.position);
	}
    }
    trajectory_marker_.header.stamp = ros::Time::now() ;
    last_trajectory_poiont_time_ = trajectory.poses.back().header.stamp ;
    marker_array_.markers.push_back(trajectory_marker_);
    ROS_INFO("draw trajectory") ;
}

void BasketballDrawMarker::drawAll()
{
    if(markarray_pub_.getNumSubscribers()>0)
    {
        all_marker_array_.markers.insert(all_marker_array_.markers.end() ,
                                        marker_array_.markers.begin(),
                                        marker_array_.markers.end()) ;
        //save all marker to the all markers array , will be use when delete all marker
        markarray_pub_.publish(marker_array_) ;
        marker_array_.markers.clear();
    }
}

//clear all draw on the rviz
void BasketballDrawMarker::clearAll()
{
    for(size_t i = 0 ; i < all_marker_array_.markers.size() ; i++)
    {
        all_marker_array_.markers[i].action = visualization_msgs::Marker::DELETE ;
    }
    markarray_pub_.publish(all_marker_array_) ;
    all_marker_array_.markers.clear();
}

void BasketballDrawMarker::delBall(int id)
{
    ball_marker_.header.stamp = ros::Time::now() ;
    ball_marker_.id = id ;
    ball_marker_.action = visualization_msgs::Marker::DELETE ;
    marker_pub_.publish(ball_marker_) ;
}

void BasketballDrawMarker::delCylinder(int id)
{
    cylinder_marker_.header.stamp = ros::Time::now() ;
    cylinder_marker_.id = id ;
    cylinder_marker_.action = visualization_msgs::Marker::DELETE ;
    markarray_pub_.publish(cylinder_marker_) ;
}
