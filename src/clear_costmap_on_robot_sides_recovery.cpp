/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <clear_costmap_on_robot_sides_recovery/clear_costmap_on_robot_sides_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(clear_costmap_on_robot_sides_recovery, clearCostmapOnRobotSidesRecovery, clear_costmap_on_robot_sides_recovery::clearCostmapOnRobotSidesRecovery, nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace clear_costmap_on_robot_sides_recovery {
clearCostmapOnRobotSidesRecovery::clearCostmapOnRobotSidesRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false) {}

void clearCostmapOnRobotSidesRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);

    private_nh.param("reset_distance", reset_distance_, 3.0);

    std::vector<std::string> clearable_layers_default, clearable_layers;
    clearable_layers_default.push_back( std::string("obstacles") );
    private_nh.param("layer_names", clearable_layers, clearable_layers_default);

    for(unsigned i=0; i < clearable_layers.size(); i++) {
        ROS_INFO("Recovery behavior will clear layer %s", clearable_layers[i].c_str());
        clearable_layers_.insert(clearable_layers[i]);
    }


    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void clearCostmapOnRobotSidesRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the clearCostmapOnRobotSidesRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Clearing the sides of the robot...");

  clear(global_costmap_);

  clear(local_costmap_);

}

void clearCostmapOnRobotSidesRecovery::clear(costmap_2d::Costmap2DROS* costmap){
  std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();


  tf::StampedTransform pose_laser_center_link;
  tf::StampedTransform transformInLaserCenterLink;


  try{

      // will transform data in the goal_frame into the planner_frame_
      tf_->waitForTransform( costmap->getGlobalFrameID(),"laser_center_link", ros::Time::now(), ros::Duration(0.20));
      tf_->lookupTransform( costmap->getGlobalFrameID(),"laser_center_link", ros::Time::now(), pose_laser_center_link);

      tf_->waitForTransform( "laser_center_link", costmap->getGlobalFrameID(),ros::Time::now(), ros::Duration(0.20));
      tf_->lookupTransform( "laser_center_link", costmap->getGlobalFrameID(), ros::Time::now(), transformInLaserCenterLink);

  }
  catch(tf::TransformException){

      ROS_ERROR("Failed to transform the given pose in the recovery behaviour");
      return;
  }


  double x = pose_laser_center_link.getOrigin().x();
  double y = pose_laser_center_link.getOrigin().y();
  double theta = tf::getYaw(pose_laser_center_link.getRotation());

  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
    boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
    std::string name = plugin->getName();
    int slash = name.rfind('/');
    if( slash != std::string::npos ){
        name = name.substr(slash+1);
    }

    if(clearable_layers_.count(name)!=0){
      boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
      costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
      clearMap(costmap, x, y, theta, transformInLaserCenterLink);
    }
  }
}


void clearCostmapOnRobotSidesRecovery::clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap,
                                        double pose_x, double pose_y, double theta, tf::StampedTransform tf_laserlink){

  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));


  unsigned char* grid = costmap->getCharMap();

  for(int x=0; x<(int)costmap->getSizeInCellsX(); x++){
    for(int y=0; y<(int)costmap->getSizeInCellsY(); y++){

      // 0. transform the grid point into the laser_center_link frame
      double x_point_in_costmap, y_point_in_costmap;
      costmap->mapToWorld(x, y, x_point_in_costmap, y_point_in_costmap);
      tf::Point p(x_point_in_costmap, y_point_in_costmap, 0);
      tf::Point point_in_laser_center_link = tf_laserlink*p;
      double xp = point_in_laser_center_link.getX();
      double yp = point_in_laser_center_link.getY();
      // 1. Clear only points in a box around the laser_center_link
      if( std::abs(xp)<0.2 && std::abs(yp)<1.2){

        int index = costmap->getIndex(x,y);
        grid[index] = FREE_SPACE;
      }

    }
  }

  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
  return;


}

};
