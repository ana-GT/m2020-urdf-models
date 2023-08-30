
#ifndef GAZEBO_PLUGINS_MHS_PLUGIN_H_
#define GAZEBO_PLUGINS_MHS_PLUGIN_H_

#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>
#include <gazebo/physics/Joint.hh>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

namespace gazebo
{
  class MHSPlugin : public ModelPlugin
  {
    public:
      MHSPlugin();

      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      virtual void Reset();

    private:
      void OnUpdate(const common::UpdateInfo &_info);

      physics::ModelPtr model_;
      sdf::ElementPtr sdf;
      std::vector<event::ConnectionPtr> connections;
      ignition::math::Vector3d target;
      ros::Time lastUpdate;

      std::default_random_engine rgen_;

    protected:
      void OnRosVelCmdMsg(const geometry_msgs::TwistConstPtr &_msg);
      void OnRosPoseCmdMsg(const geometry_msgs::PoseConstPtr &_msg);

      std::unique_ptr<ros::NodeHandle> rosNode;
      ros::Subscriber rosVelCmdSub, rosPoseSub;

      std::string robot_namespace_;
      std::string pose_input_topic_;
      std::string vel_input_topic_;
      bool use_pose_input_;
      double blade_vel_;
      ignition::math::Vector3d xyz_;
      double noise_;
      double dt_;

  };
}
#endif