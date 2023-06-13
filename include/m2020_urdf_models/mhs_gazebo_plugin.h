
#ifndef GAZEBO_PLUGINS_MHS_PLUGIN_H_
#define GAZEBO_PLUGINS_MHS_PLUGIN_H_

#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>
#include <gazebo/physics/Joint.hh>

#include <gazebo_msgs/SetModelState.h>

namespace gazebo
{
  class MHSPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: MHSPlugin();

    /// \brief Load the model plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Reset();

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Pointer to the parent model.
    private: physics::ModelPtr model_;

    /// \brief Pointer to the sdf element.
    private: sdf::ElementPtr sdf;

    /// \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief Current target location
    private: ignition::math::Vector3d target;

    /// \brief Time of the last update.
    private: common::Time lastUpdate;


    protected:
      void QueueThread();

      std::unique_ptr<ros::NodeHandle> rosNode;

      /// \brief A ROS callbackqueue that helps process messages
      ros::CallbackQueue rosQueue;

      /// \brief A thread the keeps running the rosQueue
      std::thread rosQueueThread;

      double blade_vel_;
  };
}
#endif