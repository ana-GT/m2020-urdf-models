#include <m2020_urdf_models/mhs_gazebo_plugin.h>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(MHSPlugin)


/////////////////////////////////////////////////
MHSPlugin::MHSPlugin()
{
}

/////////////////////////////////////////////////
void MHSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->model_ = _model;

  this->rosNode.reset(new ros::NodeHandle("mhs_gazebo_plugin"));
  
  this->blade_vel_ = 251.32;

  ROS_WARN_STREAM("************** MHSPlugin " << this->model_->GetName());

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&MHSPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

  ignition::math::Vector3d target_pos(0.0,0,2.5);
  ignition::math::Quaterniond target_rot(1.0,0.0,0.0,0.0);
  target_rot.Normalize();
  ignition::math::Pose3d target_pose(target_pos,target_rot);
  this->model_->SetWorldPose(target_pose);

  this->model_->GetJoint("MHS_TopBlades_v16")->SetVelocity(0, blade_vel_);
  this->model_->GetJoint("MHS_BottomBlades_v16")->SetVelocity(0, -blade_vel_);
  this->model_->GetJoint("Joint_Leg01_Axis")->SetPosition(0, 0);
  this->model_->GetJoint("Joint_Leg02_Axis")->SetPosition(0, 0);
  this->model_->GetJoint("Joint_Leg03_Axis")->SetPosition(0, 0);
  this->model_->GetJoint("Joint_Leg04_Axis")->SetPosition(0, 0);

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  ROS_WARN_STREAM("************ MHSPlugin() -- subscribing to command topic"); 

  // this->rosVelCmdSub = this->rosNode->subscribe("/base_controller/command", 10,
                                    // &MHSPlugin::OnRosVelCmdMsg, this);

  this->rosPoseSub = this->rosNode->subscribe("/base_controller/command/pose", 10,
                                    &MHSPlugin::OnRosPoseCmdMsg, this);
}

/////////////////////////////////////////////////
void MHSPlugin::Reset()
{
  this->lastUpdate = ros::Time(0);
}


/////////////////////////////////////////////////
void MHSPlugin::OnRosVelCmdMsg(const geometry_msgs::TwistConstPtr &_msg)
{
  ROS_INFO_STREAM_THROTTLE(1.0, "MHSPlugin::OnRosVelCmdMsg() -- setting model state for " << this->model_->GetName());

  // Link velocity instantaneously without applying forces
  // this->model_->SetLinearVel({_msg->linear.x, _msg->linear.y, _msg->linear.z});
  // this->model_->SetAngularVel({_msg->angular.x, _msg->angular.y, _msg->angular.z});

  ignition::math::Pose3d orig_pose = this->model_->WorldPose();
  ignition::math::Pose3d new_pose = orig_pose;

  ros::Time now = ros::Time::now();
  dt_ = (now - this->lastUpdate).toSec();

  double new_x, new_y, new_z, new_roll, new_pitch, new_yaw;

  if (fabs(_msg->linear.x) < 1e-5)
  {
    new_x = orig_pose.X();
  }
  else
  {
    new_x = orig_pose.X() + _msg->linear.x*dt_;
  }
  if (fabs(_msg->linear.y) < 1e-5) 
  {
    new_y = orig_pose.Y();
  }
  else
  {
    new_y = orig_pose.Y() + _msg->linear.y*dt_;
  }
  if (fabs(_msg->linear.z) < 1e-5) 
  {
    new_z = orig_pose.Z();
  }
  else
  {
    new_z = orig_pose.Z() + _msg->linear.z*dt_;
  }
  if (fabs(_msg->angular.x) < 1e-5)
  {
    new_roll = orig_pose.Roll();
  }
  else
  {
    new_roll = orig_pose.Roll() + _msg->angular.x*dt_;
  }
  if (fabs(_msg->angular.y) < 1e-5)
  {
    new_pitch = orig_pose.Pitch();
  }
  else
  {
    new_pitch = orig_pose.Pitch() + _msg->angular.y*dt_;
  }
  if (fabs(_msg->angular.z) < 1e-5)
  {
    new_yaw = orig_pose.Yaw();
  }
  else
  {
    new_yaw = orig_pose.Yaw() + _msg->angular.z*dt_;
  }

  new_pose.Set(new_x, new_y, new_z, new_roll, new_pitch, new_yaw);
  ignition::math::Pose3d target_pose(new_pose);
  this->model_->SetWorldPose(target_pose);
  this->lastUpdate = now;
}

/////////////////////////////////////////////////
void MHSPlugin::OnRosPoseCmdMsg(const geometry_msgs::PoseConstPtr &_msg)
{
  ROS_INFO_STREAM_THROTTLE(1.0, "MHSPlugin::OnRosPoseCmdMsg() -- setting model state for " << this->model_->GetName());
  ignition::math::Vector3d target_pos(_msg->position.x,_msg->position.y,_msg->position.z);
  ignition::math::Quaterniond target_rot(_msg->orientation.w,_msg->orientation.x,_msg->orientation.y,_msg->orientation.z);
  target_rot.Normalize();
  ignition::math::Pose3d target_pose(target_pos,target_rot);
  this->model_->SetWorldPose(target_pose);
}

/////////////////////////////////////////////////
void MHSPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  ignition::math::Pose3d pose = this->model_->WorldPose();

  // set the blades and leg info
  this->model_->GetJoint("Joint_Leg01_Axis")->SetPosition(0, 0);
  this->model_->GetJoint("Joint_Leg02_Axis")->SetPosition(0, 0);
  this->model_->GetJoint("Joint_Leg03_Axis")->SetPosition(0, 0);
  this->model_->GetJoint("Joint_Leg04_Axis")->SetPosition(0, 0);
  this->model_->GetJoint("MHS_TopBlades_v16")->SetVelocity(0, blade_vel_);
  this->model_->GetJoint("MHS_BottomBlades_v16")->SetVelocity(0, -blade_vel_);
}