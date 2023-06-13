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
  
  this->blade_vel_ = 133.33;

  ROS_WARN_STREAM("************** MHSPlugin " << this->model_->GetName());

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&MHSPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

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

  // Spin up the queue helper thread.
  // this->rosQueueThread = std::thread(std::bind(&MHSPlugin::QueueThread, this));  

}

/////////////////////////////////////////////////
void MHSPlugin::Reset()
{
  this->lastUpdate = 0;
}


void MHSPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

/////////////////////////////////////////////////
void MHSPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->model_->WorldPose();

  ROS_INFO_STREAM_THROTTLE(1.0, "MHSPlugin::OnUpdate() -- setting model state for " << this->model_->GetName());
  ignition::math::Vector3d target_pos(0.0,0,2.0);
  ignition::math::Quaterniond target_rot(1.0,0.0,0.0,0.0);
  target_rot.Normalize();
  ignition::math::Pose3d target_pose(target_pos,target_rot);
  this->model_->SetWorldPose(target_pose);

  // set the blades and leg info
  this->model_->GetJoint("Joint_Leg01_Axis")->SetPosition(0, 0);
  this->model_->GetJoint("Joint_Leg02_Axis")->SetPosition(0, 0);
  this->model_->GetJoint("Joint_Leg03_Axis")->SetPosition(0, 0);
  this->model_->GetJoint("Joint_Leg04_Axis")->SetPosition(0, 0);
  this->model_->GetJoint("MHS_TopBlades_v16")->SetVelocity(0, blade_vel_);
  this->model_->GetJoint("MHS_BottomBlades_v16")->SetVelocity(0, -blade_vel_);

  this->lastUpdate = _info.simTime;
}