// created form Jonas


#include "gazebo_tether_force_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(TetherForcePlugin)

/////////////////////////////////////////////////
TetherForcePlugin::TetherForcePlugin()
{
  this->ropeLength = 100;
  this->dragConst = 0.002347995;
  this->eModule = 121000;
  this->mass = 0.5;
  this->forceConstantB = 50;  //
  this->forceConstantA = 0.0000000000000000000038574996959278355660346856330540251495056653024605258;    // 20*e^(-50)
  this->i=0;
}

TetherForcePlugin::~TetherForcePlugin()
{
}

void TetherForcePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  GZ_ASSERT(_model, "TetherForcePlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "TetherForcePlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;


  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The TetherForcePlugin will not generate forces\n";

    }
    else
    {
      link_ = this->link;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&TetherForcePlugin::OnUpdate, this, _1));
    }
  }
}

// Called by the world update start event
void TetherForcePlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  math::Pose position = this->model->GetWorldPose();             // get the current position of the aircraft
  double distance = position.pos.GetLength();                    // Distance between the Plane and the Groundstation
  math::Vector3 velocity = this->model->GetRelativeLinearVel();  // get the velocity of the aircraft
  double speed = velocity.GetLength();
  math::Vector3 normalizedPosition = position.pos.Normalize();   // get direction of the tether

  /* calculate the tether force */
  double tetherForce = forceConstantA*exp(forceConstantB*distance/ropeLength);

  /* calculate the dragforce */
  // calculate the speed perpendicular to the tether:
  math::Vector3 perpendicularToTether = (normalizedPosition.Cross(velocity.Cross(normalizedPosition))).Normalize();
  double speedPerpendicularToTether = velocity.x*perpendicularToTether.x+velocity.y*perpendicularToTether.y+velocity.z*perpendicularToTether.z;
  // calculate Magnitude of drag force:
  double dragForce = (1/4)*dragConst*speedPerpendicularToTether*speedPerpendicularToTether*distance;

  /* add forces to the link */
  if(tetherForce > 0)
  {
    // add tether force:
    link_->AddForce(normalizedPosition*(-tetherForce));
    // add drag force
    link_->AddForce(velocity.Normalize().operator*(-dragForce));
  }

  // output current informations of the model, only for debugging purpose
  if(i>100)
  {
    std::cout << tetherForce << "\t " << distance  << "\n";
    i=0;
  }
  i++;

}
