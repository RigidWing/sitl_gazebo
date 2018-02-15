// created by Jonas


#include "gazebo_tether_force_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(TetherForcePlugin)

/////////////////////////////////////////////////
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

  getSdfParam<double>(_sdf,"ropeLength", ropeLength, ropeLength);
  getSdfParam<double>(_sdf,"maxForce", forceConstantA, forceConstantA);
  getSdfParam<double>(_sdf,"forceCoefficient", forceConstantB, forceConstantB);
  getSdfParam<math::Vector3>(_sdf,"anchorLocation", anchorLocation, anchorLocation);

  printf("[TetherPlugin] Rope length:...... %f [m]\r\n",ropeLength);
  printf("[TetherPlugin] Max force:........ %f [N]\r\n",forceConstantA);
  printf("[TetherPlugin] Force Coefficient: %f [-]\r\n",forceConstantB);
  printf("[TetherPlugin] Anchor Location:  <%f, %f, %f>  [m]\r\n",anchorLocation[0],anchorLocation[1],anchorLocation[2]);

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
  math::Vector3 position = this->link->GetWorldPose().pos;              // get the position of the aircraft
  math::Vector3 position_relative = position -= anchorLocation;
  double distance = position_relative.GetLength();                    // Distance between the Plane and the Groundstation
  math::Vector3 velocity = this->model->GetRelativeLinearVel();  // get the velocity of the aircraft
  double speed = velocity.GetLength();
  math::Vector3 normalizedPosition = position_relative.Normalize();   // get direction of the tether

  /* calculate the tether force */
  //forceConstantA = 0.00000000000000000000385749969592783556603468563305;
  //forceConstantB = 50.0;
  //double tetherForce = forceConstantA*exp(forceConstantB*distance/ropeLength);
  double tetherForce = forceConstantA/(1+exp(forceConstantB*(ropeLength-distance)));

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
  if(i++%100==0)
  {
    std::cout << tetherForce << "\t " << velocity.Normalize().operator*(-dragForce) << "\t" << distance  << "\n";
  }

}
