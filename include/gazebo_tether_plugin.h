// created form Jonas

#ifndef _GAZEBO_TETHER_FORCE_PLUGIN_HH_
#define _GAZEBO_TETHER_FORCE_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/common/Assert.hh"
#include <algorithm>
#include <string>
#include "gazebo/msgs/msgs.hh"
#include <math.h>
#include <stdio.h>

#include "gazebo/common/Plugin.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE TetherPlugin : public ModelPlugin
  {
  protected:
    math::Vector3 fSpring;
    math::Vector3 fDrag;
    math::Vector3 fTotal;

    double segmentLength;
    double indexEnd;
    double kIndex1;
    double kIndex2;



    // length of the tether: (constant at the moment) [m]
    double ropeLength;
    // dragConst = 1/2*rho_air*diameter*cDragRope = 0.5*1.2041*0.003*1.3 = 0.002347995
    double dragConst; // []
    // E-Module of the tether:
    double eModule; // [Pa]
    // mass of the aircraft:
    double mass; // [kg]
    // tuning parameters: F = A*exp(B*distance/ropeLength)
    double forceConstantA;   // [N]
    double forceConstantB;   // []
    // counter variable for debugging purpose
    int i;
    // pointer to tether attachement point:
    physics::LinkPtr link_;

  public:
    // brief Constructor
    TetherForcePlugin();

    // brief Destructor
    ~TetherForcePlugin();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/);

    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Pointer to link currently targeted by mud joint.
    physics::LinkPtr link;

    //SDF for this plugin;
    sdf::ElementPtr sdf;
  };
}
#endif
