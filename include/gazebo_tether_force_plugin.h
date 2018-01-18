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
// #include "gazebo/transport/transport.hh"
#include <math.h>
#include <stdio.h>

#include "gazebo/common/Plugin.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE TetherForcePlugin : public ModelPlugin
  {
  // length of the tether: (constant at the moment) [m]
  protected: double ropeLength;
  // dragConst = 1/2*rho_air*diameter*cDragRope = 0.5*1.2041*0.003*1.3 = 0.002347995
  protected: double dragConst; // []
  // E-Module of the tether:
  protected: double eModule; // [Pa]
  // mass of the aircraft:
  protected: double mass; // [kg]
  // tuning parameters: F = A*exp(B*distance/ropeLength)
  protected: double forceConstantA;   // [N]
  protected: double forceConstantB;   // []
  // counter variable for debugging purpose
  protected: int i;
  // pointer to tether attachement point:
  protected: physics::LinkPtr link_;

  // brief Constructor
  public: TetherForcePlugin();

  // brief Destructor
  public: ~TetherForcePlugin();

  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  // Called by the world update start event
  public: void OnUpdate(const common::UpdateInfo & /*_info*/);

  // Pointer to the model
  private: physics::ModelPtr model;

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

  /// \brief Pointer to link currently targeted by mud joint.
  protected: physics::LinkPtr link;

  /// \brief SDF for this plugin;
  protected: sdf::ElementPtr sdf;
  };
}
#endif
