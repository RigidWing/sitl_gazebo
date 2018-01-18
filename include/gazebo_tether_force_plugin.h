// created by Jonas

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

#include <common.h>

#include "gazebo/common/Plugin.hh"


namespace gazebo {
// Default Value
static constexpr double kDefaultRopeLength		= 100;
static constexpr double kDefaultDragConst		= 0.00234799;
static constexpr double kDefaultEModule			= 121000;
static constexpr double kDefaultMass			= 0.5;
static constexpr double kDefaultForceConstantA	= 1000; // Max force
static constexpr double kDefaultForceConstantB	= 5;    // Steepness of force sigmoid
static const math::Vector3 kDefaultAnchorLocation = math::Vector3(0,0,0); // Default anchor location at the origin

class GAZEBO_VISIBLE TetherForcePlugin : public ModelPlugin {
		public:
			TetherForcePlugin()
			:	ropeLength(kDefaultRopeLength),
				dragConst(kDefaultDragConst),
				eModule(kDefaultEModule),
				mass(kDefaultMass),
				forceConstantA(kDefaultForceConstantA),
				forceConstantB(kDefaultForceConstantB),
				anchorLocation(kDefaultAnchorLocation),
				i(0){
			}
			
			~TetherForcePlugin();
			
			void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
			
			// Called by the world update start event
			void OnUpdate(const common::UpdateInfo & /*_info*/);
		protected:
			double ropeLength;	// [m]	length of the tether: (constant at the moment) [m]
			double dragConst;	// []	dragConst = 1/2*rho_air*diameter*cDragRope = 0.5*1.2041*0.003*1.3 = 0.002347995
			double eModule;		// [Pa]	E-Module of the tether:
			double mass;		// [kg]	mass of the aircraft:
			
			int i;				//	[]	counter variable for debugging purpose
			
			math::Vector3 anchorLocation; // Default anchor location at the origin
			
			// tuning parameters: F = A*exp(B*distance/ropeLength)
			double forceConstantA;	// [N]
			double forceConstantB;	// [N]
			
			// pointer to tether attachement point:
			physics::LinkPtr link_;
			/// \brief Pointer to link currently targeted by mud joint.
			physics::LinkPtr link;
			/// \brief SDF for this plugin;
			sdf::ElementPtr sdf;
		private:
			// Pointer to the model
			physics::ModelPtr model;
			// Pointer to the update event connection
			event::ConnectionPtr updateConnection;
	};
}
#endif
