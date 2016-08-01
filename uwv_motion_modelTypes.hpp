#ifndef uwv_motion_model_TYPES_HPP
#define uwv_motion_model_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */
#include <base/samples/RigidBodyAcceleration.hpp>
#include <base/NamedVector.hpp>

namespace uwv_motion_model
{
	struct RigidBodyEfforts
	{
		base::Vector6d values;
		base::Time time;
	};

	struct SecondaryStates
	{
		base::samples::RigidBodyAcceleration linearAcceleration;
		base::samples::RigidBodyAcceleration angularAcceleration;
		RigidBodyEfforts					 efforts;
	};

	enum ControlMode
	{ PWM, RPM, EFFORT };
	
	struct ActuatorIDs : public base::NamedVector<int> {
	};
}

#endif

