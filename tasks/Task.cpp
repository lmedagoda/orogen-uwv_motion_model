/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/JointState.hpp>

using namespace uwv_motion_model;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    gModelParameters = _model_parameters.get();

    int controlOrder 	  = gModelParameters.ctrl_order;
    int numberOfThrusters = gModelParameters.number_of_Thrusters;
    int numberOfCells     = gModelParameters.number_of_cells;
    int numberOfVectoring = gModelParameters.number_of_vectoring;
    int simPerCycle 	  = gModelParameters.sim_per_cycle;
    double samplingTime   = TaskContext::getPeriod();

    // Creating the motion model object
    gMotionModel.reset(new underwaterVehicle::DynamicModel(controlOrder, numberOfThrusters, numberOfCells, numberOfVectoring, samplingTime, simPerCycle));
    gMotionModel->initParameters(gModelParameters);

    // Updating the samplingTime at the component property
    gModelParameters.samplingtime = samplingTime;
    _model_parameters.set(gModelParameters);
    gLastControlInput = base::Time::fromSeconds(0);
    thruster_ids = _a_thruster_ids.value();
    cells_ids = _a_cells_ids.value();
    vectoring_ids = _a_vectoring_ids.value();
    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    base::commands::Joints controlInput;
    base::samples::RigidBodyState states;
    base::samples::RigidBodyState dvl_output;
    dvl_output.invalidate();
    SecondaryStates secondaryStates;
    ControlMode controlMode = _control_mode.get();
    static bool firstRun = true;
    static base::Time dvl_time;
    static bool first_dvl = true;
    static bool first_rng = true;
   
    base::commands::Joints thrustersInput;
    base::commands::Joints cellsInput;
    base::commands::Joints vectoringInput;
    
    if(thruster_ids.size()!=0)
    {
     thrustersInput.elements.resize(thruster_ids.size());
     thrustersInput.names = thruster_ids.names;
    }
    
     if(cells_ids.size()!=0)
    {
     cellsInput.elements.resize(cells_ids.size());
     cellsInput.names = cells_ids.names;
    }
    
    if(vectoring_ids.size()!=0)
    {
     vectoringInput.elements.resize(vectoring_ids.size());
     vectoringInput.names = vectoring_ids.names;
    }

    // Updating control input
    if (_cmd_in.readNewest(controlInput) == RTT::NewData)
    {
     SplitJoints(controlInput, thrustersInput, cellsInput, vectoringInput);

    	// Checking if the control input was properly set
    	if(checkInput(controlInput, thrustersInput, cellsInput, vectoringInput))
    	{
    		if(state() != SIMULATING)
    			state(SIMULATING);

    		// Sending control input
    		switch(controlMode)
    		{
    		case PWM:
    			gMotionModel->sendPWMCommands(thrustersInput);
    			gMotionModel->sendCellsCommands(cellsInput);
    			gMotionModel->sendUpdateStates();
    			break;
    		case RPM:
    			gMotionModel->sendRPMCommands(thrustersInput);
    			gMotionModel->sendCellsCommands(cellsInput);
    			gMotionModel->sendUpdateStates();
    			break;
    		case EFFORT:
    			gMotionModel->sendEffortCommands(controlInput);
    			gMotionModel->sendUpdateStates();
    			break;
    		}

    		// Getting new samplingTime
    		if(gLastControlInput == base::Time::fromSeconds(0))
    			gLastControlInput = controlInput.time;
    		else if ((controlInput.time - gLastControlInput).toSeconds() > 0)
    		{
    		    // Updating the samplingTime at the component property
    			double samplingTime = (controlInput.time - gLastControlInput).toSeconds();
    		    gModelParameters.samplingtime = samplingTime;
    		    _model_parameters.set(gModelParameters);
    			setNewSamplingTime(samplingTime);
    			gLastControlInput = controlInput.time;
    		}


    		// Getting updated states
    		gMotionModel->getPosition(states.position);
    		gMotionModel->getQuatOrienration(states.orientation);
    		gMotionModel->getLinearVelocity(states.velocity);
    		gMotionModel->getAngularVelocity(states.angular_velocity);
    		gMotionModel->getLinearAcceleration(secondaryStates.linearAcceleration.acceleration);
    		gMotionModel->getAngularAcceleration(secondaryStates.angularAcceleration.acceleration);
    		gMotionModel->getEfforts(secondaryStates.efforts.values);

    		// Transforming from euler to axis-angle representation
    		eulerToAxisAngle(states.angular_velocity);

    		// Setting the sample time
    		states.time 				 = controlInput.time.now();
    		secondaryStates.angularAcceleration.time = controlInput.time;
    		secondaryStates.linearAcceleration.time  = controlInput.time;
    		secondaryStates.efforts.time 	         = controlInput.time;

    		// Setting source and target frame names
    		states.sourceFrame 			= _source_frame.get();
    		states.targetFrame 			= _target_frame.get();

    		// Calculating the covariance matrix
    		setUncertainty(states);

    		// Writing the updated states
    		_cmd_out.write(states);
    		_secondary_states.write(secondaryStates);
	
 		//Create simulated DVL dvl_output
		if(first_dvl)
		{
          //std::default_random_engine generator(states.time);
		  first_dvl = false;
		  dvl_time = controlInput.time.now();
          
        }
		else if(controlInput.time.now().toSeconds() - dvl_time.toSeconds() > 0.3) // 3 Hz DVL
		{
		  dvl_output.time = states.time;
		  dvl_output.velocity = states.velocity;
//           dvl_output.velocity.x() = states.velocity.x() + dvl_noise();
//           dvl_output.velocity.y() = states.velocity.y() + dvl_noise();
//           dvl_output.velocity.z() = states.velocity.z() + dvl_noise();
          dvl_output.cov_velocity = 0.0001 * Eigen::Matrix3d::Identity();
		  _dvl_output.write(dvl_output);
		  dvl_time = controlInput.time.now();
		}
		
    	}
    }
    else if(firstRun)
    {
    	firstRun = false;
    	states.initUnknown();
    	_cmd_out.write(states);      
    }
}
bool Task::SplitJoints(base::samples::Joints &controlInput, base::samples::Joints &thrustersInput, base::samples::Joints &cellsInput, base::samples::Joints &vectoringInput)
{
  if(thruster_ids.size()!=0)
     {
    	//Look for the matching thrusters names and copy the joints to thrustersInput
       for( int i = 0; i < controlInput.elements.size() && i < controlInput.names.size(); i++)
        {
	 if( (!base::isNaN(controlInput.elements[i].speed)) || (!base::isNaN(controlInput.elements[i].position)) )
	 {
	  for (size_t ii = 0; ii < thruster_ids.size(); ii++)
	   {
	    if(thrustersInput.names[ii] == controlInput.names[i])
	    {
	     thrustersInput.elements[ii] = controlInput.elements[i];
             thrustersInput.elements[ii].raw = base::unset<double>();
	    }
	   }
	   thrustersInput.time = controlInput.time;
	  }	
        }
      }
      
  if(cells_ids.size()!=0)
  {
  //Look for the matching diving cells names and copy the joints to cellsInput
  for( int i = 0; i < controlInput.elements.size() && i < controlInput.names.size(); i++)
   {
    if( (!base::isNaN(controlInput.elements[i].speed)) || (!base::isNaN(controlInput.elements[i].position)) )
    {
     for (size_t ii = 0; ii < cells_ids.size(); ii++)
     {
      if(cellsInput.names[ii] == controlInput.names[i])
      {
       cellsInput.elements[ii] = controlInput.elements[i];
       cellsInput.elements[ii].raw = base::unset<double>();
      }
     }
      cellsInput.time = controlInput.time;
    }
   }
  }
  
  if(vectoring_ids.size()!=0)
  {
    //Look for the matching vectoring servos names and copy the joints to vectoringInput
    for( int i = 0; i < controlInput.elements.size() && i < controlInput.names.size(); i++)
    {
      if( (!base::isNaN(controlInput.elements[i].speed)) || (!base::isNaN(controlInput.elements[i].position)) )
      {
	for (size_t ii = 0; ii < cells_ids.size(); ii++)
	{
	  if(vectoringInput.names[ii] == controlInput.names[i])
	  {
	    vectoringInput.elements[ii] = controlInput.elements[i];
	    vectoringInput.elements[ii].raw = base::unset<double>();    
	  }  
	}
	vectoringInput.time = controlInput.time;
	
      }
    }
  }
}

bool Task::checkInput(base::samples::Joints &controlInput, base::samples::Joints &thrusterInput, base::samples::Joints &cellsInput, base::samples::Joints &vectoringInput)
{
	ControlMode controlMode = _control_mode.get();
	bool inputError = false;
	
	if(thrusterInput.size() != gModelParameters.number_of_Thrusters)
	{
	  std::cout << "\n\n\x1b[31m (Task: uwv_motion_model.cpp)"
				" Number of thrusters doesn't match the size of thrusters names array.\x1b[0m\n\n";
	  inputError = true;
	  exception(WRONG_SIZE_OF_CONTROL_ELEMENTS);
	}
	
	if(cellsInput.size() != gModelParameters.number_of_cells)
	{
	  std::cout << "\n\n\x1b[31m (Task: uwv_motion_model.cpp)"
				" Number of diving cells doesn't match the size of diving cells names array.\x1b[0m\n\n";
	  inputError = true;
	  exception(WRONG_SIZE_OF_CONTROL_ELEMENTS);
	}
	
	if(vectoringInput.size() != gModelParameters.number_of_vectoring)
	{
	  std::cout << "\n\n\x1b[31m (Task: uwv_motion_model.cpp)"
				" Number of vectoring joints doesn't match the size of vectoring joints names array.\x1b[0m\n\n";
	  inputError = true;
	  exception(WRONG_SIZE_OF_CONTROL_ELEMENTS);
	}

	// Checks if the controlInput size is correct and then if
	// the correspondent field is set
	switch(controlMode)
	{
	case PWM:
		if(controlInput.size() != gModelParameters.ctrl_order)
		{
		  std::cout << "\n\n\x1b[31m (Task: uwv_motion_model.cpp)"
				" Size of control inputs doesn't match the model's control order.\x1b[0m\n\n";
		  inputError = true;
		  exception(WRONG_SIZE_OF_CONTROL_ELEMENTS);
		}
		else
		{
			for (uint i = 0; i < controlInput.size(); i++)
			{
				if (!thrusterInput.elements[i].hasRaw())
				{
					inputError = true;
					if(state() != RAW_FIELD_UNSET)
						error(RAW_FIELD_UNSET);
				}
			}
		}
		break;

	case RPM:
		if(controlInput.size() != gModelParameters.ctrl_order)
		{
		  std::cout << "\n\n\x1b[31m (Task: uwv_motion_model.cpp)"
				" Size of control inputs doesn't match the model's control order.\x1b[0m\n\n";
		  inputError = true;
		  exception(WRONG_SIZE_OF_CONTROL_ELEMENTS);
		}
		else
		{
			for (uint i = 0; i < gModelParameters.number_of_Thrusters; i++)
			{
				if (!thrusterInput.elements[i].hasSpeed())
				{
					inputError = true;
					if(state() != SPEED_FIELD_UNSET)
						error(SPEED_FIELD_UNSET);
				}
			}
		}
		break;

	case EFFORT:
		if(controlInput.size() != 6)
		{
		  std::cout << "\n\n\x1b[31m (Task: uwv_motion_model.cpp)"
				" Size of control inputs should be 6 when using effort inputs.\x1b[0m\n\n";
			inputError = true;
			exception(WRONG_SIZE_OF_CONTROL_ELEMENTS);
		}
		else
		{
			for (uint i = 0; i < controlInput.size(); i++)
			{
				if (!controlInput.elements[i].hasEffort())
				{
					inputError = true;
					if(state() != EFFORT_FIELD_UNSET)
						error(EFFORT_FIELD_UNSET);
				}
			}
		}
		break;
	}

	// If everything is fine, inputError = FALSE and the method returns a TRUE value
	return !inputError;
}

void Task::eulerToAxisAngle(base::Vector3d &states)
{
    Eigen::AngleAxisd axisAngle = Eigen::AngleAxisd(Eigen::AngleAxisd(states(2), Eigen::Vector3d::UnitZ()) *
						    	  Eigen::AngleAxisd(states(1), Eigen::Vector3d::UnitY()) *
								  Eigen::AngleAxisd(states(0), Eigen::Vector3d::UnitX()));

    states = axisAngle.angle() * axisAngle.axis();
}

void Task::setUncertainty(base::samples::RigidBodyState &states)
{
	base::Vector6d velocityUncertainty = _velocity_uncertainty.get();
	static base::Vector6d positionUncertainty = Eigen::VectorXd::Zero(6);
	double samplingTime = TaskContext::getPeriod();


	for(int i = 0; i < 3; i++)
	{
		states.cov_velocity(i,i) = velocityUncertainty[i];
		states.cov_angular_velocity(i,i) = velocityUncertainty[i+3];

		// Euler integration of the velocity uncertainty in order to
		// calculate the position uncertainty
		positionUncertainty[i] += samplingTime*velocityUncertainty[i];
		positionUncertainty[i+3] += samplingTime*velocityUncertainty[i+3];

		states.cov_position(i,i) = positionUncertainty[i];
		states.cov_orientation(i,i) = positionUncertainty[i+3];
	}
}

bool Task::setNewParameters(void)
{
	underwaterVehicle::Parameters uwvParameters = _model_parameters.get();
	return gMotionModel->setUWVParameters(uwvParameters);
}

void Task::setNewSamplingTime(double samplingTime)
{
	gMotionModel->setSamplingTime(samplingTime);
}

void Task::resetStates(void)
{
	gMotionModel->resetStates();
}

void Task::errorHook()
{
    TaskBase::errorHook();

    base::commands::Joints newControlInput;
    
    base::commands::Joints thrustersInput;
    base::commands::Joints cellsInput;
    base::commands::Joints vectoringInput;
    
    if(thruster_ids.size()!=0)
    {
     thrustersInput.elements.resize(thruster_ids.size());
     thrustersInput.names = thruster_ids.names;
    }
    
     if(cells_ids.size()!=0)
    {
     cellsInput.elements.resize(cells_ids.size());
     cellsInput.names = cells_ids.names;
    }
    
    if(vectoring_ids.size()!=0)
    {
     vectoringInput.elements.resize(vectoring_ids.size());
     vectoringInput.names = vectoring_ids.names;
    }

    if (_cmd_in.readNewest(newControlInput) == RTT::NewData)
    {
     SplitJoints(newControlInput, thrustersInput, cellsInput, vectoringInput);
    	// If there's no input error anymore, the system is recovered to operational state
    	if(checkInput(newControlInput, thrustersInput, cellsInput, vectoringInput))
    		recover();
    }
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
