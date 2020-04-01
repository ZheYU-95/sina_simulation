#include <gazebo_robot/position_controller_cartesian_velocity.h>
#include <pluginlib/class_list_macros.h>

namespace gazebo_robot {
    PositionControllerCartesianVelocity::PositionControllerCartesianVelocity(){
    }

    bool PositionControllerCartesianVelocity::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh) {
        ROS_INFO_ONCE("PositionControllerCartesianVelocity initialising! init lower level controller");

        bool jointGroupPosInit = position_controllers::JointGroupPositionController::init(hw, nh);
        if (jointGroupPosInit) {ROS_INFO_ONCE("initialized position_controller_cartesian_velocity successful!");}

        std::string robotDescriptionString;
        bool paramRead = nh.param("/robot_description", robotDescriptionString, std::string());

        KDL::Tree myTree;
        if (!kdl_parser::treeFromString(robotDescriptionString, myTree)){
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }

        _currentJointPositions = std::vector<double>(n_joints_);
        _targetJointPositions = std::vector<double>(n_joints_);

        std::string rootLink;
        nh.getParam("root_link", rootLink);
        std::string tipLink;
        nh.getParam("tip_link", tipLink);
        myTree.getChain(rootLink, tipLink, _kdlChain);

        KDL::JntArray q_min(n_joints_), q_max(n_joints_);
//        q_min(0) = -2.947; q_min(1) = -2.947; q_min(2) = -2.694; q_min(3) = -2.947; q_min(4) = -2.68; q_min(5) = -2.947;
//        q_max(0) = 2.947; q_max(1) = 2.947; q_max(2) = 2.694; q_max(3) = 2.947; q_max(4) = 2.68; q_max(5) = 2.947;

        _fkSolverPos.reset(new KDL::ChainFkSolverPos_recursive(_kdlChain));
        _ikSolverVel.reset(new KDL::ChainIkSolverVel_pinv_givens(_kdlChain));
        _ikSolverPos.reset(new KDL::ChainIkSolverPos_NR(_kdlChain, *_fkSolverPos, *_ikSolverVel, 150, 0.001));
//        _ikSolverPos.reset(new TRAC_IK::TRAC_IK(_kdlChain, q_min, q_max));
//        _reflexxes.reset(new RosReflexxesPositionInterface(nh.getNamespace()));

        _currentJointPositions = std::vector<double>(n_joints_);
        sub_cartVeloCmd = nh.subscribe<geometry_msgs::Twist>("command_cartesian_velocity", 1, &PositionControllerCartesianVelocity::cartesianVelocityCommand, this);

        return true;
    }

    void PositionControllerCartesianVelocity::starting(const ros::Time& time) {
        updateCurrentJointPositions();
		KDL::Vector pVel = KDL::Vector(0.0, 0.0, 0.0); 	
		KDL::Vector rVel = KDL::Vector(0.0, 0.0, 0.0);
		_targetVelocity = KDL::Twist(pVel, rVel);

//		setReflexxesCurrentJointPositions(_currentJointPositions);

        ROS_INFO("PositionControllerCartesianVelocity started!");
    }

    void PositionControllerCartesianVelocity::update(const ros::Time& time, const ros::Duration& period) {
//        setReflexxesCurrentJointPositions(_currentJointPositions);
//        std::vector<double> targetJointPosition = _reflexxes->update();
//        std::vector<double> targetJointVelocity = _reflexxes->get_current_velocity();

        updateCurrentJointPositions();
		KDL::JntArray currentJointPositions(n_joints_);
        for (int i = 0; i < n_joints_; i++) {
            currentJointPositions(i) = _currentJointPositions[i];
        }
        KDL::JntArray targetJointPositions(n_joints_);

        ros::Time currentTime = ros::Time::now();
        ros::Duration duration = currentTime - _previousTime;
		double duration_sec = duration.toSec();
		int status;
		double limited_time = 0.2;

        if (duration_sec < limited_time){
            getCartesianPoseFromVelocity(duration_sec);
            status = _ikSolverPos->CartToJnt(currentJointPositions, _targetFrame, targetJointPositions);
        } else {
            getCartesianPoseFromVelocity(limited_time);
            status = _ikSolverPos->CartToJnt(currentJointPositions, _targetFrame, targetJointPositions);
        }

        if (status >= 0) {
            for(int i = 0; i < n_joints_; i++) {
                _targetJointPositions[i] = targetJointPositions(i);
                _targetJointPositions[i] = fmod(_targetJointPositions[i], (2 * M_PI));
                if (_targetJointPositions[i] > M_PI){
                    _targetJointPositions[i] -= 2 * M_PI;
                }
                else if (_targetJointPositions[i] < -M_PI){
                    _targetJointPositions[i] += 2 * M_PI;
                }
            }
            ROS_INFO("PositionControllerCartesianVelocity received new base command! \n Target position is (%f, %f, %f, %f, %f, %f).",
                _targetJointPositions[0], _targetJointPositions[1], _targetJointPositions[2], _targetJointPositions[3], _targetJointPositions[4], _targetJointPositions[5]);
        } else {
            ROS_ERROR_ONCE("No valid solution found for (%f, %f, %f, %f, %f, %f).", _targetVelocity.vel(0), _targetVelocity.vel(1), _targetVelocity.vel(2), _targetVelocity.rot(0), _targetVelocity.rot(1), _targetVelocity.rot(2));
        }

        for(int i = 0; i < n_joints_; i++) {
            joints_[i].setCommand(_targetJointPositions[i]);
        }
    }

    void PositionControllerCartesianVelocity::updateCurrentJointPositions() {
        for(unsigned int i=0;i<n_joints_;i++){
            _currentJointPositions[i] = fmod(joints_[i].getPosition(), (2 * M_PI));
            if (_currentJointPositions[i] > M_PI) {
                _currentJointPositions[i] -= 2 * M_PI;
            } else if (_currentJointPositions[i] < -M_PI) {
                _currentJointPositions[i] += 2 * M_PI;
            }
        }
    }

    void PositionControllerCartesianVelocity::cartesianVelocityCommand(const geometry_msgs::Twist::ConstPtr& msg) {
		_previousTime = ros::Time::now();

		if (nullptr == msg){
		    KDL::Vector pVel = KDL::Vector(0.0, 0.0, 0.0);
			KDL::Vector rVel = KDL::Vector(0.0, 0.0, 0.0);
			_targetVelocity = KDL::Twist(pVel, rVel);
		} else {
		    KDL::Vector pVel = KDL::Vector(msg->linear.x, msg->linear.y, msg->linear.z);
			KDL::Vector rVel = KDL::Vector(msg->angular.x, msg->angular.y, msg->angular.z);
			_targetVelocity = KDL::Twist(pVel, rVel);
		}
    }

    void PositionControllerCartesianVelocity::getCartesianPoseFromVelocity(double& period){
        KDL::JntArray currentJointPositions(n_joints_);
        for (int i = 0; i < n_joints_; i++) {
            currentJointPositions(i) = _currentJointPositions[i];
        }
        KDL::Frame currentFrame;
        _fkSolverPos->JntToCart(currentJointPositions, currentFrame);

        KDL::Vector vec = KDL::Vector(_targetVelocity.vel(0), _targetVelocity.vel(1), _targetVelocity.vel(2));
        _targetFrame.p = currentFrame.p + vec * period;
        double roll, pitch, yaw;
        currentFrame.M.GetRPY(roll, pitch, yaw);
        roll = roll + _targetVelocity.rot(0) * period;
        pitch = pitch + _targetVelocity.rot(1) * period;
        yaw = yaw + _targetVelocity.rot(2) * period;
        _targetFrame.M = KDL::Rotation::EulerZYX(yaw, pitch, roll);
    }

//    void PositionControllerCartesianVelocity::setReflexxesCurrentJointPositions(std::vector<double> currentJointPositions) {
//        _reflexxes->starting(currentJointPositions);
//        _reflexxes->set_target_position(_targetJointPositions);
//        _reflexxes->set_target_velocity(_targetJointVelocities);
//    }

}
PLUGINLIB_EXPORT_CLASS(gazebo_robot::PositionControllerCartesianVelocity, controller_interface::ControllerBase)
