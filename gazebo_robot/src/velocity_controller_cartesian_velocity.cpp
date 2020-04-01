#include <gazebo_robot/velocity_controller_cartesian_velocity.h>
#include <pluginlib/class_list_macros.h>

namespace gazebo_robot {
    VelocityControllerCartesianVelocity::VelocityControllerCartesianVelocity(){
    }

    bool VelocityControllerCartesianVelocity::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) {
        ROS_INFO_ONCE("VelocityControllerCartesianVelocity initialising! init lower level controller");
        bool jointGroupVelInit = velocity_controllers::JointGroupVelocityController::init(hw, nh);

        std::string robotDescriptionString;
        bool paramRead = nh.param("/robot_description", robotDescriptionString, std::string());
        KDL::Tree myTree;
        if (!kdl_parser::treeFromString(robotDescriptionString, myTree)){
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }

        std::string rootLink;
        nh.getParam("root_link", rootLink);
        std::string tipLink;
        nh.getParam("tip_link", tipLink);
        myTree.getChain(rootLink, tipLink, _kdlChain);

        _ikSolverVel.reset(new KDL::ChainIkSolverVel_pinv(_kdlChain));

        _currentJointPositions = std::vector<double>(n_joints_);

        sub_cartVeloCmd = nh.subscribe<geometry_msgs::Twist>("command_cartesian_velocity", 1, &VelocityControllerCartesianVelocity::cartesianVelocityCommand, this);

        ROS_INFO_ONCE("VelocityControllerCartesianVelocity initialized!");
        return true;
    }

    void VelocityControllerCartesianVelocity::starting(const ros::Time& time) {
        updateCurrentJointPositions();
		KDL::Vector pVel = KDL::Vector(0.0, 0.0, 0.0); 	
		KDL::Vector rVel = KDL::Vector(0.0, 0.0, 0.0);
		_targetVelocity = KDL::Twist(pVel, rVel);
		
        ROS_INFO("VelocityControllerCartesianVelocity started!");
    }

    void VelocityControllerCartesianVelocity::update(const ros::Time& time, const ros::Duration& period) {
        updateCurrentJointPositions();

		KDL::JntArray currentJointPositions(n_joints_);
        KDL::JntArray targetJointVelocities(n_joints_);

        for (int i = 0; i < n_joints_; i++) {
            currentJointPositions(i) = _currentJointPositions[i];
        }
        ros::Time currentTime = ros::Time::now();
        ros::Duration duration = currentTime - _previousTime;
		double duration_sec = duration.toSec();
		int status;

        if (duration_sec < 0.2){
            status = _ikSolverVel->CartToJnt(currentJointPositions, _targetVelocity, targetJointVelocities);
        } else {
            // KDL::Vector pVel = KDL::Vector(0.0, 0.0, 0.0);
		    // KDL::Vector rVel = KDL::Vector(0.0, 0.0, 0.0);
		    // _targetVelocity = KDL::Twist(pVel, rVel);
            for(int i = 0; i < n_joints_; i++) {
                targetJointVelocities(i) = 0.0;
            }
		    status = -1;
        }

        if (status >= 0) {
            for(int i = 0; i < n_joints_; i++) {
                if (targetJointVelocities(i) > 1.745){
                    targetJointVelocities(i) = 1.745;
                }
                else if (targetJointVelocities(i) < -1.745){
                    targetJointVelocities(i) = -1.745;
                }
            }
            ROS_INFO("VelocityControllerCartesianVelocity received new base command! \n Target velocity is (%f, %f, %f, %f, %f, %f).",
                targetJointVelocities(0), targetJointVelocities(1), targetJointVelocities(2), targetJointVelocities(3), targetJointVelocities(4), targetJointVelocities(5));
        } else {
            ROS_ERROR("No valid solution found for (%f, %f, %f, %f, %f, %f).", _targetVelocity.vel(0), _targetVelocity.vel(1), _targetVelocity.vel(2), _targetVelocity.rot(0), _targetVelocity.rot(1), _targetVelocity.rot(2));
        }
        for(int i = 0; i < n_joints_; i++) {
            joints_[i].setCommand(targetJointVelocities(i));
        }
    }

    void VelocityControllerCartesianVelocity::updateCurrentJointPositions() {
        for(unsigned int i=0;i<n_joints_;i++){
            _currentJointPositions[i] = fmod(joints_[i].getPosition(), (2 * M_PI));
            if (_currentJointPositions[i] > M_PI) {
                _currentJointPositions[i] -= 2 * M_PI;
            } else if (_currentJointPositions[i] < -M_PI) {
                _currentJointPositions[i] += 2 * M_PI;
            }
        }
    }

    void VelocityControllerCartesianVelocity::cartesianVelocityCommand(const geometry_msgs::Twist::ConstPtr& msg) {
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
}
PLUGINLIB_EXPORT_CLASS(gazebo_robot::VelocityControllerCartesianVelocity, controller_interface::ControllerBase)
