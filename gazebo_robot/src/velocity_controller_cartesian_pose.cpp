#include <gazebo_robot/velocity_controller_cartesian_pose.h>
#include <pluginlib/class_list_macros.h>

namespace gazebo_robot {
    VelocityControllerCartesianPose::VelocityControllerCartesianPose(){
    }

    bool VelocityControllerCartesianPose::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) {
        ROS_INFO_ONCE("VelocityControllerCartesianPose initialising! init lower level controller");
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

        _fkSolverPos.reset(new KDL::ChainFkSolverPos_recursive(_kdlChain));
        _ikSolverVel.reset(new KDL::ChainIkSolverVel_pinv_givens(_kdlChain));
        _ikSolverPos.reset(new KDL::ChainIkSolverPos_NR(_kdlChain, *_fkSolverPos, *_ikSolverVel, 150, 0.001));

        _reflexxes.reset(new RosReflexxesPositionInterface( nh.getNamespace()));

        _currentJointPositions = std::vector<double>(n_joints_);
        _targetJointPositions = std::vector<double>(n_joints_);

        sub_cartPosCmd = nh.subscribe<geometry_msgs::Pose>("command_cartesian_pose", 1, &VelocityControllerCartesianPose::cartesianPoseCommand, this);

        ROS_INFO_ONCE("VelocityControllerCartesianPose initialized!");
        return true;
    }

    void VelocityControllerCartesianPose::starting(const ros::Time& time) {
        updateCurrentJointPositions();
        _targetJointPositions = _currentJointPositions;
        setReflexxesCurrentJointPositions(_currentJointPositions);

        ROS_INFO("VelocityControllerCartesianPose started!");
    }

    void VelocityControllerCartesianPose::update(const ros::Time& time, const ros::Duration& period) {
        updateCurrentJointPositions();
        setReflexxesCurrentJointPositions(_currentJointPositions);
        std::vector<double> targetPosition = _reflexxes->update();
        std::vector<double> targetVelocity = _reflexxes->get_current_velocity();

        //ROS_INFO("Target joint velocities: (%f, %f, %f, %f, %f, %f).",
        //targetVelocity[0], targetVelocity[1], targetVelocity[2], targetVelocity[3], targetVelocity[4], targetVelocity[5]);
        for(int i = 0; i < n_joints_; i++) {
            joints_[i].setCommand(targetVelocity[i]);
        }
    }

    void VelocityControllerCartesianPose::updateCurrentJointPositions() {
        for(unsigned int i=0;i<n_joints_;i++){
            _currentJointPositions[i] = fmod(joints_[i].getPosition(), (2 * M_PI));
            if (_currentJointPositions[i] > M_PI) {
                _currentJointPositions[i] -= 2 * M_PI;
            } else if (_currentJointPositions[i] < -M_PI) {
                _currentJointPositions[i] += 2 * M_PI;
            }
        }
    }

    void VelocityControllerCartesianPose::cartesianPoseCommand(const geometry_msgs::Pose::ConstPtr& msg) {
        KDL::Rotation rot = KDL::Rotation::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        KDL::Vector vec = KDL::Vector(msg->position.x, msg->position.y, msg->position.z);
        KDL::Frame tempTargetFrame= KDL::Frame(rot, vec);
        KDL::JntArray targetJointPositions(n_joints_);
        KDL::JntArray currentJointPositions(n_joints_);
        for (int i = 0; i < n_joints_; i++) {
            currentJointPositions(i) = _currentJointPositions[i];
        }
        int status = _ikSolverPos->CartToJnt(currentJointPositions, tempTargetFrame, targetJointPositions);

        if (status == 0) {
            for(int i = 0; i < n_joints_; i++) {
                _targetJointPositions[i] = fmod(targetJointPositions(i), (2 * M_PI));
                if (_targetJointPositions[i] > M_PI) {
                    _targetJointPositions[i] -= 2 * M_PI;
                } else if (_targetJointPositions[i] < -M_PI) {
                    _targetJointPositions[i] += 2 * M_PI;
                }
            }
            ROS_INFO("VelocityControllerCartesianPose recieved new base command! \n Target position is (%f, %f, %f, %f, %f, %f).",
                _targetJointPositions[0], _targetJointPositions[1], _targetJointPositions[2], _targetJointPositions[3], _targetJointPositions[4], _targetJointPositions[5]);
        } else {
            ROS_ERROR("No valid solution found for (%f, %f, %f, %f, %f, %f, %f).",
                msg->position.x, msg->position.y, msg->position.z, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        }
    }

    void VelocityControllerCartesianPose::setReflexxesCurrentJointPositions(std::vector<double> currentJointPositions) {
        _reflexxes->starting(currentJointPositions);
        _reflexxes->set_target_position(_targetJointPositions);
    }
}
PLUGINLIB_EXPORT_CLASS(gazebo_robot::VelocityControllerCartesianPose, controller_interface::ControllerBase)
