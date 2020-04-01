#ifndef GAZEBO_ROBOT_TEST_CONTROLLER_H
#define GAZEBO_ROBOT_TEST_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <position_controllers/joint_group_position_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Geometry>
#include <math.h>
//#include <ros_reflexxes/RosReflexxesPositionInterface.h>
//#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#ifndef M_PI
#define M_PI    3.14159265358979323846f
#endif


namespace gazebo_robot {
    class PositionControllerCartesianVelocity : public position_controllers::JointGroupPositionController {
        public:
            PositionControllerCartesianVelocity();
            virtual bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh);
            virtual void starting(const ros::Time& time);
            virtual void update(const ros::Time& time, const ros::Duration&);

        private:
            KDL::Chain _kdlChain;
            boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> _fkSolverPos;
            boost::scoped_ptr<KDL::ChainIkSolverVel_pinv_givens> _ikSolverVel;
            boost::scoped_ptr<KDL::ChainIkSolverPos_NR> _ikSolverPos;
//            boost::scoped_ptr<TRAC_IK::TRAC_IK> _ikSolverPos;
//            std::shared_ptr<RosReflexxesPositionInterface> _reflexxes;

            std::vector<double> _currentJointPositions;
            std::vector<double> _targetJointPositions;

//            KDL::JntArray _targetJointVelocities(n_joints_);
	    	KDL::Twist _targetVelocity;
	    	KDL::Frame _targetFrame;
	    	ros::Time _previousTime;

            ros::Subscriber sub_cartVeloCmd;

            void updateCurrentJointPositions();
            void cartesianVelocityCommand(const geometry_msgs::Twist::ConstPtr& msg);
            void getCartesianPoseFromVelocity(double& period);
//            void setReflexxesCurrentJointPositions(std::vector<double> currentJointPositions);
    };//class PositionControllerCartesianVelocity
}//namespace
#endif
