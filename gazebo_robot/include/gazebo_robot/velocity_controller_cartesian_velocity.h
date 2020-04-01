#ifndef GAZEBO_ROBOT_TEST_CONTROLLER_H
#define GAZEBO_ROBOT_TEST_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <velocity_controllers/joint_group_velocity_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Geometry>
#include <math.h>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>

#ifndef M_PI
#define M_PI    3.14159265358979323846f
#endif


namespace gazebo_robot {
    class VelocityControllerCartesianVelocity : public velocity_controllers::JointGroupVelocityController {
        public:
            VelocityControllerCartesianVelocity();
            virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh);
            virtual void starting(const ros::Time& time);
            virtual void update(const ros::Time& time, const ros::Duration&);

        private:
            KDL::Chain _kdlChain;
            boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> _ikSolverVel;
            std::vector<double> _currentJointPositions;
	    	KDL::Twist _targetVelocity;
	    	ros::Time _previousTime;

            ros::Subscriber sub_cartVeloCmd;

            void updateCurrentJointPositions();
            void cartesianVelocityCommand(const geometry_msgs::Twist::ConstPtr& msg);
    };//class VelocityControllerCartesianVelocity
}//namespace
#endif
