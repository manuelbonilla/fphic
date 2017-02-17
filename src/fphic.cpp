#include <fphic/fphic.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>

#include <math.h>

namespace fphic_controllers
{
fphic::fphic() {}
fphic::~fphic() {}

bool fphic::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
	KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

	jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
	id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
	fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
	wrench_subscriber = nh_.subscribe("force_meassurements", 1, &fphic::WrenchCallback, this);

	qdot_last_.resize(kdl_chain_.getNrOfJoints());
	tau_.resize(kdl_chain_.getNrOfJoints());
	J_.resize(kdl_chain_.getNrOfJoints());
	J_dot_.resize(kdl_chain_.getNrOfJoints());
	J_star_.resize(kdl_chain_.getNrOfJoints());
	// Kp_.resize(kdl_chain_.getNrOfJoints());
	// Ki_.resize(kdl_chain_.getNrOfJoints());
	// Kd_.resize(kdl_chain_.getNrOfJoints());
	// Kx_.resize(kdl_chain_.getNrOfJoints());
	// Dx_.resize(kdl_chain_.getNrOfJoints());
	M_.resize(kdl_chain_.getNrOfJoints());
	
	C_.resize(kdl_chain_.getNrOfJoints());
	G_.resize(kdl_chain_.getNrOfJoints());


	J_last_.resize(kdl_chain_.getNrOfJoints());

	sub_command_ = nh_.subscribe("command", 1, &fphic::command, this); /* A line like this is to read the sensor*/
	sub_command_ws = nh_.subscribe("/force_meassurements", 1, &fphic::WrenchCallback, this);
	sub_command_w = nh_.subscribe("w_des", 1, &fphic::w_desired, this);
	pub_error_ = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
	pub_pose_ = nh_.advertise<std_msgs::Float64MultiArray>("pose", 1000);
	// pub_marker_ = nh_.advertise<visualization_msgs::Marker>("marker", 1000);
	pub_tau_ = nh_.advertise<std_msgs::Float64MultiArray>("tau", 1000);

	wrench_integral_ = Eigen::Matrix<double, 6, 1>::Zero();				//abbiamo dichiarato i wrench nel punto h perchè servono per la callback
	wrench_last_ = Eigen::Matrix<double, 6, 1>::Zero();
	wrench_end_effector_ = Eigen::Matrix<double, 6, 1>::Zero();
	wrench_des_ = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> delta_x = Eigen::Matrix<double, 6, 1>::Zero();
	// Eigen::Matrix<double,6,1> G_ = Eigen::Matrix<double,6,1>::Zero();

	tau_ic_ = Eigen::Matrix<double, 7, 1>::Zero();
	tau_fc_ = Eigen::Matrix<double, 7, 1>::Zero();
	tau_m_ = Eigen::Matrix<double, 7, 1>::Zero();
	qdot_m = Eigen::Matrix<double, 7, 1>::Zero();

	Kx_ = Eigen::Matrix<double, 6, 6>::Identity();
	Kp_ = Eigen::Matrix<double, 6, 6>::Identity();
	Ki_ = Eigen::Matrix<double, 6, 6>::Identity();
	Kd_ = Eigen::Matrix<double, 6, 6>::Identity();
	Dx_ = Eigen::Matrix<double, 6, 6>::Identity();



	return true;
}



//funzione che ritorna la misura acquisita dal sensore
void fphic::WrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &wrenchstamped_msg)
{
	wrench_last_ = wrench_end_effector_;
	wrench_end_effector_(0, 0) = wrenchstamped_msg->wrench.force.x;
	wrench_end_effector_(1, 0) = wrenchstamped_msg->wrench.force.y;
	wrench_end_effector_(2, 0) = wrenchstamped_msg->wrench.force.z;
	wrench_end_effector_(3, 0) = wrenchstamped_msg->wrench.torque.x;
	wrench_end_effector_(4, 0) = wrenchstamped_msg->wrench.torque.y;
	wrench_end_effector_(5, 0) = wrenchstamped_msg->wrench.torque.z;
	// wrench_end_effector.request.body_name = wrenchstamped_msg->header.frame_id;
	// wrench_end_effector.request.wrench = wrenchstamped_msg.wrench
	ROS_DEBUG_STREAM("Getting force measurements");

	if (wrench_end_effector_(0, 0) >= 0.5 || wrench_end_effector_(1, 0) >= 0.5 || wrench_end_effector_(2, 0) >= 0.5 || wrench_end_effector_(0, 0) <= -0.5 || wrench_end_effector_(1, 0) <= -0.5 || wrench_end_effector_(2, 0) <= -0.5)
		cmd_flag_ = 1;
}


void fphic::starting(const ros::Time& time)
{
	// get joint positions
	for (int i = 0; i < joint_handles_.size(); i++)
	{
		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
		joint_des_states_.q(i) = joint_msr_states_.q(i);
		joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);
	}

	I_ = Eigen::Matrix<double, 7, 7>::Identity(7, 7);
	e_ref_ = Eigen::Matrix<double, 6, 1>::Zero();

	first_step_ = 0;
	cmd_flag_ = 0;
	step_ = 0;
	fk_pos_solver_->JntToCart(joint_msr_states_.q, x_des_);


}

void fphic::update(const ros::Time& time, const ros::Duration& period)
{

	double k_x_param = 1000.0, d_x_param = 0.3, k_p_param = 100.0, k_i_param = 100.0;

	nh_.param<double>("kx", k_x_param, 1000.0);
	nh_.param<double>("dx", d_x_param, 0.3);
	nh_.param<double>("kp", k_p_param, 100.0);
	nh_.param<double>("ki", k_i_param, 100.0);


	/* This function should be adapted to include the controller written in the paper*/
	// get joint positions
	for (int i = 0; i < joint_handles_.size(); i++)
	{
		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();

		qdot_m(i, 0) = joint_msr_states_.qdot(i);

	}




	// clearing msgs before publishing
	msg_err_.data.clear();
	msg_pose_.data.clear();
	// if (cmd_flag_)
	// {
		// resetting N and tau(t=0) for the highest priority task
		N_trans_ = I_;
		SetToZero(tau_);

		// computing Inertia, Coriolis and Gravity matrices
		id_solver_->JntToMass(joint_msr_states_.q, M_);
		id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
		id_solver_->JntToGravity(joint_msr_states_.q, G_);
		// G_.data.setZero();

		// computing the inverse of M_ now, since it will be used often
		pseudo_inverse(M_.data, M_inv_, false); //M_inv_ = M_.data.inverse();


		// computing Jacobian J(q)
		jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

		// computing the distance from the mid points of the joint ranges as objective function to be minimized
		phi_ = task_objective_function(joint_msr_states_.q);

		// using the first step to compute jacobian of the tasks
		if (first_step_)
		{
			J_last_ = J_;
			phi_last_ = phi_;
			first_step_ = 0;
			return;
		}

		// computing the derivative of Jacobian J_dot(q) through numerical differentiation
		J_dot_.data = (J_.data - J_last_.data) / period.toSec();

		// computing forward kinematics
		fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

		// if (Equal(x_, x_des_, 0.05))
		// {
		// 	ROS_INFO("On target");
		// 	cmd_flag_ = 0;
		// 	return;
		// }

		// pushing x to the pose msg
		for (int i = 0; i < 3; i++)
			msg_pose_.data.push_back(x_.p(i));

		// setting marker parameters
		// set_marker(x_, msg_id_);

		// computing end-effector position/orientation error w.r.t. desired frame
		x_err_ = diff(x_, x_des_);

		delta_x(0, 0) = x_err_(0);
		delta_x(1, 0) = x_err_(1);
		delta_x(2, 0) = x_err_(2);
		delta_x(3, 0) = x_err_(3)*0;
		delta_x(4, 0) = x_err_(4)*0;
		delta_x(5, 0) = x_err_(5)*0;

		x_dot_ = J_.data * joint_msr_states_.qdot.data;

		// setting error reference
		for (int i = 0; i < e_ref_.size(); i++)
		{
			// e = x_des_dotdot + Kd*(x_des_dot - x_dot) + Kp*(x_des - x)
			// e_ref_(i) =  -Kd_(i) * (x_dot_(i)) + Kp_(i) * x_err_(i);
			msg_err_.data.push_back(delta_x(i,0));
		}

		Eigen::Matrix<double, 7, 6> J_t = Eigen::Matrix<double, 7, 6>::Zero();
		J_t = J_.data.transpose();

		wrench_integral_ = wrench_last_ + ((wrench_last_ - wrench_end_effector_) * period.toSec());
		tau_ic_ = 1.0 * J_t * (k_x_param * Kx_ * delta_x - d_x_param * Dx_ * J_.data * qdot_m );//+ G_.data); //+ G_.data);
		tau_fc_ = J_t * (k_p_param * Kp_ * (wrench_end_effector_ - wrench_des_) + k_i_param * Ki_ * wrench_integral_);
		if (cmd_flag_)
		tau_m_ = tau_ic_ + tau_fc_;
		else
		tau_m_ = tau_ic_;

	// }

	// std::cout << "wrench_end_effector_: " << std::endl;

	for (int i = 0; i < 7; i++)
	{
		// if (cmd_flag_)
		// {
			joint_handles_[i].setCommand(tau_m_(i, 0)); /*******/
			// std::cout << wrench_end_effector_(i, 0) << " ";
		// }
		// else
		// {
			// joint_handles_[i].setCommand(0.0);
			// joint_handles_[i].setCommand(PIDs_[i].computeCommand(joint_des_states_.q(i) - joint_msr_states_.q(i),period));
		// }

	}
	// std::cout << std::endl;



	// std::cout << std::endl;
	// // publishing markers for visualization in rviz
	// pub_marker_.publish(msg_marker_);
	// msg_id_++;

	// // publishing error
	pub_error_.publish(msg_err_);
	// // publishing pose
	pub_pose_.publish(msg_pose_);
	std_msgs::Float64MultiArray tau_msg;
	for (int i = 0; i < 7; i++)
	{
		tau_msg.data.push_back(tau_m_(i, 0));
	}
	pub_tau_.publish(tau_msg);
	ros::spinOnce();

}

void fphic::w_desired(const geometry_msgs::Wrench::ConstPtr& msg)
{
	wrench_des_(0, 0) = msg->force.x;
	wrench_des_(1, 0) = msg->force.y;
	wrench_des_(2, 0) = msg->force.z;
	wrench_des_(3, 0) = msg->torque.x;
	wrench_des_(4, 0) = msg->torque.y;
	wrench_des_(5, 0) = msg->torque.z;
	ROS_DEBUG_STREAM("Getting force measurements desired");
}

void fphic::command(const lwr_controllers::PoseRPY::ConstPtr &msg)
{
	KDL::Frame frame_des_;

	switch (msg->id)
	{
	case 0:
		frame_des_ = KDL::Frame(
		                 KDL::Rotation::RPY(msg->orientation.roll,
		                                    msg->orientation.pitch,
		                                    msg->orientation.yaw),
		                 KDL::Vector(msg->position.x,
		                             msg->position.y,
		                             msg->position.z));
		break;

	case 1: // position only
		frame_des_ = KDL::Frame(
		                 KDL::Vector(msg->position.x,
		                             msg->position.y,
		                             msg->position.z));
		break;

	case 2: // orientation only
		frame_des_ = KDL::Frame(
		                 KDL::Rotation::RPY(msg->orientation.roll,
		                                    msg->orientation.pitch,
		                                    msg->orientation.yaw));
		break;

	default:
		ROS_INFO("Wrong message ID");
		return;
	}

	x_des_ = frame_des_;
	// cmd_flag_ = 1;
}

void fphic::set_marker(KDL::Frame x, int id)
{
	msg_marker_.header.frame_id = "world";
	msg_marker_.header.stamp = ros::Time();
	msg_marker_.ns = "end_effector";
	msg_marker_.id = id;
	msg_marker_.type = visualization_msgs::Marker::SPHERE;
	msg_marker_.action = visualization_msgs::Marker::ADD;
	msg_marker_.pose.position.x = x.p(0);
	msg_marker_.pose.position.y = x.p(1);
	msg_marker_.pose.position.z = x.p(2);
	msg_marker_.pose.orientation.x = 0.0;
	msg_marker_.pose.orientation.y = 0.0;
	msg_marker_.pose.orientation.z = 0.0;
	msg_marker_.pose.orientation.w = 1.0;
	msg_marker_.scale.x = 0.005;
	msg_marker_.scale.y = 0.005;
	msg_marker_.scale.z = 0.005;
	msg_marker_.color.a = 1.0;
	msg_marker_.color.r = 0.0;
	msg_marker_.color.g = 1.0;
	msg_marker_.color.b = 0.0;
}

double fphic::task_objective_function(KDL::JntArray q)
{
	double sum = 0;
	double temp;
	int N = q.data.size();

	for (int i = 0; i < N; i++)
	{
		temp = ((q(i) - joint_limits_.center(i)) / (joint_limits_.max(i) - joint_limits_.min(i)));
		sum += temp * temp;
	}

	sum /= 2 * N;

	return -sum;
}
}

PLUGINLIB_EXPORT_CLASS(fphic_controllers::fphic, controller_interface::ControllerBase)
