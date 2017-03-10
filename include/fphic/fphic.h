#ifndef FPHIC_H
#define FPHIC_H

#include <lwr_controllers/PIDKinematicChainControllerBase.h>
#include <lwr_controllers/MultiPriorityTask.h>

#include <lwr_controllers/PoseRPY.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>

namespace fphic_controllers
{
	class fphic: public controller_interface::PIDKinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:
		fphic();
		~fphic();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const lwr_controllers::PoseRPY::ConstPtr &msg);
		void WrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &wrenchstamped_msg);
		void w_desired(const geometry_msgs::Wrench::ConstPtr& msg);		
		void set_marker(KDL::Frame x, int id);
		double task_objective_function(KDL::JntArray q);

	private:
		ros::Subscriber sub_command_;
		ros::Subscriber wrench_subscriber, sub_command_ws, sub_command_w;
		ros::Publisher pub_error_;
		ros::Publisher pub_pose_;
		ros::Publisher pub_marker, pub_tau_;

		std_msgs::Float64MultiArray msg_err_;
		std_msgs::Float64MultiArray msg_pose_;
		visualization_msgs::Marker msg_marker_;
		geometry_msgs::WrenchStamped ws;
		std::stringstream sstr_;
        
		KDL::JntArray qdot_last_;

		KDL::Frame x_,x0_;	//current e-e pose
		Eigen::Matrix<double,6,1> x_dot_;	//current e-e velocity

		KDL::Frame x_des_;	//desired pose
		KDL::Twist x_des_dot_;
		KDL::Twist x_des_dotdot_;

		KDL::Twist x_err_;

		Eigen::MatrixXd wrench_integral_ ;
  		Eigen::Matrix<double,6,1> wrench_last_;
		Eigen::Matrix<double,6,1> wrench_end_effector_; 
		Eigen::Matrix<double,6,1> wrench_end_effector_dummy; 
    	Eigen::Matrix<double,6,1> wrench_des_;
    	Eigen::Matrix<double,6,1> delta_x;
    	Eigen::Matrix<double,6,1> force_error, force_error_0;
    	Eigen::Matrix<double,7,1> qdot_m;
    	double f_tresh;
		// KDL::Wrench wrench_des_;
		// KDL::Wrench wrench_last_;
		// KDL::Wrench wrench_integral_;
		// KDL::Wrench wrench_end_effector;


		// KDL::JntArray Kp_,Kd_,Kx_,Ki_, Dx_;

		KDL::JntArray tau_;
		// KDL::JntArray tau_ic_;
		// KDL::JntArray tau_fc_;
		// KDL::JntArray tau_m_;
		

		KDL::JntSpaceInertiaMatrix M_;	// intertia matrix
		KDL::JntArray C_;	// coriolis
		KDL::JntArray G_;	// gravity

		KDL::Jacobian J_;	//Jacobian J(q)
		KDL::Jacobian J_last_;	//Jacobian of the last step
		KDL::Jacobian J_dot_;	//d/dt(J(q))
		KDL::Jacobian J_star_; // it will be J_*P_

		Eigen::MatrixXd J_pinv_;

		Eigen::Matrix<double,6,1> e_ref_;
		Eigen::Matrix<double,7,7> I_;
		Eigen::Matrix<double,7,7> N_trans_;
		Eigen::MatrixXd M_inv_;
		Eigen::MatrixXd omega_;
		Eigen::MatrixXd lambda_;
		Eigen::Matrix<double,6,1> b_;


		Eigen::Matrix<double,7,1> tau_ic_;
 		Eigen::Matrix<double,7,1> tau_fc_;
 		Eigen::Matrix<double,7,1> tau_m_ ;
	    Eigen::Matrix<double,6,6> Kx_;
	    Eigen::Matrix<double,6,6> Kd_;
    	Eigen::Matrix<double,6,6> Kp_ ;
    	Eigen::Matrix<double,6,6> Ki_ ;
    	Eigen::Matrix<double,6,6> Dx_ ;
		Eigen::Matrix<double,6,6> Adg_trans; //matrice aggiunta per la traslazione dei wrenches

		
		double phi_;	
		double phi_last_;

		int step_;
		int first_step_;
		int msg_id_;
		int cmd_flag_;
		int ntasks_;
		bool on_target_flag_;
		int links_index_;
		double f_des;


		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		//boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;
		//boost::scoped_ptr<KDL::ChainFkSolverAcc_recursive> fk_acc_solver_;
	};

}

#endif