//Eigen
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


// tf2
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"


class Controller : public rclcpp::Node
{

    protected:
	    std::string ROBOT_MODEL_GROUP = "panda_arm";
	    std::string EE_LINK_ = "racchetta";
	    std::string DESIDERATA = "target_frame";
	    //desiderata è una terna da pubblicare, rappresenta quella in cui l'EE deve portarsi, utile per fare una prova

	    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_cmd_pub_;

	    // il RobotModelLoader serve per caricare il modello da URDF con la libreria MoveIt!
	    // è usato per costruire l'oggetto RobotState kinematic_state_
	    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
	    rclcpp::Node::SharedPtr robot_loader_node_;

	    const moveit::core::JointModelGroup* joint_model_group_;
	    const moveit::core::LinkModel* last_link_;

	    //stato cinematico del robot
	    moveit::core::RobotStatePtr kinematic_state_;

	    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener; 

      rclcpp::TimerBase::SharedPtr q_dot_timer_;
      Eigen::Vector3d reference_point_position_;

      std::vector<std::string> joint_names_={"panda_joint_1",
                                             "panda_joint_2",
                                            "panda_joint_3",
                                             "panda_joint_4",
                                             "panda_joint_5",
                                             "panda_joint_6",
                                             "panda_joint_7"};

      double k;

	  public:
	   Controller(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("Controller", opt)
    	{
    	  
        reference_point_position_ = Eigen::Vector3d::Zero(); //servirà per il calcolo dello jacobiano

	    	robot_loader_node_ = std::make_shared<rclcpp::Node>(
	            "robot_model_loader", rclcpp::NodeOptions(opt).automatically_declare_parameters_from_overrides(true));

	        robot_model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(robot_loader_node_);

	        const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader_->getModel();
	        RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());

	        joint_model_group_ = kinematic_model->getJointModelGroup(ROBOT_MODEL_GROUP);
	        kinematic_state_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(kinematic_model));
	        last_link_ = kinematic_state_->getLinkModel(EE_LINK_);

		    //istanzio elementi per l'uso del topic tf
	        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

	        //publisher per i comandi in velocità
	        velocity_cmd_pub_= this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_velocity__controller/command", 1);

		    //timer per il calcolo della nuova q_dot da fornire al robot
	        q_dot_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(33)),
	                                          std::bind(&Controller::timer_cb, this));


	        k=0.001;
	        
	    }

   
   void timer_cb()
    {
        using namespace std::chrono_literals;

        //valore di TFMessage in cui salvo la trasformata tra la terna target e la terna hand_tcp
        tf2_msgs::msg::TFMessage trasformata_ee; 
        tf2_msgs::msg::TFMessage trasformata_tp; 

        trasformata_ee.transforms.resize(1);
        trasformata_tp.transforms.resize(1);

        //calcolo le trasformate
        try
        {
           trasformata_ee.transforms[0] = tf_buffer->lookupTransform("world",EE_LINK_, tf2::TimePointZero, 5s);
           trasformata_tp.transforms[0] = tf_buffer->lookupTransform("world",DESIDERATA, tf2::TimePointZero, 5s);
        }
        catch (tf2::TransformException& ex)
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "ERRORE NEL CALCOLO DELLE TRASFORMATE " << ex.what());
        }
        
        //dichiaro un vettore di eigen per salvare l'errore in posizione ed orientamento
        Eigen::Matrix<double,6,1> errore;

        //calcolo l'errore in posizione
        errore[0]=trasformata_tp.transforms[0].transform.translation.x-trasformata_ee.transforms[0].transform.translation.x;
        errore[1]=trasformata_tp.transforms[0].transform.translation.y-trasformata_ee.transforms[0].transform.translation.y;
        errore[2]=trasformata_tp.transforms[0].transform.translation.z-trasformata_ee.transforms[0].transform.translation.z;

        //calcolo l'errore in orientamento
        Eigen::Quaterniond quaternion_tp_;
        quaternion_tp_.x()=trasformata_tp.transforms[0].transform.rotation.x;
        quaternion_tp_.y()=trasformata_tp.transforms[0].transform.rotation.y;
        quaternion_tp_.z()=trasformata_tp.transforms[0].transform.rotation.z;
        quaternion_tp_.w()=trasformata_tp.transforms[0].transform.rotation.w;
        quaternion_tp_.normalize();

        Eigen::Quaterniond quaternion_ee_;
        quaternion_ee_.x()=trasformata_ee.transforms[0].transform.rotation.x;
        quaternion_ee_.y()=trasformata_ee.transforms[0].transform.rotation.y;
        quaternion_ee_.z()=trasformata_ee.transforms[0].transform.rotation.z;
        quaternion_ee_.w()=trasformata_ee.transforms[0].transform.rotation.w;
        quaternion_ee_.normalize();

        
        //modificare questa parte
        Eigen::Quaterniond  delta_q=quaternion_tp_*quaternion_ee_.inverse();
        errore[3]=delta_q.vec().x(); 
        errore[4]=delta_q.vec().y();
        errore[5]=delta_q.vec().z();

        //calcolo lo jacobiano
        Eigen::MatrixXd jacobian;
        kinematic_state_->getJacobian(joint_model_group_, last_link_, reference_point_position_, jacobian);

        // Calcolo q_dot
        Eigen::VectorXd q_dot = k*jacobian.completeOrthogonalDecomposition().solve(errore); 
        //il * dopo il k mi risolve direttamente pinv(J)*e


        //dichiaro un messaggio std_msgs/Float64MultiArray nel cui campo velocity setto la q_dot appena calcolata

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(joint_names_.size());

        for(int i=0; i<(int)joint_names_.size(); i++)
        {
           msg.data[i]=q_dot[i];
        }


        RCLCPP_INFO(this->get_logger(), "Publishing: [%f, %f, %f, %f, %f, %f, %f]",
                    msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6]);

        // Pubblica il messaggio
        velocity_cmd_pub_->publish(msg);
  	}
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}