#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <fstream>
#include <iostream>
#include <sstream>

#include "std_srvs/Empty.h"
#include "gazebo_msgs/SetModelConfiguration.h"

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

const int dof = 6;
typedef Eigen::Matrix< double, 6, 1 > Vector6d;
typedef Eigen::Matrix< double, 6, 6 > Matrix6d;

void MotorInertiaReflectedCompositeRigidBodyAlgorithm(RigidBodyDynamics::Model &model, Vector6d q, Eigen::MatrixXd &A, bool update_kinematics){

    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model, q, A, update_kinematics);
    A(0,0) += 2.6496;
    A(1,1) += 2.6496;
    A(2,2) += 1.37;
    A(3,3) += 0.6336;
    A(4,4) += 0.6336;
    A(5,5) += 0.63296;
}

// Get C matrix (C*qdot is coriolis+centrifugal torque)
Eigen::MatrixXd getC(RigidBodyDynamics::Model &model, Vector6d q, Vector6d qdot){
    double h = 2e-12;
    Vector6d q_new;
    Eigen::MatrixXd C(dof, dof);
    C.setZero();
    Eigen::MatrixXd C1(dof, dof);
    C1.setZero();
    Eigen::MatrixXd C2(dof, dof);
    C2.setZero();
    Eigen::MatrixXd H_origin(dof, dof), H_new(dof, dof);
    Eigen::MatrixXd m[dof];
    double b[dof][dof][dof];

    for (int i = 0; i < dof; i++)
    {
        H_origin.setZero();
        MotorInertiaReflectedCompositeRigidBodyAlgorithm(model, q, H_origin, true);

        q_new = q;
        q_new(i) += h;
        H_new.setZero();
        MotorInertiaReflectedCompositeRigidBodyAlgorithm(model, q_new, H_new, true);

        m[i].resize(dof, dof);
        m[i] = (H_new - H_origin) / h;
    }

    for (int i = 0; i < dof; i++)
        for (int j = 0; j < dof; j++)
            for (int k = 0; k < dof; k++)
                b[i][j][k] = 0.5 * (m[k](i, j) + m[j](i, k) - m[i](j, k));


    for (int i = 0; i < dof; i++)
        for (int j = 0; j < dof; j++)
            C1(i, j) = b[i][j][j] * qdot(j);

    for (int k = 0; k < dof; k++)
        for (int j = 0; j < dof; j++)
            for (int i = 1 + j; i < dof; i++)
            C2(k, j) += 2.0 * b[k][j][i] * qdot(i);
    C = C1 + C2;

    return C;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyros_reproduce");

    ros::NodeHandle n;

    // Don't use simulation time since we will stop simulation(simulation is used just for rendering)
    ros::param::set("use_sim_time", false);
    ros::Duration(1.0).sleep();

    ros::Rate loop_rate(1000);

    // Service client
    ros::ServiceClient physics_client = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    ros::ServiceClient joint_position_client = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

    // Stop simulation
    std_srvs::Empty pauseSrv;
    physics_client.call(pauseSrv);

    // Load URDF model
    RigidBodyDynamics::Model model;
    RigidBodyDynamics::Addons::URDFReadFromFile("/home/kim/doosan_ws/src/doosan-robot/dsr_description/urdf/m0609_modify.urdf", &model, false, false);

    // Load data file
    std::ifstream openFile("/home/kim/ssd2/dusan_ws/2nd/data/robot1_20190829/free/0_00kg/1/DRCL_Data_1.txt");
    std::string line;
    std::stringstream line_ss;
    float data_arr[100];
    if( !openFile.is_open() ){
        return 0;
    }

    // Request to render robot configuration
    gazebo_msgs::SetModelConfiguration modelconfig;
    modelconfig.request.model_name = (std::string) "dsr01";

    modelconfig.request.urdf_param_name = (std::string) "robot_description";

    modelconfig.request.joint_names.push_back("joint1");
    modelconfig.request.joint_names.push_back("joint2");
    modelconfig.request.joint_names.push_back("joint3");
    modelconfig.request.joint_names.push_back("joint4");
    modelconfig.request.joint_names.push_back("joint5");
    modelconfig.request.joint_names.push_back("joint6");

    modelconfig.request.joint_positions.push_back(0.0);
    modelconfig.request.joint_positions.push_back(0.0);
    modelconfig.request.joint_positions.push_back(0.0);
    modelconfig.request.joint_positions.push_back(0.0);
    modelconfig.request.joint_positions.push_back(0.0);
    modelconfig.request.joint_positions.push_back(0.0);

    // Data and dynamic matrix variables
    Vector6d q;
    Vector6d q_dot;  
    Vector6d q_dot_d;  
    Vector6d q_dot_d_pre;   
    Vector6d q_ddot; 
    Vector6d tau_m;
    Eigen::VectorXd tau_inv_dyn;

    Eigen::MatrixXd A;
    Eigen::MatrixXd C;
    Eigen::MatrixXd C_T;
    Eigen::VectorXd g;    
    Eigen::VectorXd nonlinear;
    Vector6d b;
    Vector6d b_custom;

    Vector6d q_dot_zero;
    q_dot_zero.setZero();


    // Momentum observer variables
    Vector6d integral_term;
    integral_term.setZero();
    Vector6d residual;
    residual.setZero();
    Matrix6d K_mob;
    K_mob.setZero();
    for (int i=0; i<6; i++)
    {
        K_mob(i,i)=10;
    }

    int tick = 0;
    double dt = 0.001;
    double last_time = 0.0;
    std::ofstream result; 
    result.open("/home/kim/doosan_ws/src/doosan-robot/reproduce_dyros/result/result.txt");

    while (ros::ok() && openFile.peek() != EOF)
    {
        int count = 0;
        q.setZero();
        q_dot.setZero();
        A.setZero(6,6);
        C.setZero(6,6);
        C_T.setZero(6,6);
        g.setZero(6);
        nonlinear.setZero(6);
        tau_inv_dyn.setZero(6);
        q_dot_zero.setZero();

        // Read data
        float data;
        getline(openFile, line);
        line_ss.str(line);
        while(line_ss >> data)
        {
            data_arr[count] = data;
            count++;
        }
        line_ss.clear();

        // Allocate data
        q << data_arr[7], data_arr[8], data_arr[9], data_arr[10], data_arr[11], data_arr[12];
        q_dot << data_arr[13], data_arr[14], data_arr[15], data_arr[16], data_arr[17], data_arr[18];
        tau_m << data_arr[1], data_arr[2], data_arr[3], data_arr[4], data_arr[5], data_arr[6];
        q_dot_d << data_arr[25], data_arr[26], data_arr[27], data_arr[28], data_arr[29], data_arr[30];

        // Current dynamics information
        MotorInertiaReflectedCompositeRigidBodyAlgorithm(model, q, A, true);
        C = getC(model, q, q_dot);
        C_T = C.transpose();
        RigidBodyDynamics::NonlinearEffects(model, q, q_dot_zero, g);
        // RigidBodyDynamics::NonlinearEffects(model, q, q_dot, nonlinear);
        // b = nonlinear - g;
        // b_custom = C * q_dot;

        // Momentum observer
        dt = data_arr[0] - last_time;
        if( tick == 0)
            dt = 0.001;

        integral_term = integral_term + (tau_m + C_T*q_dot - g - residual)*dt;
        residual = K_mob * (integral_term - A*q_dot);
        last_time = data_arr[0];

        // Send request to render
        // modelconfig.request.joint_positions[0]= q(0);
        // modelconfig.request.joint_positions[1]= q(1);
        // modelconfig.request.joint_positions[2]= q(2);
        // modelconfig.request.joint_positions[3]= q(3);
        // modelconfig.request.joint_positions[4]= q(4);
        // modelconfig.request.joint_positions[5]= q(5);
        // if (!joint_position_client.call(modelconfig)) ROS_ERROR("Failed to call the service");

        if (tick%100 == 0)
        {
            std::cout << "Time: " << data_arr[0] << std::endl;
            std::cout << "tau_m: " << tau_m(1) << std::endl;
            std::cout << "g: "  << g(1) << std::endl<< std::endl;
        }
        // loop_rate.sleep();
        tick++;
        result << data_arr[0] << "\t" << residual(0) << "\t" <<residual(1)<< "\t" << residual(2)<< "\t" << residual(3)<< "\t" << residual(4)<< "\t" << residual(5) \
        << "\t" << data_arr[49]<< "\t" << data_arr[50]<< "\t" << data_arr[51]<< "\t" << data_arr[52]<< "\t" << data_arr[53]<< "\t" << data_arr[54] \
        << "\t" << tau_m(0)-data_arr[67]<< "\t" << tau_m(1)-data_arr[68]<< "\t" << tau_m(2)-data_arr[69]<< "\t" << tau_m(3)-data_arr[70]<< "\t" << tau_m(4)-data_arr[71]<< "\t" << tau_m(5)-data_arr[72]<< std::endl;
    }

    openFile.close();
    result.close();

    return 0;
}