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
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(model, q, H_origin, true);

        q_new = q;
        q_new(i) += h;
        H_new.setZero();
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(model, q_new, H_new, true);

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

    ros::param::set("use_sim_time", false);
    ros::Duration(1.0).sleep();

    ros::Rate loop_rate(1000);

    ros::ServiceClient physics_client = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    ros::ServiceClient joint_position_client = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

    std_srvs::Empty pauseSrv;
    physics_client.call(pauseSrv);

    RigidBodyDynamics::Model model;
    RigidBodyDynamics::Addons::URDFReadFromFile("/home/kim/doosan_ws/src/doosan-robot/dsr_description/urdf/m0609.urdf", &model, false, false);

    std::ifstream openFile("/home/kim/ssd2/dusan_ws/2nd/data/robot1_20190829/free/0_00kg/1/DRCL_Data_1.txt");
    std::string line;
    std::stringstream line_ss;
    float data_arr[100];

    if( !openFile.is_open() ){
        return 0;
    }

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

    Vector6d q;
    Vector6d q_dot;
    Eigen::MatrixXd A;
    Eigen::MatrixXd C;
    Eigen::VectorXd g;
    Vector6d q_dot_zero;
    q_dot_zero.setZero();
    Eigen::VectorXd nonlinear;
    Vector6d b;
    Vector6d b_custom;

    int tick = 0;
    while (ros::ok())
    {
        int count = 0;
        q.setZero();
        q_dot.setZero();
        A.setZero(6,6);
        C.setZero(6,6);
        g.setZero(6);
        nonlinear.setZero(6);

        float data;
        getline(openFile, line);
        line_ss.str(line);
        while(line_ss >> data)
        {
            data_arr[count] = data;
            count++;
        }
        line_ss.clear();

        q << data_arr[7], data_arr[8], data_arr[9], data_arr[10], data_arr[11], data_arr[12];
        q_dot << data_arr[13], data_arr[14], data_arr[15], data_arr[16], data_arr[17], data_arr[18];

        RigidBodyDynamics::CompositeRigidBodyAlgorithm(model, q, A, true);
        C = getC(model, q, q_dot);
        RigidBodyDynamics::NonlinearEffects(model, q, q_dot, nonlinear);
        RigidBodyDynamics::NonlinearEffects(model, q, q_dot_zero, g);
        b = nonlinear - g;
        b_custom = C * q_dot;

        modelconfig.request.joint_positions[0]= q(0);
        modelconfig.request.joint_positions[1]= q(1);
        modelconfig.request.joint_positions[2]= q(2);
        modelconfig.request.joint_positions[3]= q(3);
        modelconfig.request.joint_positions[4]= q(4);
        modelconfig.request.joint_positions[5]= q(5);

        if (!joint_position_client.call(modelconfig)) ROS_ERROR("Failed to call the service");

        if (tick%100 == 0)
            std::cout << "Time: " << data_arr[0] << std::endl;
 
        loop_rate.sleep();
        tick++;
    }
    openFile.close();

    return 0;
}