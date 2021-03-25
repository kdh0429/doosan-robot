// ************************************************************************************
//      Usage: rosrun reproduce_dyros reproduce_dyros process_type(int) folder_name(string) tool_weight(string)
//      Example: rosrun reproduce_dyros reproduce_dyros 0 /home/kim/ssd2/dusan_ws/2nd/data_tro/robot1_20190829/free/0_00kg/1 0_00kg  
// ************************************************************************************
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <fstream>
#include <iostream>
#include <iomanip> 
#include <sstream>

#include "std_srvs/Empty.h"
#include "gazebo_msgs/SetModelConfiguration.h"

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include <ctime>

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
    // Uncomment to simulate with Gazebo
    srand((unsigned int)time(0));
    ros::init(argc, argv, "dyros_reproduce"+std::to_string(std::rand()));
    ros::NodeHandle n;

    // Data Selection
    int process_type = std::atoi(argv[1]); // 0: Process MOB, motor inertia, and accelermeter data     1: 1000Hz to 100Hz
    std::string data_folder_name = argv[2];
    std::string tool_weight = argv[3];

    // Load URDF model
    RigidBodyDynamics::Model model;
    std::string urdf_name = "/home/kim/doosan_ws/src/doosan-robot/dsr_description/urdf/m0609_modify_" + tool_weight +".urdf";
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &model, false, false);


    if (process_type == 0) // Process MOB, motor inertia, and accelermeter data
    {
        // Load data file
        std::string data_file_name = data_folder_name + "/" +"DRCL_Data_";
        int file_idx = 1; 
        std::string open_data_file_name = data_file_name + std::to_string(file_idx) + ".txt";
        std::ifstream openFile(open_data_file_name);
        std::string line;
        std::stringstream line_ss;
        double data_arr[100];
        if( !openFile.is_open() ){
            std::cout << "Failed to open the data file: " << open_data_file_name << std::endl;
            return 0;
        }
        
        // Data and dynamic matrix variables
        Vector6d q;
        Vector6d q_d;
        Vector6d q_dot;  
        Vector6d q_dot_d;  
        Vector6d q_dot_d_pre;   
        Vector6d q_ddot; 
        Vector6d tau_m;
        Eigen::VectorXd tau_inv_dyn;
        Eigen::Vector3d accelermeter;

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
            K_mob(i,i)=100;
        }

        // Accelermeter variables
        Eigen::MatrixXd Jac;
        Jac.resize(3,6);
        Jac.setZero();
        Eigen::MatrixXd Jac_pre;
        Jac_pre.resize(3,6);
        Jac_pre.setZero();
        Eigen::MatrixXd j_temp;
        j_temp.resize(6,6);
        j_temp.setZero();
        Eigen::Vector3d zero_point;
        zero_point.setZero();


        int tick = 0;
        double dt = 0.001;
        double last_time = 0.0;
        std::ofstream result; 
        std::string write_data_file_name = data_file_name + "MOB" + std::to_string(file_idx) + ".txt";
        result.open(write_data_file_name);
        result << std::fixed << std::setprecision(4);

        while (ros::ok())
        {
            if (openFile.peek() == EOF)
            {
                result.close();
                openFile.close();
                file_idx++;

                std::string open_data_file_name = data_file_name + std::to_string(file_idx) + ".txt";
                openFile.open(open_data_file_name);
                if( !openFile.is_open() ){
                    std::cout<<"Finished Proccessing" << std::endl;
                    return 0;
                }

                std::string write_data_file_name = data_file_name + "MOB" + std::to_string(file_idx) + ".txt";
                result.open(write_data_file_name);
            }

            int count = 0;
            q.setZero();
            q_dot.setZero();
            q_dot_d.setZero();
            A.setZero(6,6);
            C.setZero(6,6);
            C_T.setZero(6,6);
            g.setZero(6);
            nonlinear.setZero(6);
            tau_inv_dyn.setZero(6);
            q_dot_zero.setZero();

            // Read data
            double data;
            getline(openFile, line);
            line_ss.str(line);
            while(line_ss >> data)
            {
                data_arr[count] = data;
                count++;
            }
            line_ss.clear();

            if (count == 79){
                std::cout <<"It is not a raw data: You have already processed data!" << std::endl;
                return 0;
            }

            // Allocate data
            q << data_arr[7], data_arr[8], data_arr[9], data_arr[10], data_arr[11], data_arr[12];
            q_d << data_arr[19], data_arr[20], data_arr[21], data_arr[22], data_arr[23], data_arr[24];
            q_dot << data_arr[13], data_arr[14], data_arr[15], data_arr[16], data_arr[17], data_arr[18];
            tau_m << data_arr[1], data_arr[2], data_arr[3], data_arr[4], data_arr[5], data_arr[6];
            q_dot_d << data_arr[25], data_arr[26], data_arr[27], data_arr[28], data_arr[29], data_arr[30];
            accelermeter << data_arr[61], data_arr[62], data_arr[63];

            ////////////////////////////////////////////////////// Momentum observer ////////////////////////////////////////////////////////////
            // Current dynamics information
            MotorInertiaReflectedCompositeRigidBodyAlgorithm(model, q, A, true);
            C = getC(model, q, q_dot);
            C_T = C.transpose();
            RigidBodyDynamics::NonlinearEffects(model, q, q_dot_zero, g);

            dt = data_arr[0] - last_time;
            if( tick == 0)
            {
                dt = 0.001;
            }
            if (dt < 0.0)
            {
                integral_term.setZero();
                residual.setZero();
                dt = 0.001;
                std::cout << "dt is smaller than 0!" << std::endl;
            }

            integral_term = integral_term + (tau_m + C_T*q_dot - g - residual)*dt;
            residual = K_mob * (integral_term - A*q_dot);
            last_time = data_arr[0];
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            //////////////////////////////////////////// Add motor inertia to dynamic torque //////////////////////////////////////////////////
            if( tick == 0)
            {
                q_dot_d_pre = q_dot_d;
            }
            if (dt < 0.0)
            {
                q_dot_d_pre = q_dot_d;
                std::cout << "dt is smaller than 0!" << std::endl;
            }
            data_arr[31] += 2.6496  * (q_dot_d(0) - q_dot_d_pre(0))/dt;
            data_arr[32] += 2.6496  * (q_dot_d(1) - q_dot_d_pre(1))/dt;
            data_arr[33] += 1.37    * (q_dot_d(2) - q_dot_d_pre(2))/dt;
            data_arr[34] += 0.6336  * (q_dot_d(3) - q_dot_d_pre(3))/dt;
            data_arr[35] += 0.6336  * (q_dot_d(4) - q_dot_d_pre(4))/dt;
            data_arr[36] += 0.63296 * (q_dot_d(5) - q_dot_d_pre(5))/dt;
            q_dot_d_pre = q_dot_d;
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////// Accelermeter error //////////////////////////////////////////////////////////////
            Eigen::Matrix3d Rotm = (RigidBodyDynamics::CalcBodyWorldOrientation(model, q_d, model.GetBodyId("link6"), true));
            Eigen::Vector3d gravity;
            gravity << 0, 0, -9.80;
            
            j_temp.setZero();
            RigidBodyDynamics::CalcPointJacobian6D(model, q_d, model.GetBodyId("link6"), zero_point, j_temp, true);
            Jac.block<3, 6>(0, 0) = j_temp.block<3, 6>(3, 0);

            if( tick == 0)
            {
                Jac_pre = Jac;
            }
            Eigen::Matrix3d Rotm_acc_ee;
            Rotm_acc_ee << -1, 0, 0,
                            0, -1, 0,
                            0, 0, -1;
            Eigen::Vector3d gravity_local = Rotm_acc_ee*Rotm*gravity;
            Eigen::Vector3d ee_acc_d = (Jac - Jac_pre)/dt * q_dot_d + Jac / dt * (q_dot_d - q_dot_d_pre);
            ee_acc_d = Rotm_acc_ee*Rotm*ee_acc_d;
            Eigen::Vector3d accelermeter_err;
            accelermeter_err = accelermeter - (gravity_local + ee_acc_d);
            data_arr[61] = accelermeter_err[0];
            data_arr[62] = accelermeter_err[1];
            data_arr[63] = accelermeter_err[2];

            Jac_pre = Jac;
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            if (tick%1000 == 0)
            {
                std::cout << "Time: " << data_arr[0] << std::endl;
            }
            // loop_rate.sleep();
            tick++;
            for (int i=0; i<73; i++){
                result << data_arr[i] << "\t";
            }
            result << residual(0) << "\t" <<residual(1)<< "\t" << residual(2)<< "\t" << residual(3)<< "\t" << residual(4)<< "\t" << residual(5) << std::endl;
        }

        openFile.close();
        result.close();
    }

    if (process_type == 1)
    {
        // Load data file
        std::string data_file_name = data_folder_name +"/" +"DRCL_Data_";
        int file_idx = 1; 
        std::string open_data_file_name = data_file_name + "MOB" + std::to_string(file_idx) + ".txt";
        std::ifstream openFile(open_data_file_name);
        std::string line;
        std::stringstream line_ss;
        double data_arr[100];
        if( !openFile.is_open() ){
            std::cout << "Failed to open the data file: " << open_data_file_name << std::endl;
            return 0;
        }
        
        std::ofstream reduced_result;
        std::string write_data_file_name = data_folder_name + "/" + "Reduced_DRCL_Data.txt";
        reduced_result.open(write_data_file_name);
        reduced_result << std::fixed << std::setprecision(4);

        int tick = 0;
        float time_pre = 0.0;
        float dt = 0.0;

        while (ros::ok())
        {
            if (openFile.peek() == EOF)
            {
                openFile.close();
                file_idx++;

                std::string open_data_file_name = data_file_name + "MOB" + std::to_string(file_idx) + ".txt";
                openFile.open(open_data_file_name);
                if( !openFile.is_open() ){
                    reduced_result.close();
                    std::cout<<"Finished Proccessing" << std::endl;
                    return 0;
                }
            }

            // Read data
            double data;
            int count = 0;
            getline(openFile, line);
            line_ss.str(line);
            while(line_ss >> data)
            {
                data_arr[count] = data;
                count++;
            }
            line_ss.clear();

            if (!count == 79)
            {
                std::cout<< "Number of data is not 79. Process MOB value!" << std::endl;
                return 0;
            }

            if (tick % 10 == 0){
                for (int i=0; i<78; i++){
                    reduced_result << data_arr[i] << "\t";
                }
                reduced_result << data_arr[78];
                reduced_result << std::endl;
                dt = data_arr[0] - time_pre;
                time_pre = data_arr[0];
                if ((dt < 0.01 - 0.001) || (0.01+0.001 < dt)){
                    std::cout<< "If you see this many times, dt might be strange" << std::endl;
                    std::cout << "Dt: " << dt << std::endl;
                }
            }

            if (tick%1000 == 0)
            {
                std::cout << "Time: " << data_arr[0] << std::endl;
            }
            tick++;
        }
    }

    return 0;
}