#include <iostream>
#include <raisim/World.hpp>
#include <yaml-cpp/yaml.h>
#include "raisim/RaisimServer.hpp"
#include <fstream>
#include <vector>
#include <iomanip>
#include <memory>

Eigen::Vector4d quaternionFromDirectionVector(Eigen::Vector3d vec){
    std::cout.precision(4);
    double norm = vec.norm();
    if (norm<1e-6){
        return Eigen::Vector4d{1,0,0,0};
    }
    Eigen::Vector3d direction = vec/norm;
    double thetaZ = atan2(direction(1),direction(0))+M_PI/2; //x를 회전축에 맞추기
    double thetaX = acos(direction(2)); //z축 기울이기
    raisim::Vec<3> eulerAngle = raisim::Vec<3>{thetaX,0,thetaZ};
    raisim::Mat<3,3> rot;
    raisim::Vec<4> quat;
    raisim::rpyToRotMat_intrinsic(eulerAngle,rot);
    raisim::rotMatToQuat(rot,quat);
    return quat.e();
}

int main() {
//    std::ofstream outputFile;
//    outputFile.open("/home/savanna-runner/Desktop/grf_estimator/data.txt");

    Eigen::MatrixXd gc_matrix, gv_matrix, ga_matrix, tau_matrix;

    std::ifstream gc_log;
    gc_matrix.setZero(100000,19);
    gc_log.open("/home/savanna-runner/log_data/sim/generalized_position.txt");
    for (size_t i = 0; i < 100000; ++i){
        for (size_t j = 0; j < 19; ++j){
            gc_log >> gc_matrix(i,j);
        }
    }
    gc_log.close();

    std::ifstream gv_log;
    gv_matrix.setZero(100000,18);
    gv_log.open("/home/savanna-runner/log_data/sim/generalized_velocity.txt");
    for (size_t i = 0; i < 100000; ++i){
        for (size_t j = 0; j < 18; ++j){
            gv_log >> gv_matrix(i,j);
        }
    }
    gv_log.close();

    std::ifstream ga_log;
    ga_matrix.setZero(100000,18);
    ga_log.open("/home/savanna-runner/log_data/sim/generalized_acceleration.txt");
    for (size_t i = 0; i < 100000; ++i){
        for (size_t j = 0; j < 18; ++j){
            ga_log >> ga_matrix(i,j);
        }
    }
    ga_log.close();

    std::ifstream tau_log;
    tau_matrix.setZero(100000,18);
    tau_log.open("/home/savanna-runner/log_data/sim/generalized_torque.txt");
    for (size_t i = 0; i < 100000; ++i){
        for (size_t j = 0; j < 18; ++j){
            tau_log >> tau_matrix(i,j);
        }
    }
    tau_log.close();


    raisim::World world_;
    raisim::RaisimServer server(&world_);

    std::string resourceDir_;
    resourceDir_ = "/home/savanna-runner/Desktop/grf_estimator/urdf";
    auto robot_ = world_.addArticulatedSystem(resourceDir_+"/hound2/hound2.urdf");
//    auto robot_ = world_.addArticulatedSystem(resourceDir_+"/hound1/hound1.urdf");
    //auto robot_ = world_.addArticulatedSystem(resourceDir_+"/Hound2.urdf");

    YAML::Node cfg_ = YAML::LoadFile("/home/savanna-runner/Desktop/grf_estimator/cfg.yaml");

    Eigen::VectorXd gc,gv;
    gc.setZero(19);
    gv.setZero(18);
    gc << 0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

    robot_->setState(gc, gv);
//    world_.setTimeStep(0.0005);

    Eigen::VectorXd tau;
    tau.setZero(18);

    server.launchServer();

//    outputFile.is_open();
    std::this_thread::sleep_for(std::chrono::milliseconds (100));

    std::vector<Eigen::Vector3d> contact_force(4, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> contact_force_est(4, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> contact_force_actuated(4, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> contact_point(4, Eigen::Vector3d::Zero());

    Eigen::VectorXd p, p_pre, p_dot;
    Eigen::VectorXd q_dot, q_dot_pre, ga;
    Eigen::VectorXd b;
    Eigen::MatrixXd M, M_pre, M_dot;
    Eigen::VectorXd tau_d, tau_d_est;
    Eigen::VectorXd r, r_pre;
    Eigen::VectorXd sum;

    p.setZero(18);
    p_pre.setZero(18);
    p_dot.setZero(18);

    q_dot.setZero(18);
    q_dot_pre.setZero(18);
    ga.setZero(18);

    b.setZero(18);

    M.setZero(18, 18);
    M_pre.setZero(18, 18);
    M_dot.setZero(18, 18);

    tau_d.setZero(18);
    tau_d_est.setZero(18);
    r.setZero(18);
    r_pre.setZero(18);

    sum.setZero(18);

    //double Ki = 500;

    std::vector<Eigen::MatrixXd> Selector(4,Eigen::MatrixXd::Zero(3,18));
    Selector[0] << Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,9);
    Selector[1] << Eigen::MatrixXd::Zero(3,9), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,6);
    Selector[2] << Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);
    Selector[3] << Eigen::MatrixXd::Zero(3,15), Eigen::MatrixXd::Identity(3,3);

    std::vector<Eigen::MatrixXd> J_foot(4, Eigen::MatrixXd::Zero(3,18));


    std::ofstream grfData_RR("/home/savanna-runner/log_data/grf_RR.csv");
    std::ofstream grfData_RL("/home/savanna-runner/log_data/grf_RL.csv");
    std::ofstream grfData_FR("/home/savanna-runner/log_data/grf_FR.csv");
    std::ofstream grfData_FL("/home/savanna-runner/log_data/grf_FL.csv");


    Eigen::VectorXd q_ddot, q_ddot_lpf;
    q_ddot.setZero(18,1);
    q_ddot_lpf.setZero(18,1);

    for (int i = 0; i<100000; ++i) {
        gc = gc_matrix.row(i);
        gv = gv_matrix.row(i);
        ga = ga_matrix.row(i);
        tau = tau_matrix.row(i);

        robot_->setState(gc, gv);

        q_dot = gv;
        q_ddot = ga;

        b = robot_->getNonlinearities(world_.getGravity()).e();
        M = robot_->getMassMatrix().e();
//        p = M*q_dot;
//        M_dot = (M-M_pre)/dt;

        //r = M*q_ddot+b-tau;

        r = M*q_ddot+b-tau;
        //r = (r_pre+Ki*(p-p_pre+dt*(-M_dot*q_dot+b-tau)))/(1+Ki*dt); // residual(tau_d) with LPF.

        r_pre = r;
        M_pre = M;
        p_pre = p;
        q_dot_pre = q_dot;

        robot_->getDenseFrameJacobian("RR_foot_fixed", J_foot[0]);
        robot_->getDenseFrameJacobian("RL_foot_fixed", J_foot[1]);
        robot_->getDenseFrameJacobian("FR_foot_fixed", J_foot[2]);
        robot_->getDenseFrameJacobian("FL_foot_fixed", J_foot[3]);
// foot center point jacobian. Not the actual contact point. Actual contact point is on the foot sphere.

        std::vector<raisim::Vec<3>> footPos_(4, Eigen::Vector3d::Zero(3,1));
        robot_->getFramePosition("RR_foot_fixed", footPos_[0]);
        robot_->getFramePosition("RL_foot_fixed", footPos_[1]);
        robot_->getFramePosition("FR_foot_fixed", footPos_[2]);
        robot_->getFramePosition("FL_foot_fixed", footPos_[3]);

        std::vector<raisim::Vec<3>> footVel_(4, Eigen::Vector3d::Zero(3,1));
        robot_->getFrameVelocity("RR_foot_fixed", footVel_[0]);
        robot_->getFrameVelocity("RL_foot_fixed", footVel_[1]);
        robot_->getFrameVelocity("FR_foot_fixed", footVel_[2]);
        robot_->getFrameVelocity("FL_foot_fixed", footVel_[3]);

        for (int i = 0; i<4; ++i){
            contact_force_est[i] = (Selector[i]*J_foot[i].transpose()).inverse()*Selector[i]*r;
            contact_force_actuated[i] = -(Selector[i]*J_foot[i].transpose()).inverse()*Selector[i]*tau;
        }

//        std::cout << "contact_force[0]" << std::endl;
//        std::cout << contact_force_est[0] << std::endl;

        int num_leg;

        num_leg = 0;
        grfData_RR << contact_force_est[num_leg][0] << "," << contact_force_est[num_leg][1] << "," << contact_force_est[num_leg][2] << ",";
        grfData_RR << contact_force_actuated[num_leg][0] << "," << contact_force_actuated[num_leg][1] << "," << contact_force_actuated[num_leg][2] <<  ",";
        grfData_RR << footPos_[num_leg][0] << "," << footPos_[num_leg][1] << "," << footPos_[num_leg][2];
        if (i < 100000) {grfData_RR << "\n";}

        num_leg = 1;
        grfData_RL << contact_force_est[num_leg][0] << "," << contact_force_est[num_leg][1] << "," << contact_force_est[num_leg][2] << ",";
        grfData_RL << contact_force_actuated[num_leg][0] << "," << contact_force_actuated[num_leg][1] << "," << contact_force_actuated[num_leg][2] <<  ",";
        grfData_RL << footPos_[num_leg][0] << "," << footPos_[num_leg][1] << "," << footPos_[num_leg][2];
        if (i < 100000) {grfData_RL << "\n";}

        num_leg = 2;
        grfData_FR << contact_force_est[num_leg][0] << "," << contact_force_est[num_leg][1] << "," << contact_force_est[num_leg][2] << ",";
        grfData_FR << contact_force_actuated[num_leg][0] << "," << contact_force_actuated[num_leg][1] << "," << contact_force_actuated[num_leg][2] <<  ",";
        grfData_FR << footPos_[num_leg][0] << "," << footPos_[num_leg][1] << "," << footPos_[num_leg][2];
        if (i < 100000) {grfData_FR << "\n";}

        num_leg = 3;
        grfData_FL << contact_force_est[num_leg][0] << "," << contact_force_est[num_leg][1] << "," << contact_force_est[num_leg][2] << ",";
        grfData_FL << contact_force_actuated[num_leg][0] << "," << contact_force_actuated[num_leg][1] << "," << contact_force_actuated[num_leg][2] <<  ",";
        grfData_FL << footPos_[num_leg][0] << "," << footPos_[num_leg][1] << "," << footPos_[num_leg][2];
        if (i < 100000) {grfData_FL << "\n";}

        std::this_thread::sleep_for(std::chrono::microseconds (500));

    }
    server.killServer();
//    outputFile.close();
    std::cout << "simulation ended" << std::endl;
    return 0;

}