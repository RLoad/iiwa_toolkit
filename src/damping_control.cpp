//|
//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Farshad Khadivr (maintainer)
//|    email:   farshad.khadivar@epfl.ch
//|    website: lasa.epfl.ch
//|
//|    This file is part of iiwa_toolkit.
//|
//|    iiwa_toolkit is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    iiwa_toolkit is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|

#include "damping_control.h"
#include <fstream> // Include for file handling
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

DampingDS::DampingDS(const double& lam0, const double& lam1):eigVal0(lam0),eigVal1(lam1){
    set_damping_eigval(lam0,lam1);
}

DampingDS::~DampingDS(){}
void DampingDS::set_damping_eigval(const double& lam0, const double& lam1){
    if((lam0 > 0)&&(lam1 > 0)){
        eigVal0 = lam0;
        eigVal1 = lam1;
        damping_eigval(0,0) = eigVal0;
        damping_eigval(1,1) = eigVal1;
        damping_eigval(2,2) = eigVal1;
    }else{
        std::cerr << "wrong values for the eigenvalues"<<"\n";
    }
}
void DampingDS::updateDampingMatrix(const Eigen::Vector3d& ref_vel,const Eigen::Vector3d& ref_dvel,const Eigen::Vector3d& ref_dzvel){ 
    //-----first damping controller:eigbector use commend vel
        if(ref_vel.norm() > 1e-6){
            baseMat.setRandom();
            //----passive controller
                // baseMat.col(0) = ref_vel.normalized();
                // for(uint i=1;i<3;i++){
                //     for(uint j=0;j<i;j++)
                //         baseMat.col(i) -= baseMat.col(j).dot(baseMat.col(i))*baseMat.col(j);
                //     baseMat.col(i).normalize();
                // }
                // Dmat = baseMat*damping_eigval*baseMat.transpose();
            //----DAMPING controller
                if(ref_dzvel.norm() > 1e-6)
                {
                    Eigen::Matrix3d realMat; realMat.setRandom();
                    realMat.col(0) = ref_dvel.normalized();
                    realMat.col(1) = ref_dzvel.normalized();
                    realMat.col(2) = realMat.col(0).cross(realMat.col(1));
                    realMat.col(2).normalize();
                    Dmat = realMat*damping_eigval*realMat.transpose();
                }else
                {
                    Dmat = Eigen::Matrix3d::Identity();
                    ROS_WARN_STREAM_THROTTLE(1, "ref_dzvel too small");
                }
                

        }else{
            Dmat = Eigen::Matrix3d::Identity();
            ROS_WARN_STREAM_THROTTLE(1, "ref_vel too small");
        }
        // otherwise just use the last computed basis
    //-----second damping controller:eigbector use random slect
        // if(ref_vel.norm() > 1e-6){
        //     baseMat.setRandom();
        //     //----passive controller
        //         baseMat.col(0) = ref_vel.normalized();
        //         for(uint i=1;i<3;i++){
        //             for(uint j=0;j<i;j++)
        //                 baseMat.col(i) -= baseMat.col(j).dot(baseMat.col(i))*baseMat.col(j);
        //             baseMat.col(i).normalize();
        //         }
        //     //----DAMPING controller
        //         Eigen::Vector3d e1,e2,e3;
        //         Eigen::Matrix3d realMat; realMat.setRandom();
        //         realMat.col(0) = ref_dvel.normalized();
        //         realMat.col(1) = ref_dzvel.normalized();
        //         realMat.col(2) = realMat.col(0).cross(realMat.col(1));
        //         realMat.col(2).normalize();
                
        //         //-----calculate eigenvalue in controller eigen
        //         double alpha1,alpha2;
        //         alpha1=realMat.col(1).dot(baseMat.col(1));
        //         alpha2=realMat.col(1).dot(baseMat.col(2));
        //         Eigen::Matrix3d damping_eigval_rotate = Eigen::Matrix3d::Identity();
        //         damping_eigval_rotate(0,0)=damping_eigval(0,0);
        //         damping_eigval_rotate(1,1) = abs(alpha1)*damping_eigval(1,1);
        //         damping_eigval_rotate(2,2) = abs(alpha2)*damping_eigval(2,2);

        //     Dmat = baseMat*damping_eigval_rotate*baseMat.transpose();

        //     std::cerr<<"ref_vel : "<<ref_vel(0)<<","<<ref_vel(1)<<","<<ref_vel(2)<<"\n"
        //         <<"ref_dvel: "<<ref_dvel(0)<<","<<ref_dvel(1)<<","<<ref_dvel(2)<<"\n"
        //         <<"ref_dzvel : "<<ref_dzvel(0)<<","<<ref_dzvel(1)<<","<<ref_dzvel(2)<<"\n"
        //         <<"baseMat.col(0) : "<<baseMat.col(0)<<"\n"
        //         <<"baseMat.col(1) : "<<baseMat.col(1)<<"\n"
        //         <<"baseMat.col(2) : "<<baseMat.col(2)<<"\n"
        //         <<"realMat.col(0) : "<<realMat.col(0)<<"\n"
        //         <<"realMat.col(1) : "<<realMat.col(1)<<"\n"
        //         <<"realMat.col(2) : "<<realMat.col(2)<<"\n"
        //         <<"alpha: "<<alpha1<<","<<alpha2<<","<<realMat.col(1).dot(baseMat.col(1))<<"\n"
        //         <<"damping of vel: "<<damping_eigval(0,0)<<","<<damping_eigval(1,1)<<","<<damping_eigval(2,2)<<"\n"
        //         <<"damping of controller: "<<damping_eigval_rotate(0,0)<<","<<damping_eigval_rotate(1,1)<<","<<damping_eigval_rotate(2,2)<<"\n";
        // }else{
        //     Dmat = Eigen::Matrix3d::Identity();
        // }
        // // otherwise just use the last computed basis
}

void DampingDS::update(const Eigen::Vector3d& vel, const Eigen::Vector3d& des_vel,const Eigen::Vector3d& vel_Dmax, const Eigen::Vector3d& zvel_Dmax){
    // compute damping
    updateDampingMatrix(des_vel,vel_Dmax,zvel_Dmax);
    
    //----passive controller
    // // dissipate
    // control_output = - Dmat * vel;
    // // compute control
    // control_output += eigVal0*des_vel;

    //------ damping conterol
    control_output = Dmat * (des_vel - vel);

}
Eigen::Vector3d DampingDS::get_output(){ return control_output;}

//************************************************

DampingControl::DampingControl(const std::string& urdf_string,const std::string& end_effector)
{
    _tools.init_rbdyn(urdf_string, end_effector);

    dsGain_pos = 5.00;
    dsGain_ori = 2.50;

    dsContPos = std::make_unique<DampingDS>( 100., 100.);
    dsContOri = std::make_unique<DampingDS>(5., 5.);
    
  
    _robot.name +=std::to_string(0);
    _robot.jnt_position.setZero();
    _robot.jnt_velocity.setZero();
    _robot.jnt_torque.setZero();
    _robot.nulljnt_position.setZero();
    _robot.ee_pos.setZero(); 
    _robot.ee_vel.setZero();   
    _robot.ee_acc.setZero();

    
    double angle0 = 0.25*M_PI;
    _robot.ee_quat[0] = (std::cos(angle0/2));
    _robot.ee_quat.segment(1,3) = (std::sin(angle0/2))* Eigen::Vector3d::UnitZ();
    
    _robot.ee_angVel.setZero();
    _robot.ee_angAcc.setZero();
   
   //* desired things
    _robot.ee_des_pos = {-0.5 , 0.5, 0.2}; 
    double angled = 1.0*M_PI;
    _robot.ee_des_quat[0] = (std::cos(angled/2));
    _robot.ee_des_quat.segment(1,3) = (std::sin(angled/2))* Eigen::Vector3d::UnitX();
    
    //** do we need these parts in here?
    _robot.ee_des_vel.setZero();   
    _robot.ee_des_acc.setZero();
    _robot.ee_des_angVel.setZero();
    _robot.ee_des_angAcc.setZero();
    _robot.ee_des_vel_for_DMatrix.setZero();
    _robot.ee_des_z_vel_for_DMatrix.setZero();
    _robot.ee_des_vel_for_DMatrix_ANGLE.setZero();
    _robot.ee_des_z_vel_for_DMatrix_ANGLE.setZero();

    _robot.null_space_optimal.setZero();


    _robot.jacob.setZero();
    _robot.jacob.setZero();       
    _robot.jacob_drv.setZero();   
    _robot.jacob_t_pinv.setZero();
    _robot.jacobPos.setZero();   
    _robot.jacobAng.setZero();
    _robot.pseudo_inv_jacob.setZero();   
    _robot.pseudo_inv_jacobPos.setZero();

    _robot.Measure.setZero();

    _robot.nulljnt_position << 0.0, 0.0, 0.0, -.75, 0., 0.0, 0.0;
    // _robot.nulljnt_position << 0.6590053837294496, 1.4907334858074615, -1.8296910450078991, -1.3719579419959391, -0.3195112954702344, -0.7669408312305244, 1.9642857845397614;


}

DampingControl::~DampingControl(){}



void DampingControl::updateRobot(const Eigen::VectorXd& jnt_p,const Eigen::VectorXd& jnt_v,const Eigen::VectorXd& jnt_t){
    

    _robot.jnt_position = jnt_p;
    _robot.jnt_velocity = jnt_v;
    _robot.jnt_torque   = jnt_t;

    iiwa_tools::RobotState robot_state;
    robot_state.position.resize(jnt_p.size());
    robot_state.velocity.resize(jnt_p.size());
    for (size_t i = 0; i < jnt_p.size(); i++) {
        robot_state.position[i] = _robot.jnt_position[i];
        robot_state.velocity[i] = _robot.jnt_velocity[i];
    }

    std::tie(_robot.jacob, _robot.jacob_drv) = _tools.jacobians(robot_state);
    _robot.jacobPos =  _robot.jacob.bottomRows(3);
    _robot.jacobAng =  _robot.jacob.topRows(3);

    _robot.pseudo_inv_jacob    = pseudo_inverse(Eigen::MatrixXd(_robot.jacob * _robot.jacob.transpose()) );
    _robot.pseudo_inv_jacobPos = pseudo_inverse(Eigen::MatrixXd(_robot.jacobPos * _robot.jacobPos.transpose()) );
    // _robot.pseudo_inv_jacobPJnt = pseudo_inverse(Eigen::MatrixXd(_robot.jacobPos.transpose() * _robot.jacobPos ) );
    _robot.pseudo_inv_jacobJnt = pseudo_inverse(Eigen::MatrixXd(_robot.jacob.transpose() * _robot.jacob ) );
    
    auto ee_state = _tools.perform_fk(robot_state);
    _robot.ee_pos = ee_state.translation;
    _robot.ee_quat[0] = ee_state.orientation.w();
    _robot.ee_quat.segment(1,3) = ee_state.orientation.vec();
    

    Eigen::VectorXd vel = _robot.jacob * _robot.jnt_velocity;
    _robot.ee_vel    = vel.tail(3); // check whether this is better or filtering position derivitive

    _robot.ee_angVel = vel.head(3); // compare it with your quaternion derivitive equation

    // for(int i = 0; i < 3; i++)
    //     _plotVar.data[i] = (_robot.ee_des_vel - _robot.ee_vel)[i];

    Eigen::MatrixXd JJt = _robot.jacobPos * _robot.jacobPos.transpose();
    Eigen::MatrixXd invJJt = JJt.inverse();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(JJt);


    if (solver.info() != Eigen::Success) {
        std::cerr << "Eigen decomposition failed!" << std::endl;
    }

    // Velocity Ellipsoid
    Eigen::VectorXd eigenValues_v = solver.eigenvalues();
    Eigen::MatrixXd eigenVectors_v = solver.eigenvectors();

    
    // --- Measure manipulability ---
    Eigen::MatrixXd matJacobPos = _robot.jacobPos * _robot.jacobPos.transpose();
    double manipulability = std::sqrt(matJacobPos.determinant());
    _robot.Measure[0] = manipulability; // Store manipulability Measure

    // --- Compute manipulability ellipsoid ---
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_m(_robot.jacobPos, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd eigenValues_m = svd_m.singularValues();
    Eigen::MatrixXd eigenVectors_m_U = svd_m.matrixU();
    Eigen::MatrixXd eigenVectors_m_U_sub = eigenVectors_m_U.block(0, 0, 3, 3);

    for (size_t i = 1; i < 4; i++) {
        _robot.Measure[i] = eigenValues_v[i - 1]; // Store eigenvalues
    }
    Eigen::VectorXd vec_v = Eigen::Map<Eigen::VectorXd>(eigenVectors_v.data(), eigenVectors_v.size());
    for (size_t i = 4; i < 13; i++) {
        _robot.Measure[i] = vec_v[i - 4]; // Store eigenvectors
    }

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver_invJJt(invJJt);
    // Force Ellipsoid
    Eigen::VectorXd eigenValues_f2 = solver_invJJt.eigenvalues();
    Eigen::MatrixXd eigenVectors_f2 = solver_invJJt.eigenvectors();

    // --- Compute force ellipsoid ---
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_f(_robot.jacob.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd eigenValues_f = svd_f.singularValues();
    Eigen::MatrixXd eigenVectors_f_U = svd_f.matrixU();
    Eigen::MatrixXd eigenVectors_f_U_sub = eigenVectors_f_U.block(0, 0, 3, 3);

    for (size_t i = 13; i < 16; i++) {
        _robot.Measure[i] = eigenValues_f2[i - 13]; // Store eigenvalues
    }
    Eigen::VectorXd vec_f2 = Eigen::Map<Eigen::VectorXd>(eigenVectors_f2.data(), eigenVectors_f2.size());
    for (size_t i = 16; i < 25; i++) {
        _robot.Measure[i] = vec_f2[i - 16]; // Store eigenvectors
    }

    // --- Save data to TXT ---
    std::string recPath="/home/ros/ros_ws/src/iiwa_toolkit/Data/";
    // std::cerr << "-------------------------------------------------" << std::endl;
    std::ofstream file(recPath + "robot_data.txt", std::ios::app); // Open in append mode
    if (file.is_open()) {
        // std::cerr << "????????????????????????????????????????????" << std::endl;
        // Save joint positions
        file << "Joint Positions: ";
        for (size_t i = 0; i < _robot.jnt_position.size(); ++i) {
            file << _robot.jnt_position[i] << " ";
        }
        file << "\n";

        // Save end-effector position
        file << "End-Effector Position: ";
        for (size_t i = 0; i < 3; ++i) {
            file << _robot.ee_pos[i] << " ";
        }
        file << "\n";

        // Save end-effector quaternion (orientation)
        file << "End-Effector Quaternion: ";
        for (size_t i = 0; i < 4; ++i) {
            file << _robot.ee_quat[i] << " ";
        }
        file << "\n";

        // Save manipulability Measure
        file << "Manipulability: " << manipulability << "\n";

        // Save manipulability ellipsoid eigenvalues
        file << "Manipulability Ellipsoid Eigenvalues: ";
        for (size_t i = 0; i < 3; ++i) {
            file << eigenValues_v[i] << " ";
        }
        file << "\n";

        // Save manipulability ellipsoid eigenvectors
        file << "Manipulability Ellipsoid Eigenvectors: ";
        for (size_t i = 0; i < eigenVectors_v.size(); ++i) {
            file << eigenVectors_v(i) << " ";
        }
        file << "\n";

        // Save force ellipsoid eigenvalues
        file << "Force Ellipsoid Eigenvalues: ";
        for (size_t i = 0; i < 3; ++i) {
            file << eigenValues_f2[i] << " ";
        }
        file << "\n";

        // Save force ellipsoid eigenvectors
        file << "Force Ellipsoid Eigenvectors: ";
        for (size_t i = 0; i < eigenVectors_f2.size(); ++i) {
            file << eigenVectors_f2(i) << " ";
        }
        file << "\n";

        file << "----------------------------------------\n"; // Separator for readability
        file.close();
    } else {
        std::cerr << "Error: Unable to open file for writing!" << std::endl;
    }


}

Eigen::Vector3d DampingControl::getEEpos(){
    return _robot.ee_pos;
}

Eigen::Vector4d DampingControl::getEEquat(){
    return _robot.ee_quat;
}
Eigen::Vector3d DampingControl::getEEVel(){
    return _robot.ee_vel;
}
Eigen::Vector3d DampingControl::getEEAngVel(){
    return _robot.ee_angVel;
}


void DampingControl::set_pos_gains(const double& ds, const double& lambda0,const double& lambda1){
    dsGain_pos = ds;
    dsContPos->set_damping_eigval(lambda0,lambda1);

}
void DampingControl::set_ori_gains(const double& ds, const double& lambda0,const double& lambda1){
    dsGain_ori = ds;
    dsContOri->set_damping_eigval(lambda0,lambda1);
}
void DampingControl::set_null_pos(const Eigen::VectorXd& nullPosition){
    if (nullPosition.size() == _robot.nulljnt_position.size() )
    {
        _robot.nulljnt_position = nullPosition;
    }else{
        ROS_ERROR("wrong size for the null joint position");
    }
}


void DampingControl::set_desired_pose(const Eigen::Vector3d& pos, const Eigen::Vector4d& quat){
    _robot.ee_des_pos = pos;
    _robot.ee_des_quat = quat;
    is_just_velocity = false;
}
void DampingControl::set_desired_position(const Eigen::Vector3d& pos){
    _robot.ee_des_pos = pos;
     is_just_velocity = false;
}
void DampingControl::set_desired_quat(const Eigen::Vector4d& quat){
    _robot.ee_des_quat = quat;
}
void DampingControl::set_desired_velocity(const Eigen::Vector3d& vel){
     _robot.ee_des_vel = vel;
     is_just_velocity = true;
    // std::cerr<<"vel: "<<vel(0)<<","<<vel(1)<<","<<vel(2)<<"\n";
}
void DampingControl::set_desired_and_z_velocity(const Eigen::Vector3d& desired_vel, const Eigen::Vector3d& desired_vel_z){
     _robot.ee_des_vel_for_DMatrix = desired_vel;
     _robot.ee_des_z_vel_for_DMatrix = desired_vel_z;
    //  std::cerr<<"desired_vel: "<<desired_vel(0)<<","<<desired_vel(1)<<","<<desired_vel(2)<<"\n";
}

void DampingControl::set_null_space(const Eigen::Matrix<double, 7, 1>& desired_null_space){
     _robot.null_space_optimal = desired_null_space;
     std::cerr<<"null_space_optimal: "<<_robot.null_space_optimal(0)<<","<<_robot.null_space_optimal(1)<<","<<_robot.null_space_optimal(2)<<"\n";
}

void DampingControl::set_load(const double& mass ){
    load_added = mass;
}
void DampingControl::computeTorqueCmd(){
    
    // desired position values
    Eigen::Vector3d deltaX = _robot.ee_des_pos - _robot.ee_pos;
    double maxDx = 0.1;
    if (deltaX.norm() > maxDx)
        deltaX = maxDx * deltaX.normalized();
    
    double theta_g = (-.5/(4*maxDx*maxDx)) * deltaX.transpose() * deltaX;
    
    Eigen::Matrix3d zgain = Eigen::Matrix3d::Identity();
    zgain(0,0) *= 1.5; 
    zgain(2,2) *= 1.5; 

    Eigen::Matrix3d xgain = Eigen::Matrix3d::Identity();
    xgain(0,0) *= 1.5; 

    if(!is_just_velocity)
        {
            _robot.ee_des_vel = dsGain_pos*(1+std::exp(theta_g)) *deltaX;
            _robot.ee_des_vel_for_DMatrix = dsGain_pos*(1+std::exp(theta_g)) *deltaX;
            Eigen::MatrixXd Tran_rot_eigen(3,3);
            Tran_rot_eigen(0,0)=1;Tran_rot_eigen(0,1)=0;Tran_rot_eigen(0,2)=0;
            Tran_rot_eigen(1,0)=0;Tran_rot_eigen(1,1)=0;Tran_rot_eigen(1,2)=1;
            Tran_rot_eigen(2,0)=0;Tran_rot_eigen(2,1)=1;Tran_rot_eigen(2,2)=0;
            _robot.ee_des_z_vel_for_DMatrix=Tran_rot_eigen*_robot.ee_des_vel_for_DMatrix;
            ROS_WARN_THROTTLE(0.1, "didn't get desired vel");
        }
    // desired angular values
    Eigen::Vector4d dqd = Utils<double>::slerpQuaternion(_robot.ee_quat, _robot.ee_des_quat, 0.5);    
    Eigen::Vector4d deltaQ = dqd -  _robot.ee_quat;

    Eigen::Vector4d qconj = _robot.ee_quat;
    qconj.segment(1,3) = -1 * qconj.segment(1,3);
    Eigen::Vector4d temp_angVel = Utils<double>::quaternionProduct(deltaQ, qconj);

    Eigen::Vector3d tmp_angular_vel = temp_angVel.segment(1,3);
    double maxDq = 0.2;
    if (tmp_angular_vel.norm() > maxDq)
        tmp_angular_vel = maxDq * tmp_angular_vel.normalized();

    double theta_gq = (-.5/(4*maxDq*maxDq)) * tmp_angular_vel.transpose() * tmp_angular_vel;
    _robot.ee_des_angVel  = 2 * dsGain_ori*(1+std::exp(theta_gq)) * tmp_angular_vel;
    _robot.ee_des_vel_for_DMatrix_ANGLE  = 2 * dsGain_ori*(1+std::exp(theta_gq)) * tmp_angular_vel;
    Eigen::MatrixXd Tran_rot_eigen(3,3);
    Tran_rot_eigen(0,0)=1;Tran_rot_eigen(0,1)=0;Tran_rot_eigen(0,2)=0;
    Tran_rot_eigen(1,0)=0;Tran_rot_eigen(1,1)=0;Tran_rot_eigen(1,2)=1;
    Tran_rot_eigen(2,0)=0;Tran_rot_eigen(2,1)=1;Tran_rot_eigen(2,2)=0;
    _robot.ee_des_z_vel_for_DMatrix_ANGLE  = Tran_rot_eigen*_robot.ee_des_vel_for_DMatrix_ANGLE;


    //----- wr ------ set ref_dzvel small if some code didn't send this comment
        if (std::isnan(_robot.ee_des_z_vel_for_DMatrix.norm())) {
            ROS_WARN_THROTTLE(0.1, "ref_dzvel is generating NaN. Setting the output near to zero.");
            _robot.ee_des_z_vel_for_DMatrix.setZero();
            _robot.ee_des_z_vel_for_DMatrix= {0.0 , 0.0, 0.00001};
        }
        
    // -----------------------get desired force in task space
    dsContPos->update(_robot.ee_vel,_robot.ee_des_vel,_robot.ee_des_vel_for_DMatrix,_robot.ee_des_z_vel_for_DMatrix);
    Eigen::Vector3d wrenchPos = dsContPos->get_output() + load_added * 9.8*Eigen::Vector3d::UnitZ();   
    Eigen::VectorXd tmp_jnt_trq_pos = _robot.jacobPos.transpose() * wrenchPos;

    // std::cerr<<"ee_des_vel: "<<_robot.ee_des_vel(0)<<","<<_robot.ee_des_vel(1)<<","<<_robot.ee_des_vel(2)
    // <<" _robot.ee_des_vel_for_DMatrix: "<<_robot.ee_des_vel_for_DMatrix(0)<<","<<_robot.ee_des_vel_for_DMatrix(1)<<","<<_robot.ee_des_vel_for_DMatrix(2)<<"\n";


    // Orientation
    dsContOri->update(_robot.ee_angVel,_robot.ee_des_angVel,_robot.ee_des_vel_for_DMatrix_ANGLE,_robot.ee_des_z_vel_for_DMatrix_ANGLE);
    Eigen::Vector3d wrenchAng   = dsContOri->get_output();
    Eigen::VectorXd tmp_jnt_trq_ang = _robot.jacobAng.transpose() * wrenchAng;


    //sum up:
    Eigen::VectorXd tmp_jnt_trq = tmp_jnt_trq_pos + tmp_jnt_trq_ang;

    // null pos control
    Eigen::MatrixXd tempMat2 =  Eigen::MatrixXd::Identity(7,7) - _robot.jacob.transpose()* _robot.pseudo_inv_jacob* _robot.jacob;
    Eigen::VectorXd nullgains = Eigen::VectorXd::Zero(7);
    nullgains << 5.,80,10.,30,5.,2.,2.;
    Eigen::VectorXd er_null = _robot.jnt_position -_robot.nulljnt_position;
    if(er_null.norm()<1.5){
        first = false;
    }
    if(er_null.norm()>2e-1){
        er_null = 0.2*er_null.normalized();
    }
    Eigen::VectorXd tmp_null_trq = Eigen::VectorXd::Zero(7);
    for (int i =0; i<7; i++){ 
        tmp_null_trq[i] = -nullgains[i] * er_null[i];
        tmp_null_trq[i] +=-1. * _robot.jnt_velocity[i];
    }
    if (first){
        _trq_cmd = tmp_null_trq;
        ROS_INFO_ONCE("going to the first pose ");                 
    }else{
        ROS_INFO_ONCE("Tracking in process");
        _trq_cmd = tmp_jnt_trq;// + 10.*tempMat2 * tmp_null_trq;
    }
    
    
    
    // Gravity Compensationn
    // the gravity compensation should've been here, but a server form iiwa tools is doing the job.
   

}