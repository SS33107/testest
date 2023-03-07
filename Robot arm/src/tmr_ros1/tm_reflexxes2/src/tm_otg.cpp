/*********************************************************************
 * tm_safety.cpp
 *
 * Copyright (c) 2017, ISCI / National Chiao Tung University (NCTU)
 *
 * Author: Howard Chen (s880367@gmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************
 *
 * Author: Howard Chen
 */
/*
 * tm_otg.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: Howard
 */

#include "tm_reflexxes/tm_otg.h"

double last_point_x = 0.0;
double last_point_y = 0.0;
double last_point_z = 0.0;

namespace tm_otg {

    bool CheckJointLimit(double *q)
    {
        bool valid = true;

        if(abs(q[0]) > 270*DEG2RAD)
        {
            ROS_WARN("[Position] 1st joint position out of limit (270) : %lf",q[0]*RAD2DEG );
            valid = false;
        }
        else if(abs(q[1]) > 180*DEG2RAD)
        {
            ROS_WARN("[Position] 2nd joint position out of limit (180): %lf",q[1]*RAD2DEG );
            valid = false;
        }
        else if(abs(q[2]) > 155*DEG2RAD)
        {
            ROS_WARN("[Position] 3rd joint position out of limit (155): %lf",q[2]*RAD2DEG );
            valid = false;
        }
        else if(abs(q[3]) > 180*DEG2RAD)
        {
            ROS_WARN("[Position] 4th joint position out of limit (180): %lf",q[3]*RAD2DEG );
            valid = false;
        }
        else if(abs(q[4]) > 180*DEG2RAD)
        {
            ROS_WARN("[Position] 5th joint position out of limit (180): %lf",q[4]*RAD2DEG );
            valid = false;
        }
        else if(abs(q[5]) > 270*DEG2RAD)
        {
            ROS_WARN("[Position] 6th joint position out of limit (180): %lf",q[5]*RAD2DEG );
            valid = false;
        }
        else
            valid = true;

        return valid;
    }

    bool CheckVelocityLimit(std::vector<double> qd, double &MaxScalingFactor)
    {
        bool valid = true;
        double ExceedRatio[6] = {0,0,0,0,0,0};
        double MaxExceed = 1;
        short  ExceedJoint;

        for (int i = 0; i < 3; ++i)
        {
            if (abs(qd[i])/(150*DEG2RAD) > 1.0)
            {
                ExceedRatio[i] = abs(qd[i])/M_PI;
                ROS_WARN("[Velocity] %dth joint velocity exceed limit(150): %10.4lf",i+1,qd[i]*RAD2DEG);
                valid = false;
            }
            else
                ExceedRatio[i] = 0.0;
        }

        for (int i = 3; i < 6; ++i)
        {
            if (abs(qd[i])/(200*DEG2RAD) > 1.0)
            {
                ExceedRatio[i] = abs(qd[i])/(255*DEG2RAD);
                ROS_WARN("[Velocity] %dth joint velocity exceed limit(200): %10.4lf",i+1,qd[i]*RAD2DEG);
                valid = false;
            }
            else
                ExceedRatio[i] = 0.0;
        }

        if(!valid)
        {
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            {
                if(ExceedRatio[i] > MaxExceed)
                {
                    MaxExceed = ExceedRatio[i];
                    ExceedJoint = i;
                }
            }
            MaxScalingFactor = MaxExceed;
            ROS_WARN("[Velocity] THE MOST EXCEED JOINT IS %dth joint, NEED SCALING %10.4lf",ExceedJoint+1,MaxScalingFactor);
        }

        return valid;
    }

    bool pinv_QR(const Eigen::MatrixXd &A, Eigen::MatrixXd &invA, Scalar eps)
    {
        Eigen::MatrixXd At = A.transpose();
        Eigen::HouseholderQR < Eigen::MatrixXd > qr = At.householderQr();
        int m = A.rows();
        //int n = A.cols();

        Eigen::MatrixXd Rt = Eigen::MatrixXd::Zero(m, m);
        bool invertible;

       Eigen:: MatrixXd hR = (Eigen::MatrixXd) qr.matrixQR();
        Eigen::MatrixXd Y = ((Eigen::MatrixXd) qr.householderQ()).leftCols(m);

        //take the useful part of R
        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j <= i; j++)
                Rt(i, j) = hR(j, i);
        }
        Eigen::FullPivLU < Eigen::MatrixXd > invRt(Rt);

        invertible = fabs(invRt.determinant()) > eps;

        if (invertible)
        {
            invA = Y * invRt.inverse();
            return true;
        }
        else
        {
            return false;
        }

    }

    // Use only for a (mxn) matrix which m<n
    bool pinv(const Eigen::MatrixXd &A, Eigen::MatrixXd &invA, Scalar eps)
    {
        int m = A.rows() - 1;
        Eigen::VectorXd sigma;  //vector of singular values

        Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(A.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
        sigma = svd_A.singularValues();
        if (((m > 0) && (sigma(m) > eps)) || ((m == 0) && (A.array().abs() > eps).any()))
        {
            for (int i = 0; i <= m; i++)
            {
                sigma(i) = 1.0 / sigma(i);
            }
            invA = svd_A.matrixU() * sigma.asDiagonal() * svd_A.matrixV().transpose();
            return true;
        }
        else
        {
            return false;
        }
    }

    bool pinv_SVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &invJ)
    {
        Eigen::VectorXd sigma;  //vector of singular values
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

        sigma = svd.singularValues();
        int m = sigma.rows();

        for (int i = 0; i < m ; ++i)
        {
            if(sigma(i) > EPS)
                sigma(i) = 1.0/ sigma(i);
            else
                sigma(i) = 0.0;
        }

        invJ = svd.matrixV() * sigma.asDiagonal() * svd.matrixU().transpose();

        return true;
    }

    bool GetQfromInverseKinematics( std::vector<double> CartesianPosition, double *q_inv)
    {
        Eigen::Matrix<float,4,4> T_;
        Eigen::AngleAxisf rollAngle (CartesianPosition[5], Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf yawAngle  (CartesianPosition[4], Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf pitchAngle(CartesianPosition[3], Eigen::Vector3f::UnitX());
        Eigen::Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
        Eigen::Matrix<float,3,3> RotationMatrix = q.matrix();
        double *T = new double[16];

        
        T_ <<   0., 0., 0., CartesianPosition[0],
                0., 0., 0., CartesianPosition[1],
                0., 0., 0., CartesianPosition[2],
                0., 0., 0., 1.;

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                T_(i,j) = RotationMatrix(i,j);
            }
        }

        tm_jacobian::Matrix2DoubleArray(T_,T);
        std::cout << ">>>> T " << std::endl;
        tm_jacobian::printMatrix(T,4,16);

        int num_sol =  tm_kinematics::inverse(T, q_inv);

        delete [] T;
        return CheckJointLimit(q_inv);
    }

    bool GetQdfromInverseJacobian(std::vector<double> CurrentPosition,std::vector<double> EFF_Velocity, std::vector<double>& qd)
    {

        Eigen::Matrix<float, 6, 1> home,q;
        home << 0, -PI*0.5, 0, PI*0.5, 0, 0;
        Eigen::Matrix<float,6,1> effspd,jointspd;

        home   << 0, -PI*0.5, 0, PI*0.5, 0, 0;
        effspd << EFF_Velocity[0], EFF_Velocity[1], EFF_Velocity[2], EFF_Velocity[3], EFF_Velocity[4], EFF_Velocity[5];
        q      << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
        q += home;

        Eigen::Matrix<float, 6, 6> Inverse_Jacobian = tm_jacobian::Inverse_Jacobian(q);
        jointspd = Inverse_Jacobian*effspd;
        //std::std::cout << ">>>> Inverse jacobian" << std::endl;
        //tm_jacobian::printMatrix(Inverse_Jacobian);

        tm_jacobian::Matrix2DoubleVector(jointspd,qd);

        double ScalingFactor;

        return CheckVelocityLimit(qd,ScalingFactor);
    }

    bool GetQdfromLinearJacobian(std::vector<double> CurrentPosition,std::vector<double> EFF_Velocity, std::vector<double>& qd)
    {
        Eigen::Matrix<float , 6, 1> home,q;
        Eigen::Matrix<double, 3, 1> EFFSpeed;
        Eigen::Matrix<double, 6, 1> JointSpeed, TaskVecolity;

        home     <<                  0,            -PI*0.5,                  0,             PI*0.5,                  0,                  0;
        q        << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
        EFFSpeed <<    EFF_Velocity[0],    EFF_Velocity[1],    EFF_Velocity[2];
        q += home;

        Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_d(q);
        Eigen::Matrix<double, 3, 6> Jacobian_123456   = Geometry_Jacobian.block<3,6>(0,0);
        
        Eigen::MatrixXd jacobian_123456_inv;
        pinv_SVD(Jacobian_123456,jacobian_123456_inv);
        JointSpeed = jacobian_123456_inv * EFFSpeed;

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            qd[i] = JointSpeed(i);

        double ScalingFactor;

        return CheckVelocityLimit(qd,ScalingFactor);
    }

    bool GetQdfromCartesianConstrain(std::vector<double> CurrentPosition,
                                     std::vector<double> EFF_Velocity, 
                                     std::vector<double>& qd)
    {
        Eigen::Matrix<float , 6, 1> home,q;
        Eigen::Matrix<double , 6, 1> JointSpeed;
        Eigen::Matrix<double, 3, 1> EFFSpeed;

        home     <<                  0,            -PI*0.5,                  0,             PI*0.5,                  0,                  0;
        q        << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
        EFFSpeed <<    EFF_Velocity[0],    EFF_Velocity[1],    EFF_Velocity[2];
        q += home;

        Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_d(q);
        Eigen::Matrix<double, 3, 6> Jacobian_123456   = Geometry_Jacobian.block<3,6>(0,0);



        Eigen::Vector3d TaskVelocity_3;
        std::vector<double> ConstrainedPoint(3);
        double *q_now = new double [6];
        
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            q_now[i] = CurrentPosition[i];

        ConstrainedPoint = {-0.3, 0.1, 0.234};

        if(OnlineCartesianConstrainGeneration_wall(TaskVelocity_3,ConstrainedPoint,q_now)) //constrain generated
        {
            Eigen::Matrix<double, 3, 4> Jacobian_3456     = Jacobian_123456.block<3,4>(0,2);
            Eigen::Matrix<double, 3, 2> Jacobian_12       = Jacobian_123456.block<3,2>(0,0);
            Eigen::Matrix<double, 3, 2> TaskJacobian_3    = tm_jacobian::Forward_Linear_Jacobian_3(q);

            double EFFScalar  = sqrt(EFFSpeed.dot(EFFSpeed));
            double TaskScalar = sqrt(TaskVelocity_3.dot(TaskVelocity_3));
            if(EFFScalar < 0.01)
            {
                if(TaskScalar > EPS)
                    TaskVelocity_3 = TaskVelocity_3*(EFFScalar/TaskScalar);
            }
            //ROS_INFO("EFFScalar  =         %10.4lf",EFFScalar);
            //ROS_INFO("TaskScalar =         %10.4lf",TaskScalar);
            //ROS_INFO("ratio      =         %10.4lf",EFFScalar/TaskScalar);
            ROS_INFO("TaskVelocity_3_disr: %10.4lf %10.4lf %10.4lf",TaskVelocity_3(0),TaskVelocity_3(1),TaskVelocity_3(2));
            
            Eigen::MatrixXd TaskJacobian_3_inv;
            if(!pinv_SVD(TaskJacobian_3, TaskJacobian_3_inv))
                return false;

            Eigen::VectorXd qd_12 = TaskJacobian_3_inv * TaskVelocity_3;
            Eigen::Vector3d TaskVelocity_3_act = TaskJacobian_3*qd_12;
            Eigen::MatrixXd jacobian_123456_inv;
            pinv_SVD(Jacobian_123456,jacobian_123456_inv);
            JointSpeed = jacobian_123456_inv * EFFSpeed;
            qd_12(0) += JointSpeed(0);
            qd_12(1) += JointSpeed(1);
            Eigen::Vector3d TaskVelocity_3_add = TaskJacobian_3*qd_12;
            Eigen::Vector3d TaskVelocity_3_orig = TaskVelocity_3_add-TaskVelocity_3_act;

            ROS_INFO("TaskVelocity_3_act : %10.4lf %10.4lf %10.4lf %10.4lf",TaskVelocity_3_act(0),TaskVelocity_3_act(1),TaskVelocity_3_act(2),sqrt(TaskVelocity_3_act.dot(TaskVelocity_3_act)));
            ROS_INFO("TaskVelocity_3_orig: %10.4lf %10.4lf %10.4lf %10.4lf",TaskVelocity_3_orig(0),TaskVelocity_3_orig(1),TaskVelocity_3_orig(2),sqrt(TaskVelocity_3_orig.dot(TaskVelocity_3_orig)));
            ROS_INFO("TaskVelocity_3_add : %10.4lf %10.4lf %10.4lf %10.4lf",TaskVelocity_3_add(0),TaskVelocity_3_add(1),TaskVelocity_3_add(2),sqrt(TaskVelocity_3_add.dot(TaskVelocity_3_add)));

            if(sqrt(TaskVelocity_3_add.dot(TaskVelocity_3_add)) > 0.14)
            {
                double desire_velocity = 0.14-sqrt(TaskVelocity_3_act.dot(TaskVelocity_3_act));
                double downscaling = desire_velocity/sqrt(TaskVelocity_3_add.dot(TaskVelocity_3_add));
                qd_12(0) =qd_12(0)-JointSpeed(0)+(JointSpeed(0)*downscaling);
                qd_12(1) =qd_12(1)-JointSpeed(1)+(JointSpeed(1)*downscaling);
                TaskVelocity_3_add = TaskJacobian_3*qd_12;
                ROS_WARN("TaskVelocity_3_scal: %10.4lf %10.4lf %10.4lf %10.4lf",TaskVelocity_3_add(0),TaskVelocity_3_add(1),TaskVelocity_3_add(2),sqrt(TaskVelocity_3_add.dot(TaskVelocity_3_add)));
            }

            Eigen::Matrix<double, 3, 1> linear_velocity_temp = EFFSpeed - Jacobian_12*qd_12;
            Eigen::MatrixXd jacobian_3456_inv;
            if(!pinv_SVD(Jacobian_3456,jacobian_3456_inv))
                return false;
            Eigen::VectorXd qd_3456 = jacobian_3456_inv*linear_velocity_temp;

            qd[0] = qd_12(0);
            qd[1] = qd_12(1);
            qd[2] = qd_3456(0);
            qd[3] = qd_3456(1);
            qd[4] = qd_3456(2);
            qd[5] = qd_3456(3);
        
            ROS_WARN("qd_12   :     %10.4lf %10.4lf",qd_12(0),qd_12(1));
            ROS_WARN("qd_3456 :     %10.4lf %10.4lf %10.4lf %10.4lf",qd_3456(0),qd_3456(1),qd_3456(2),qd_3456(3));
        }
        else //no constrained generated
        {
            Eigen::MatrixXd jacobian_123456_inv;
            pinv_SVD(Jacobian_123456,jacobian_123456_inv);
            JointSpeed = jacobian_123456_inv * EFFSpeed;

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                qd[i] = JointSpeed(i);
        }
        
            ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

        double ScalingFactor;
        if(CheckVelocityLimit(qd,ScalingFactor))
            return true;
        else
        {
            return false;
            /*for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            {
                qd[i] = qd[i]/ScalingFactor;
            }
            ROS_WARN("Scaled qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);
            return CheckVelocityLimit(qd,ScalingFactor);*/
        }
    }

    bool OnlineCartesianConstrainGeneration(Eigen::Vector3d &TaskVelocity,
                                        std::vector<double> ConstrainPoint, 
                                        double *q) 
    {
        double Vmax_cc = 0.1;
        double Survelliance_cc = 2/0.45;  //dangerous zone = 0.2m
        double ShapingFactor_cc = 8;
        double *T3 = new double [16];

        tm_jacobian::Forward_Kinematics_3(q,T3,-0.15);

        Eigen::Vector3d Constrain2TaskJoint_vector, TaskJoint_vector, Normal_vector, OrthogonalAvoid_vector;
        Constrain2TaskJoint_vector << ConstrainPoint[0]-T3[3], ConstrainPoint[1]-T3[7], ConstrainPoint[2]-T3[11];
        TaskJoint_vector    << T3[3]-last_point_x, T3[7]-last_point_y, T3[11]-last_point_z;
        
        double dis_constrain = sqrt(pow(Constrain2TaskJoint_vector(0),2) + pow(Constrain2TaskJoint_vector(1),2) + pow(Constrain2TaskJoint_vector(2),2));
        double CartesianInfluence  = Vmax_cc / (1 + exp((dis_constrain*Survelliance_cc-1)*ShapingFactor_cc));
        ROS_INFO("Constrain distance : %10.4lf",dis_constrain);
        ROS_INFO("CartesianInfluence : %10.4lf ",CartesianInfluence);

        if(sqrt(TaskJoint_vector.dot(TaskJoint_vector)) < EPS)
        {
            TaskVelocity << 0.0, 0.0, 0.0;
        }
        else
        {
            Normal_vector = Constrain2TaskJoint_vector.cross(TaskJoint_vector);
            OrthogonalAvoid_vector = Constrain2TaskJoint_vector.cross(Normal_vector);
            double unit =  sqrt(pow(OrthogonalAvoid_vector(0),2)+pow(OrthogonalAvoid_vector(1),2)+pow(OrthogonalAvoid_vector(2),2));
            OrthogonalAvoid_vector = OrthogonalAvoid_vector/unit;
            TaskVelocity = CartesianInfluence*OrthogonalAvoid_vector;

            if(TaskVelocity.dot(TaskJoint_vector) < 0)
                TaskVelocity = -TaskVelocity;
            ROS_INFO("Task vector        : %10.4lf %10.4lf %10.4lf",OrthogonalAvoid_vector(0),OrthogonalAvoid_vector(1),OrthogonalAvoid_vector(2));
        }

        //ROS_INFO("TaskVelocity       : %10.4lf %10.4lf %10.4lf",TaskVelocity(0),TaskVelocity(1),TaskVelocity(2));
        ROS_INFO("joint 3 position   : %10.4lf %10.4lf %10.4lf",T3[3],T3[7],T3[11]);
        last_point_x = T3[3];
        last_point_y = T3[7];
        last_point_z = T3[11];

        delete [] T3;

        if(abs(CartesianInfluence) < 0.08)//Vmax_cc-0.01)
            return 0;
        else
            return 1;
    }

    bool OnlineCartesianConstrainGeneration_wall(Eigen::Vector3d &TaskVelocity,
                                                 std::vector<double> ConstrainPoint, 
                                                 double *q) 
    {
        double Vmax_cc = 0.1;
        double Survelliance_cc = 2/0.31;  //dangerous zone = 0.2m
        double ShapingFactor_cc = 8;
        double *T3 = new double [16];

        tm_jacobian::Forward_Kinematics_3(q,T3,-0.15);
        Eigen::Vector3d Constrain2TaskJoint;
        Constrain2TaskJoint << T3[3]-ConstrainPoint[0], 0, 0;
        
        double dis_constrain = sqrt(Constrain2TaskJoint.dot(Constrain2TaskJoint));
        double CartesianInfluence  = Vmax_cc / (1 + exp((dis_constrain*Survelliance_cc-1)*ShapingFactor_cc));

        if((T3[7] - last_point_y) >= 0)
            TaskVelocity << 0., CartesianInfluence, 0.;
        else
            TaskVelocity << 0., -CartesianInfluence, 0.;


        ROS_INFO("Constrain distance : %10.4lf" ,dis_constrain);
        ROS_INFO("CartesianInfluence : %10.4lf ",CartesianInfluence);
        ROS_INFO("joint 3 position   : %10.4lf %10.4lf %10.4lf",T3[3],T3[7],T3[11]);
        //ROS_INFO("TaskVelocity       : %10.4lf %10.4lf %10.4lf",TaskVelocity(0),TaskVelocity(1),TaskVelocity(2));

        last_point_y = T3[7];
        delete [] T3;

        if(abs(CartesianInfluence) < 0.001)//Vmax_cc-0.01)
            return 0;
        else
            return 1;
    }

    bool OnlineRepulsiveForceGeneration(std::vector<double>& PotentialField_qd,
                                        double *ObstaclePoint,  // obstacle position        : for min_distance
                                        double *TCP_position,   // robot TCP position       : for min_distance
                                        /*double *TCP_velocity,*/   // robot TCP velocity       : for repulsive vector
                                        double *q,              // robot joint position     : for jacobian
                                        double dis_repuslive)   // eff to obstacle distance : for repulsive
    {
        bool succeed = false;
        double Vmax_rep = 1.0;
        double Survelliance_rep = 2/DANGEROUS_ZONE;//0.2;
        double ShapingFactor_rep = 8;

        Eigen::Vector3d Obstacle2Eff;
        Obstacle2Eff << TCP_position[3]-ObstaclePoint[0], TCP_position[7]-ObstaclePoint[1], TCP_position[11]-ObstaclePoint[2];
        Eigen::Vector3d RepulsiveVector   = Obstacle2Eff/dis_repuslive;


        //Eigen::Vector3d relative_speed, relative_speed_unitvector;
        //relative_speed << g_obstacle_velocity[0]-TCP_velocity[0], g_obstacle_velocity[1]-TCP_velocity[1], g_obstacle_velocity[2]-TCP_velocity[2];
        //double obstacle_speed_factor = sqrt( pow(relative_speed(0),2) + pow(relative_speed(1),2) + pow(relative_speed(2),2));
        //relative_speed_unitvector = relative_speed/obstacle_speed_factor;
       // Eigen::Vector3d RepulsiveVector = relative_speed_unitvector;

        double RepulsiveForce  = Vmax_rep / (1 + exp((dis_repuslive*Survelliance_rep-1)*ShapingFactor_rep));
        Eigen::Vector3d RepulsiveVelocity  = RepulsiveForce *RepulsiveVector;

        std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
        std::vector<double> repulsive_qd(6);
        std::vector<double> EFF_Velocity = {RepulsiveVelocity(0), RepulsiveVelocity(1), RepulsiveVelocity(2), 0,0,0}; 
        succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,repulsive_qd);   
        PotentialField_qd = repulsive_qd;
    /*

        std::cout << "================================================"<<std::endl;
        //std::cout << "[ INFO ] Obstacle2Eff : " ;
        //tm_jacobian::printVector(Obstacle2Eff) ;
        printf("[ INFO ] dis_repuslive = [ %lf ] \n",dis_repuslive );
        std::cout << "[ INFO ] RepulsiveVector   : ";
        tm_jacobian::printVector(RepulsiveVector);
        printf("[ INFO ] RepulsiveForce  = [ %lf]  \n",RepulsiveForce );
        std::cout << "[ INFO ] RepulsiveVelocity  :";
        tm_jacobian::printVector(RepulsiveVelocity);
        std::cout << "[ INFO ] PotentialField_qd  :";
        tm_jacobian::printVector(repulsive_qd);
        std::cout << "================================================"<<std::endl;
    */
        return succeed;

    }

    bool OnlineAttractiveForceGeneration(   std::vector<double>& action_qd,
                                        double *TCP_position,             // robot TCP position       : for min_distance
                                        double *q,                        // robot joint position     : for jacobian
                                        std::vector<double> GoalPoint,    // eff to obstacle goal     : for att
                                        std::vector<double> &EFF_Velocity) // eff velocity             : for att
    {
        bool   succeed = false;
        double Vmax_att = 0.1;
        double Survelliance_att = 0.15;
        double ShapingFactor_att = 5;

        Eigen::Vector3d Eff2Goal;
        Eff2Goal << GoalPoint[0]-TCP_position[3], GoalPoint[1]-TCP_position[7], GoalPoint[2]-TCP_position[11];
        double dis_goal = sqrt( pow(Eff2Goal(0),2) + pow(Eff2Goal(1),2) + pow(Eff2Goal(2),2)     );

        Eigen::Vector3d AttractiveVecotor = Eff2Goal/dis_goal;

        double AttractiveForce = Vmax_att - Vmax_att*exp(-(dis_goal*ShapingFactor_att)/Survelliance_att); 

        Eigen::Vector3d AttractiveVelocity = AttractiveForce*AttractiveVecotor; 

        std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
        EFF_Velocity = {AttractiveVelocity(0), AttractiveVelocity(1), AttractiveVelocity(2), 0,0,0}; 
        //ROS_INFO("Attractive Velocity: %10.4lf %10.4lf %10.4lf", AttractiveVelocity(0), AttractiveVelocity(1), AttractiveVelocity(2));
        //succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,action_qd);
        //succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,action_qd);  
        succeed = GetQdfromCartesianConstrain(CurrentPosition,EFF_Velocity,action_qd);     

        return succeed;
    }
}


