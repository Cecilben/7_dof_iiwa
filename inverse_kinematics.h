//
// Created by Metallica on 20/04/2020.
//

#ifndef INC_7_DOF_IIWA_INVERSE_KINEMATICS_H
#define INC_7_DOF_IIWA_INVERSE_KINEMATICS_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXf;

void test_print_matrix();

MatrixXd computeTransformationMatrix(VectorXd);
MatrixXd createTmatrix (double theta,double ai, double di);

#endif //INC_7_DOF_IIWA_INVERSE_KINEMATICS_H
