#include <iostream>
#include "inverse_kinematics.h"

int main() {
    std::cout << "Hello, World!" << std::endl;

    //test_print_matrix();
    VectorXd cartesian_coordinate(6);
    //Input angles
    VectorXd theta(7);
    theta << M_PI_2,M_PI_4,0,M_PI_4,M_PI_2,M_PI_4,M_PI_2;
    MatrixXd T ;
    computeTransformationMatrix(theta);

    //test_print_matrix();
    return 0;
}
