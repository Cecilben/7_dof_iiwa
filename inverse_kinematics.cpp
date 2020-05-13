//
// Created by Metallica on 20/04/2020.
//

#include <iostream>
#include "inverse_kinematics.h"


constexpr std::array<double,7> al = {-M_PI_2,M_PI_2,M_PI_2,-M_PI_2,-M_PI_2,M_PI_2,0 };

// Euclidean distance between joints
constexpr double dbs = 0.34;
constexpr double dse = 0.40;
constexpr double dew = 0.40;
constexpr double dwf = 0.126;


MatrixXd T0_1(4,4);
MatrixXd T1_2(4,4);
MatrixXd T2_3(4,4);
MatrixXd T3_4(4,4);
MatrixXd T4_5(4,4);
MatrixXd T5_6(4,4);
MatrixXd T6_7(4,4);

MatrixXd T0_7(4,4);

MatrixXd Tv0_1(4,4);
MatrixXd Tv1_2(4,4);
MatrixXd Tv2_3(4,4);
MatrixXd Tv3_4(4,4);

MatrixXd Tv0_4(4,4);

VectorXd cartesian_coordinate(6);

double psi = 1;
double GC;

void test_print_matrix()
{
    Eigen::Vector3d p0_2, p2_4, p4_6, p6_7;
    Eigen::RowVector3d r0_2;
    p0_2 << 0, 0, dbs;
    r0_2 << 0, 0, dbs;
    p2_4 << 0, dse, 0;
    p4_6 << 0, 0, dew;
    p6_7 << 0, 0, dwf;

    std::cout << p0_2 << '\n' << r0_2 << '\n';
}
MatrixXd rot_matrix(3,3);

MatrixXd createTmatrix(double theta, double ai, double di) {

    MatrixXd test(4,4);
    std::cout << theta << '\t' << ai << '\t' << di << '\n';
    test << cos(theta), -(sin(theta)*cos(ai)), sin(theta)*sin(ai), 0,
            sin(theta), cos(theta)*cos(ai), -(cos(theta)*sin(ai)), 0,
            0, sin(ai), cos(ai), di,
            0, 0, 0, 1;
    std::cout << test << '\n';
    return (test);
}

MatrixXd computeTransformationMatrix(VectorXd theta)
{
    std::cout << theta << '\n';
    T0_1 << createTmatrix(theta[0],al[0],dbs);
    T1_2 << createTmatrix(theta[1],al[1],0);
    T2_3 << createTmatrix(theta[2],al[2],dse);
    T3_4 << createTmatrix(theta[3],al[3],0);
    T4_5 << createTmatrix(theta[4],al[4],dew);
    T5_6 << createTmatrix(theta[5],al[5],0);
    T6_7 << createTmatrix(theta[6],al[6],dwf);

    T0_7 << T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_7;
    std::cout << "Transformation :" << '\n' << T0_7 << '\n';

    MatrixXd R0_7;
    R0_7 = T0_7.block(0,0,3,3);

    std::cout << "Rotation Matrix" << '\n'<< R0_7 << '\n';

    VectorXd position(3);
    position(0) = T0_7(0,3);
    position(1) = T0_7(1,3);
    position(2) = T0_7(2,3);

    VectorXd orientation(3);
    orientation(0) = atan2(T0_7(2,1), T0_7(2,2)) * 180 / M_PI;
    orientation(1) = -asin(T0_7(2,0)) * 180 / M_PI;
    orientation(2) = atan2(T0_7(1,0), T0_7(0,0)) * 180 / M_PI;

    std::cout << "Position : " << '\n' << position << '\n' << "Orientation : " << '\n' << orientation << '\n';


    double a{orientation(0)*M_PI/180};
    double b{orientation(1)*M_PI/180};
    double c{orientation(2)*M_PI/180};
    rot_matrix <<  cos(b)*cos(c), sin(a)*sin(b)*cos(c) - cos(a)*sin(c), cos(a)*sin(b)*cos(c) + sin(a)*sin(c),
            cos(b)*sin(c), sin(a)*sin(b)*sin(c) + cos(a)*cos(c), cos(a)*sin(b)*sin(c) - sin(a)*cos(c),
            -sin(b), sin(a)*cos(b), cos(a)*cos(b);
    // r0077 << euler2HomogeneousMatrix(orientation(0),orientation(1),orientation(2));

    std::cout << "Rotation_matrix from euler \n" << rot_matrix << '\n';

    Eigen::Vector3d p0_2, p2_4, p4_6, p6_7, p2_6, p0_7;
    Eigen::RowVector3d r0_2;
    p0_2 << 0, 0, dbs;
    r0_2 << 0, 0, dbs;
    p2_4 << 0, dse, 0;
    p4_6 << 0, 0, dew;
    p6_7 << 0, 0, dwf;
    p0_7 = T0_7.block(0,3,3,1);

    std::cout << "Position : P0_7 \n" << p0_7 << '\n' << "P0_2  \n" << p0_2 << '\n' << "rot_matrix*p6_7 \n" <<rot_matrix*p6_7 << '\n';

    p2_6 = p0_7-p0_2-(rot_matrix*p6_7);

    std::cout << " p2_6 : \n"<< p2_6 << '\n' << p2_6.norm() << '\n';

    // declaration for virtual joint angles
    double th_v1,th_v2,th_v3,th_v4;
    //computing theta 4 of virtual robot arm in radians
    th_v4 = 1* acos(pow(p2_6.norm(),2)-pow(dse,2)-pow(dew,2)/(2*dse*dew));

    std::cout << "θv4 values in degree "<< th_v4 * 180/M_PI << '\n';
    std::cout << "θv4 values in radian "<< th_v4 << '\n';
    std::cout << "T0_1" << T0_1 <<'\n';
    Eigen::Vector3d R0_1_z = T0_1.block(0,2,3,1);
    std::cout << "R0_1_z" << R0_1_z << '\n';

    // Computing Theta 1 of virtual robot arm
    if (((p2_6.cross(R0_1_z)).norm()) > 0)
        th_v1 = atan2(p2_6(1), p2_6(0));
    else if (((p2_6.cross(R0_1_z)).norm()) == 0)
        th_v1 = 0;
    std::cout << "θv1 values in degree "<< th_v1 * 180/M_PI << '\n';
    std::cout << "θv1 values in radian "<< th_v1 << '\n';

    // Computing Theta 2 of virtual robot
    double phi = acos(((pow(dse,2)+pow(p2_6.norm(),2)-pow(dew,2))/(dse *2*p2_6.norm())));
    std::cout << "φ values in degree "<< phi * 180/M_PI << '\n';
    std::cout << "φ values in radian "<< phi << '\n' << pow(p2_6(0),2) << '\n' << sqrt(pow(p2_6(0),2)+pow(p2_6(1),2)) << '\n';

    th_v2 = atan2(sqrt(pow(p2_6(0),2)+pow(p2_6(1),2)),p2_6(2)) + 1*phi;
    std::cout << "θv2 values in degree "<< th_v2 * 180/M_PI << '\n';
    std::cout << "θv2 values in radian "<< th_v2  << '\n';

    // Compute the
    MatrixXd Tv0_1(4,4);
    Tv0_1 << createTmatrix(th_v1,al[0],dbs);
    std::cout << "Tv0_1 \n" << Tv0_1 << '\n';
    Tv1_2 << createTmatrix(th_v2,al[1],0);
    std::cout << "Tv1_2 \n" << Tv1_2 << '\n';
    Tv2_3 << createTmatrix(th_v3,al[2],dse);
    std::cout << "Tv2_3 \n" << Tv2_3 << '\n';
    Tv3_4 << createTmatrix(th_v4,al[3],0);
    std::cout << "Tv3_4 \n" << Tv3_4 << '\n';

    Tv0_4 = Tv0_1*Tv1_2*Tv2_3*Tv3_4;
    std::cout << Tv0_4 << '\n';

    th_v3=0;
    std::cout << "θv3 values in degree "<< th_v3 * 180/M_PI << '\n';

    /// Declaration for inverse kinematics
    MatrixXd R0_psi(3,3);
    MatrixXd R0_3(3,3);
    MatrixXd Rv0_3(3,3);
    MatrixXd I3(3,3);
    I3.setIdentity();
    std::cout << I3 << '\n';
    MatrixXd p2_6_unit_skew(3,3);
    p2_6_unit_skew << 0, -p2_6.normalized()(2), p2_6.normalized()(1),
                    p2_6.normalized()(2), 0, -p2_6.normalized()(0),
                    -p2_6.normalized()(1), p2_6.normalized()(0), 0;

    std::cout << "p2_6_unit_skew \n" << p2_6_unit_skew << '\n';
    R0_psi = I3.setIdentity() + sin(psi)*p2_6_unit_skew + (1-cos(psi))*(p2_6_unit_skew*p2_6_unit_skew) ;
    std::cout << "R0_psi" <<R0_psi<< '\n';


    MatrixXd Tv0_3 = Tv0_1*Tv1_2*Tv2_3;

    Rv0_3 = Tv0_3.block(0,0,3,3);
    std::cout << "Rv0_3" <<  Rv0_3 <<  '\n';
    MatrixXd As, Bs, Cs;
    As = p2_6_unit_skew * Rv0_3;
    Bs = (-(p2_6_unit_skew * p2_6_unit_skew))*Rv0_3;
    Cs = p2_6_unit_skew*p2_6_unit_skew.transpose()*Rv0_3;
    std::cout << "As "<< As << '\n' << "Bs" << Bs << '\n' << "Cs" <<  Cs << '\n';

    R0_3 = (p2_6_unit_skew * Rv0_3)*sin(psi) + (-(p2_6_unit_skew * p2_6_unit_skew)*Rv0_3)*cos(psi) + p2_6_unit_skew*p2_6_unit_skew.transpose()*Rv0_3;
    std::cout << "R0_3 : " << R0_3 << '\n';
    double th_1,th_2, th_3, th_4;
    double GC2 = 1;
    th_1 = atan2(GC2*(As(1, 1)*sin(psi) + Bs(1, 1) * cos(psi) + Cs(1,1)), GC2*(As(0,1)*sin(psi) + Bs(0,1)*cos(psi) + Cs(0,1)));
    th_2 = GC2*acos((As(2,1)*sin(psi)) + Bs(2,1)*cos(psi) + Cs(2,1));
    th_3 = atan2(GC2*(-As(2,2)*sin(psi)- Bs(2,2)*cos(psi) - Cs(2,2)),GC2*(-As(2,0)*sin(psi)-Bs(2,0)*cos(psi)-Cs(2,0)));
    th_4 = th_v4;

    MatrixXd Aw, Bw, Cw, R3_4;
    R3_4 = Tv3_4.block(0,0,3,3);
    std::cout << "R3_4 \n" << R3_4 << '\n';

    Aw = R3_4.transpose() * As.transpose() * rot_matrix;
    Bw = R3_4.transpose() * Bs.transpose() * rot_matrix;
    Cw = R3_4.transpose() * Cs.transpose() * rot_matrix;

    std::cout << Aw << '\n' << Bw << '\n' << Cw << '\n';


    double th_5, th_6, th_7;
    double GC6 = 1;
    th_5 = atan2(GC6*(Aw(1,2)*sin(psi) + Bw(1,2) *cos(psi) + Cw(1,2)) , GC6*(Aw(0,2)*sin(psi) + Bw(0,2)*cos(psi) + Cw(0,2)));
    th_6 = GC6*acos( Aw(2,2)*sin(psi) + Bw(2,2)*cos(psi) + Cw(2,2));
    th_7 = atan2(GC6*(Aw(2,1)*sin(psi) + Bw(2,1) *cos(psi) + Cw(2,1)) , GC6*(-Aw(2,0)*sin(psi) - Bw(2,0)*cos(psi) - Cw(2,0)));

    std::cout << th_1* 180/M_PI << '\t' << th_2*180/M_PI << '\t' << th_3*180/M_PI << '\t' << th_4*180/M_PI << '\t' << th_6*180/M_PI << '\t' <<th_7*180/M_PI << '\n';


    /*
     * for (int i = 1; i <= 4; ++i)
      {
          std::cout << "Block of size " << i << "x" << i << std::endl;
          std::cout << T0_7.block(0,0,1,1) << std::endl << std::endl;
      }
      */
    std::cout << T0_7 << '\n';

    return T0_7;
}







