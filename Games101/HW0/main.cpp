#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

using namespace Eigen;
int main(){

    Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i.row(0) << std::endl;
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v


    /* 
    * PA 0
    */
    // TO DO: Define point P
    Vector3f P(2,1,1);
    // TO DO: Define rotation matrix M
    Matrix3f r;
    float radian = 45.0f / 180.0f * 3.1415f;
    r << cos(radian), -sin(radian), 0, sin(radian), cos(radian), 0, 0, 0, 1;
    // TO DO: M * P
    P = r * P;
    P += Vector3f(1, 2, 0);
    std::cout <<"result:"<< P << std::endl;
    return 0;
}