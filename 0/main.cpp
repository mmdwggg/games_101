#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
using namespace std;

int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,1.0f,1.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;
    std::cout <<"叉乘"<< std::endl;
    std::cout <<  v.cross(w) << std::endl;
    std::cout <<"点乘"<< std::endl;
    std::cout <<  v.dot(w) << std::endl;
    std::cout <<"点乘但不想加为整数"<< std::endl;
    std::cout <<  v.cwiseProduct(w) << std::endl;
    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v




    //My_work
    Eigen::Vector3f p(2.0f,1.0f,1.0f);
    cout<<"old p"<<endl;
    cout<<p<<endl;
    double cs=cos(45.0*acos(-1)/180.0);
    double sn=sin(45.0/180.0*acos(-1));
    double fsn=0-sn;
    Eigen::Matrix3f r;
    r<<cs,fsn,1.0,sn,cs,2.0,0,0,1;
    cout<<"new p"<<endl;
    cout<<p<<endl;

    //test
    std::cout << "Test \n";
    Eigen::Vector3f kd = Eigen::Vector3f(0.7937, 0.7937, 0.7937);
    Eigen::Vector3f position = Eigen::Vector3f(20, 20, 20);
    Eigen::Vector3f point = Eigen::Vector3f(0, 0, 10);
    Eigen::Vector3f intensity = Eigen::Vector3f(500, 500, 500);
    Eigen::Vector3f n = Eigen::Vector3f(1,1,1);
    Eigen::Vector3f l=position-point;//入射方向
    auto rr=l.dot(l);
    auto ld=kd.cross(intensity/rr)*max(0.0f,n.dot(l));
    std::cout << ld << std::endl;
    return 0;
}