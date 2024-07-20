#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
class MPC {
public:
    Eigen::MatrixXd A, B, C, A_bar, B_bar, C_bar;
    Eigen::VectorXd x_bar;
    Eigen::MatrixXd Q, R, Q_barbar, T_barbar, R_barbar, C_barbar, A_hathat, H_barbar, F_barbar,Minus_H_barbar_inverse,F_barbar_tranposed,F_H, F_H_2;
    int N;
    bool augmented= true;

    MPC();
        
    Eigen::VectorXd compute(const Eigen::VectorXd& x_bar, const Eigen::VectorXd& r); 

    private:
    Eigen::MatrixXd matrixPower(const Eigen::MatrixXd& matrix, int power) {
        
        Eigen::MatrixXd result = Eigen::MatrixXd::Identity(matrix.rows(), matrix.cols());
        for (int i = 0; i < power; ++i) {
            result *= matrix;
        }
        return result;
    }

};

