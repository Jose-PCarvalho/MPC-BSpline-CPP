#include "MPC.h"

void printMatrix(const Eigen::MatrixXd& matrix) {
    // Using the << operator
    std::cout << matrix << std::endl;

    
}

MPC::MPC(){
    // Initialization of A
    
    if (augmented == true)
    {   
        A = Eigen::MatrixXd::Zero(4, 4);
        B = Eigen::MatrixXd::Zero(4, 2);
        C = Eigen::MatrixXd::Zero(2, 4);

        A.topLeftCorner(2, 2) = Eigen::MatrixXd::Identity(2, 2);
        A.topRightCorner(2, 2) = Eigen::MatrixXd::Identity(2, 2) * 0.03297;
        A.bottomRightCorner(2, 2) = Eigen::MatrixXd::Identity(2, 2) * 0.6703;
        A.bottomLeftCorner(2, 2) = Eigen::MatrixXd::Zero(2, 2); 
        std::cout << "Matrix A" << std::endl;
        printMatrix(A);
        // Initialization of B
        B.topRows(2) = Eigen::MatrixXd::Identity(2, 2) * 0.007032;
        B.bottomRows(2) = Eigen::MatrixXd::Identity(2, 2) * 0.3297;
        std::cout << "Matrix B" << std::endl;
        printMatrix(B);
        // Initialization of C
        C.leftCols(2) = Eigen::MatrixXd::Identity(2, 2);
        C.rightCols(2) = Eigen::MatrixXd::Zero(2, 2);
        std::cout << "Matrix C" << std::endl;
        printMatrix(C);
        x_bar = Eigen::VectorXd::Zero(6);
    }
    else {

        A = Eigen::MatrixXd::Identity(2, 2);
        B = Eigen::MatrixXd::Identity(2, 2) * 1.0/25.0;
        C = Eigen::MatrixXd::Identity(2, 2);
        x_bar = Eigen::VectorXd::Zero(4);
    }

    // Initialization of A_bar
    int n = A.rows();
    int m = B.cols();

    // Construct A_bar
    A_bar = Eigen::MatrixXd::Zero(n + m, n + m);
    A_bar.block(0, 0, n, n) = A;
    A_bar.block(0, n, n, m) = B;
    A_bar.block(n, n, m, m) = Eigen::MatrixXd::Identity(m, m);
    std::cout << "Matrix A_Bar" << std::endl;
    printMatrix(A_bar);


    // Initialization of B_bar
    B_bar = Eigen::MatrixXd::Zero(n + m, m);
    B_bar.block(0, 0, n, m) = B;
    B_bar.block(n, 0, m, m) = Eigen::MatrixXd::Identity(m, m);

    std::cout << "Matrix B_Bar" << std::endl;
    printMatrix(B_bar);
    // Initialization of C_bar
    C_bar = Eigen::MatrixXd::Zero(C.rows(), n + m);
    C_bar.block(0, 0, C.rows(), n) = C;
    std::cout << "Matrix C_Bar" << std::endl;
    printMatrix(C_bar);

    Eigen::MatrixXd Q(2, 2);
    Q << 1, 0,
            0, 1;
    // std::cout << "Matrix Q" << std::endl;
    // printMatrix(Q);
    Eigen::MatrixXd R(2, 2); // 5 works great
    R << 5, 0,
            0, 5;
    // std::cout << "Matrix R" << std::endl;
    // printMatrix(R);

    N = 100;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(N, N);


    // Initialization of Q_barbar
    
    Q_barbar = Eigen::kroneckerProduct(I, C_bar.transpose() * Q * C_bar);
    // std::cout << "Matrix Q_bar" << std::endl;
    // printMatrix(Q_barbar);

    // Initialization of T_barbar
    T_barbar = Eigen::kroneckerProduct(I, Q * C_bar);
    // std::cout << "Matrix T_barbar" << std::endl;
    // printMatrix(T_barbar);

    // Initialization of R_barbar
    R_barbar = Eigen::kroneckerProduct(I, R);
    // std::cout << "Matrix R_barbar" << std::endl;
    // printMatrix(R_barbar);

    // Initialization of C_barbar
    n = A_bar.rows();
    m = B_bar.cols();
    C_barbar = Eigen::MatrixXd::Zero(N * n, N * m);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            if (i - j >= 0) {     
                C_barbar.block(i * n, j * m, n, m) = (matrixPower(A_bar, i - j)) * B_bar; 
            }
        }
    }
    // std::cout << "Matrix C_barbar" << std::endl;
    // printMatrix(C_barbar);

    // Initialization of A_hathat
    A_hathat = Eigen::MatrixXd::Zero(N * n, n);
    for (int i = 0; i < N; ++i) {
        A_hathat.block(i * n, 0, n, n) =  matrixPower(A_bar, i + 1);
    }
    // std::cout << "Matrix A_hathat" << std::endl;
    // printMatrix(A_hathat);

    // Initialization of H_barbar
    H_barbar = C_barbar.transpose() * Q_barbar * C_barbar + R_barbar;
    // std::cout << "Matrix H_barbar" << std::endl;
    // printMatrix(H_barbar);

    // Initialization of F_barbar
    int p1 = A_hathat.transpose().rows();  // Number of rows in A_hathat.transpose()
    int p2 = T_barbar.rows();  // Number of rows in T_barbar
    int q = C_barbar.cols();   // Number of cols in C_barbar

    F_barbar = Eigen::MatrixXd::Zero(p1 + p2, q);
    F_barbar.topRows(p1) = A_hathat.transpose() * Q_barbar * C_barbar;
    F_barbar.bottomRows(p2) = -T_barbar * C_barbar;
    // std::cout << "Matrix F_barbar" << std::endl;
    // printMatrix(F_barbar);

    Minus_H_barbar_inverse = -H_barbar.inverse();
    F_barbar_tranposed = F_barbar.transpose();
    F_H =  Minus_H_barbar_inverse * F_barbar_tranposed ;
    F_H_2 = F_H.block(0, 0, 2, F_H.cols());

        
    }


 Eigen::VectorXd MPC::compute(const Eigen::VectorXd& x_bar, const Eigen::VectorXd& r) {
        this->x_bar = x_bar;

        Eigen::VectorXd xr_combined(x_bar.size() + r.size());
        xr_combined << x_bar, r;

        return F_H_2 * xr_combined;
    }
