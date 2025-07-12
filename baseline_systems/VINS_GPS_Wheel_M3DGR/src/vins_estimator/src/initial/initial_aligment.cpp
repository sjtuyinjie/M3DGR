#include "initial_alignment.h"

void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs)
{
    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero();
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
        // tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        tmp_A = frame_j->second.pre_integration->jacobian_enc.template block<3, 3>(3, 15);
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;

    }
    delta_bg = A.ldlt().solve(b);
    ROS_INFO("Before WARN.");
    ROS_WARN_STREAM("gyroscope bias initial calibration " << delta_bg.transpose());
    ROS_INFO("Before WARN.");

    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] += delta_bg;
    
    ROS_INFO("Before repropagate.");
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
    }
    ROS_INFO("After repropagate.");
}

// 估计初始尺度
double solveScale(map<double, ImageFrame> &all_image_frame)
{
    MatrixXd A(1, 1);
    VectorXd b(1, 1);
    VectorXd s(1, 1);
    A.setZero();
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 1);
        tmp_A.setZero();
        VectorXd tmp_b(3, 1);
        tmp_b.setZero();
        tmp_A = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T);
        tmp_b = frame_j->second.pre_integration->delta_eta - frame_i->second.pre_integration->delta_q * TIO + TIO
              + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0];
        // ROS_INFO_STREAM("tmp_A = " << tmp_A.transpose());
        // ROS_INFO_STREAM("tmp_b = " << tmp_b.transpose());
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    s = A.ldlt().solve(b);
    ROS_INFO_STREAM("A = " << A);
    ROS_INFO_STREAM("b = " << b);
    ROS_INFO("Visual scale %f", s[0]);
    return s[0];
}

// 施密特正交化分解到球面上
MatrixXd TangentBasis(Vector3d &g0)
{
    Vector3d b, c;
    Vector3d a = g0.normalized();
    Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}

void RefineGravity(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x)
{
    Vector3d g0 = g.normalized() * G.norm();
    Vector3d lx, ly;
    //VectorXd x;
    int all_frame_count = all_image_frame.size();
    // int n_state = all_frame_count * 3 + 2 + 1;
    int n_state = all_frame_count * 3 + 2;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    // 循环4次，是为何意？
    for(int k = 0; k < 4; k++)
    {
        MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0);
        int i = 0;

        Eigen::Quaterniond q_b0bk;
        q_b0bk.setIdentity();
        q_b0bk = q_b0bk * all_image_frame.begin()->second.pre_integration->delta_q;

        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
        {
            frame_j = next(frame_i);

            // MatrixXd tmp_A(6, 9);
            MatrixXd tmp_A(6, 8);
            tmp_A.setZero();
            VectorXd tmp_b(6);
            tmp_b.setZero();

            double dt = frame_j->second.pre_integration->sum_dt;

            // tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
            // tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
            // tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;     
            // tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0] - frame_i->second.R.transpose() * dt * dt / 2 * g0;

            // tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
            // tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
            // tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity() * lxly;
            // tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * Matrix3d::Identity() * g0;

            q_b0bk = q_b0bk * frame_j->second.pre_integration->delta_q;
            tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
            tmp_A.block<3, 2>(0, 6) = 0.5 * frame_i->second.R.transpose() * RIC[0].transpose() * dt * dt * Matrix3d::Identity() * lxly;
            // tmp_A.block<3, 2>(0, 6) = 0.5 * q_b0bk.inverse().toRotationMatrix() * dt * dt * Matrix3d::Identity() * lxly;
            tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p - frame_j->second.pre_integration->delta_eta + 
                                    frame_j->second.pre_integration->delta_q.toRotationMatrix() * TIO - TIO -
                                    0.5 * frame_i->second.R.transpose() * RIC[0].transpose() * dt * dt * g0;
            // tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p - frame_j->second.pre_integration->delta_eta + 
            //                         frame_j->second.pre_integration->delta_q.toRotationMatrix() * TIO - TIO -
            //                         0.5 * q_b0bk.inverse().toRotationMatrix() * dt * dt * g0;
            tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
            tmp_A.block<3, 3>(3, 3) = frame_i->second.pre_integration->delta_q.toRotationMatrix();
            tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * RIC[0].transpose() * dt * Matrix3d::Identity() * lxly;
            // tmp_A.block<3, 2>(3, 6) = q_b0bk.inverse().toRotationMatrix() * dt * Matrix3d::Identity() * lxly;
            tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v -
                                      frame_i->second.R.transpose() * RIC[0].transpose() * dt * Matrix3d::Identity() * g0;
            // tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v -
            //                           q_b0bk.inverse().toRotationMatrix() * dt * Matrix3d::Identity() * g0;


            Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
            //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
            //MatrixXd cov_inv = cov.inverse();
            cov_inv.setIdentity();

            MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
            VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();

            // A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
            // b.tail<3>() += r_b.tail<3>();

            // A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
            // A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();

            A.bottomRightCorner<2, 2>() += r_A.bottomRightCorner<2, 2>();
            b.tail<2>() += r_b.tail<2>();

            A.block<6, 2>(i * 3, n_state - 2) += r_A.topRightCorner<6, 2>();
            A.block<2, 6>(n_state - 3, i * 2) += r_A.bottomLeftCorner<2, 6>();
        }
            A = A * 1000.0;
            b = b * 1000.0;
            x = A.ldlt().solve(b);
            // VectorXd dg = x.segment<2>(n_state - 3);
            VectorXd dg = x.segment<2>(n_state - 2);
            g0 = (g0 + lxly * dg).normalized() * G.norm();
            //double s = x(n_state - 1);
    }   
    g = g0;
}

bool LinearAlignment(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x)
{
    int all_frame_count = all_image_frame.size();
    // int n_state = all_frame_count * 3 + 3 + 1;
    int n_state = all_frame_count * 3 + 3;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    ROS_INFO("Solve velocity and gravity.");
    
    Eigen::Quaterniond q_b0bk;
    q_b0bk.setIdentity();
    q_b0bk = q_b0bk * all_image_frame.begin()->second.pre_integration->delta_q;

    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);

        // MatrixXd tmp_A(6, 10);
        MatrixXd tmp_A(6, 9);
        tmp_A.setZero();
        VectorXd tmp_b(6);
        tmp_b.setZero();

        double dt = frame_j->second.pre_integration->sum_dt;

        // tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        // tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
        // tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0; // 100??    
        // tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0];
        // //cout << "delta_p   " << frame_j->second.pre_integration->delta_p.transpose() << endl;
        // tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
        // tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
        // tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity();
        // tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        // //cout << "delta_v   " << frame_j->second.pre_integration->delta_v.transpose() << endl;

        q_b0bk = q_b0bk * frame_j->second.pre_integration->delta_q;
        tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = 0.5 * frame_i->second.R.transpose() * RIC[0].transpose() * dt * dt * Matrix3d::Identity();
        // tmp_A.block<3, 3>(0, 6) = 0.5 * q_b0bk.inverse().toRotationMatrix() * dt * dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p - frame_j->second.pre_integration->delta_eta + 
                                  frame_j->second.pre_integration->delta_q.toRotationMatrix() * TIO - TIO;
        cout << "delta_eta   " << frame_j->second.pre_integration->delta_eta.transpose() << endl;
        cout << "delta_p   " << frame_j->second.pre_integration->delta_p.transpose() << endl;
        cout << "delta_q   " << frame_j->second.pre_integration->delta_q.toRotationMatrix() << endl;
        tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i->second.pre_integration->delta_q.toRotationMatrix();
        tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * RIC[0].transpose() * dt * Matrix3d::Identity();
        // tmp_A.block<3, 3>(3, 6) = q_b0bk.inverse().toRotationMatrix() * dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        cout << "delta_v   " << frame_j->second.pre_integration->delta_v.transpose() << endl;

        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        //MatrixXd cov_inv = cov.inverse();
        cov_inv.setIdentity();

        MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
        b.segment<6>(i * 3) += r_b.head<6>();

        // A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
        // b.tail<4>() += r_b.tail<4>();

        // A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
        // A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();

        A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
        b.tail<3>() += r_b.tail<3>();

        A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
        A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    VectorXd tmp_x;
    tmp_x = A.ldlt().solve(b);
    // double s = x(n_state - 1) / 100.0;
    // ROS_DEBUG("estimated scale: %f", s);
    // g = x.segment<3>(n_state - 4);
    g = tmp_x.segment<3>(n_state - 3);
    ROS_INFO_STREAM(" result g     " << g.norm() << " " << g.transpose());
    // if(fabs(g.norm() - G.norm()) > 1.0 || s < 0)
    // {
    //     return false;
    // }
    if(fabs(g.norm() - G.norm()) > 1.0)
    {
        return false;
    }

    RefineGravity(all_image_frame, g, tmp_x);

    // 这里补充了尺度信息，保证外部接口调用的一致性
    double s = solveScale(all_image_frame);
    x.resize(tmp_x.rows() + 1, tmp_x.cols());
    x = tmp_x;
    (x.tail<1>())(0) = s;
    // s = (x.tail<1>())(0) / 100.0;
    // (x.tail<1>())(0) = s;
    g = RIC[0].transpose() * g; // 再转换到camera坐标系下
    ROS_INFO_STREAM(" refine     " << g.norm() << " " << g.transpose());
    if(s < 0.0 )
        return false;   
    else
        return true;
}

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x)
{
    solveGyroscopeBias(all_image_frame, Bgs);

    if(LinearAlignment(all_image_frame, g, x))
        return true;
    else 
        return false;
}
