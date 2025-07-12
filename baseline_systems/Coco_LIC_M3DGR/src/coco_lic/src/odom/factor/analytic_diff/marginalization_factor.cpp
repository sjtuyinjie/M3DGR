/*
 * Coco-LIC: Coco-LIC: Continuous-Time Tightly-Coupled LiDAR-Inertial-Camera Odometry using Non-Uniform B-spline
 * Copyright (C) 2023 Xiaolei Lang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "marginalization_factor.h"
#include <iomanip>

void ResidualBlockInfo::Evaluate() {
  //
  residuals.resize(cost_function->num_residuals());

  // 
  std::vector<int> block_sizes = cost_function->parameter_block_sizes();

  // 
  jacobians.resize(block_sizes.size());  
  std::vector<double *> raw_jacobians;
  raw_jacobians.reserve(block_sizes.size());
  for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
    jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
    raw_jacobians.push_back(jacobians[i].data());
  }
  cost_function->Evaluate(parameter_blocks.data(), residuals.data(),
                          raw_jacobians.data());

  // 
  if (loss_function) {
    double residual_scaling_, alpha_sq_norm_;

    double sq_norm, rho[3];
    // 
    sq_norm = residuals.squaredNorm();  
    // 
    loss_function->Evaluate(sq_norm, rho);
    // printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm,
    // rho[0], rho[1], rho[2]);

    double sqrt_rho1_ = sqrt(rho[1]);
    if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
      residual_scaling_ = sqrt_rho1_;
      alpha_sq_norm_ = 0.0;
    } else {
      const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
      const double alpha = 1.0 - sqrt(D);
      residual_scaling_ = sqrt_rho1_ / (1 - alpha);
      alpha_sq_norm_ = alpha / sq_norm;
    }

    for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
      jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals *
                                       (residuals.transpose() * jacobians[i]));
    }
    residuals *= residual_scaling_;
  }
}

MarginalizationInfo::~MarginalizationInfo() {
  for (auto it = parameter_block_data.begin(); it != parameter_block_data.end();
       ++it) {
    delete it->second;
  }

  for (int i = 0; i < (int)factors.size(); i++) {
    delete factors[i]->cost_function;
    delete factors[i];
  }
}

void MarginalizationInfo::addResidualBlockInfo(
    ResidualBlockInfo *residual_block_info) {
  factors.emplace_back(residual_block_info);

  // 
  std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;
  std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();
                                    
  // 
  for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++) {
    double *addr = parameter_blocks[i];  
    int size = parameter_block_sizes[i];
    parameter_block_size[reinterpret_cast<long>(addr)] = size;
  }
  // 
  for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++) {
    double *addr = parameter_blocks[residual_block_info->drop_set[i]];  
    parameter_block_idx[reinterpret_cast<long>(addr)] = 0;  
  }
}

void MarginalizationInfo::preMarginalize() {
  for (auto it : factors) {
    // 
    it->Evaluate();

    if (it->residual_type == RType_Image) {
      if (it->residuals.norm() > 1.5) {
        LOG(INFO) << "image: " << it->residuals.transpose();
      }
    }

    /// 
    std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();

    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
      long addr = reinterpret_cast<long>(it->parameter_blocks[i]);
      int size = block_sizes[i];
      if (parameter_block_data.find(addr) == parameter_block_data.end()) {
        // memcpy(double *dest, const double *src, size_t count)
        // sizeof(double) 返回 double类型所占字节个数
        // 
        double *data = new double[size];
        memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
        // 
        parameter_block_data[addr] = data;
      }
    }
  }
}

int MarginalizationInfo::localSize(int size) const {
  return size == 4 ? 3 : size;
}

void *ThreadsConstructA(void *threadsstruct) {
  ThreadsStruct *p = ((ThreadsStruct *)threadsstruct);

  // 
  for (auto it : p->sub_factors) {
    for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++) {
      // 
      int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
      int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
      if (size_i == 4) size_i = 3;
      Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
      for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++) {
        int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
        int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
        if (size_j == 4) size_j = 3;
        Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);

        if (i == j) {
          p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
        } else {
          p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
          p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
        }
      }
      p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals; 
    }
  }
  return threadsstruct;
}

//
bool MarginalizationInfo::marginalize() {
  TicToc timer;
  double time[10];

  int pos = 0;
  for (auto &it : parameter_block_idx) {
    // 
    it.second = pos;
    pos += localSize(parameter_block_size[it.first]);
  }
  // 
  m = pos;  
  //
  for (const auto &it : parameter_block_size) {
    if (parameter_block_idx.find(it.first) == parameter_block_idx.end()) {
      // 
      parameter_block_idx[it.first] = pos;
      pos += localSize(it.second);
    }
  }
  // 
  n = pos - m;

  if (n <= 0) {
    return false;
  }

  TicToc t_summing;
  Eigen::MatrixXd A(pos, pos);
  Eigen::VectorXd b(pos);
  A.setZero();
  b.setZero();

  // 
  TicToc t_thread_summing;
  pthread_t tids[NUM_THREADS];
  ThreadsStruct threadsstruct[NUM_THREADS];
  int i = 0;
  for (auto it : factors) {
    threadsstruct[i].sub_factors.push_back(it);
    i++;
    i = i % NUM_THREADS;
  }
  for (int i = 0; i < NUM_THREADS; i++) {
    TicToc zero_matrix;
    threadsstruct[i].A = Eigen::MatrixXd::Zero(pos, pos);
    threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
    threadsstruct[i].parameter_block_size = parameter_block_size;
    threadsstruct[i].parameter_block_idx = parameter_block_idx;
    int ret = pthread_create(&tids[i], NULL, ThreadsConstructA, (void *)&(threadsstruct[i])); 
    if (ret != 0) {
      LOG(WARNING) << "pthread_create error";
      break;
    }
  }
  //
  for (int i = NUM_THREADS - 1; i >= 0; i--)  
  {
    pthread_join(tids[i], NULL);
    A += threadsstruct[i].A;
    b += threadsstruct[i].b;
  }


  time[0] = timer.toc();

  // 
  Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());  
  // 
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
  Eigen::VectorXd Amm_S = saes.eigenvalues();
  // 
  Eigen::VectorXd Amm_S_inv_trunc = Eigen::VectorXd((Amm_S.array() > eps).select(Amm_S.array().inverse(), 0));
  Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Amm_S_inv_trunc.asDiagonal() * saes.eigenvectors().transpose();

  time[1] = timer.toc();
#if false
  int cnt[2] = {0,0};
  for (int i = 0; i < Amm_S.size(); ++i) {
    if (Amm_S(i) > eps) {
      cnt[0]++;
    } else {
      cnt[1]++;
    }
  }

  std::stringstream ss; 
  if (m > 1000) {
    ss << std::setprecision(2) << Amm_S.head<10>().transpose(); 
  } else {
    // ss << std::setprecision(15) << Amm_S.transpose();
    ss << std::setprecision(2) << Amm_S.transpose();
  }
  LOG(INFO) << "Amm_S cnt: " << cnt[0] << "/" << cnt[1] << "; n/m " << n << "/" << m << "; " << ss.str();
 #endif

  Eigen::VectorXd bmm = b.segment(0, m);
  Eigen::MatrixXd Amr = A.block(0, m, m, n);
  Eigen::MatrixXd Arm = A.block(m, 0, n, m);
  Eigen::MatrixXd Arr = A.block(m, m, n, n);
  Eigen::VectorXd brr = b.segment(m, n);
  A = Arr - Arm * Amm_inv * Amr;
  b = brr - Arm * Amm_inv * bmm;

  time[2] = timer.toc();

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
  Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps)
                          .select(saes2.eigenvalues().array(), 0));
  Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps)
                          .select(saes2.eigenvalues().array().inverse(), 0));
  Eigen::VectorXd S_sqrt = S.cwiseSqrt();
  Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

  time[3] = timer.toc();

  // 
  linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
  linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
  
  time[4] = timer.toc();
  LOG(INFO) << "marginalize costs " 
            << time[0] << "/" 
            << time[1] - time[0] << "/" 
            << time[2] - time[1] << "/" 
            << time[3] - time[2] << "/"
            << time[4] - time[3] << " = " 
            << time[4] << " ms.";

  return true;
}

std::vector<double *> MarginalizationInfo::getParameterBlocks(
    std::unordered_map<long, double *> &addr_shift) {
  std::vector<double *> keep_block_addr;
  keep_block_size.clear();
  keep_block_idx.clear();
  keep_block_data.clear();

  // 
  for (const auto &it : parameter_block_idx) {
    if (it.second >= m) {
      // 
      keep_block_size.push_back(parameter_block_size[it.first]);
      keep_block_idx.push_back(parameter_block_idx[it.first]);
      keep_block_data.push_back(parameter_block_data[it.first]);
      // 
      keep_block_addr.push_back(addr_shift[it.first]);
    }
  }

  // sum_block_size = std::accumulate(std::begin(keep_block_size),
  // std::end(keep_block_size), 0);

  return keep_block_addr;
}


std::vector<double *> MarginalizationInfo::getParameterBlocks() {
  std::vector<double *> keep_block_addr;
  keep_block_size.clear();
  keep_block_idx.clear();
  keep_block_data.clear();

  //
  for (const auto &it : parameter_block_idx) {
    if (it.second >= m) {
      //
      keep_block_size.push_back(parameter_block_size[it.first]);
      keep_block_idx.push_back(parameter_block_idx[it.first]);
      keep_block_data.push_back(parameter_block_data[it.first]);
      //
      keep_block_addr.push_back(reinterpret_cast<double *>(it.first));
    }
  }

  return keep_block_addr;
}


MarginalizationFactor::MarginalizationFactor(
    MarginalizationInfo::Ptr& _marginalization_info)
    : marginalization_info(_marginalization_info.get()) {
  int cnt = 0;
  ///
  for (auto it : marginalization_info->keep_block_size) {
    mutable_parameter_block_sizes()->push_back(it);
    cnt += it;
  }
  // 
  set_num_residuals(marginalization_info->n);  
};

// 
// 
bool MarginalizationFactor::Evaluate(double const *const *parameters,
                                     double *residuals,
                                     double **jacobians) const {
  int n = marginalization_info->n; 
  int m = marginalization_info->m; 

  Eigen::VectorXd dx(n);  // delta_x
  // 
  for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++) {
    int size = marginalization_info->keep_block_size[i];
    // 
    int idx = marginalization_info->keep_block_idx[i] - m;  
    Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);  
     // 
    Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size); 
    if (size != 4)
      dx.segment(idx, size) = x - x0;
    else {
      // dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
      Eigen::Quaterniond q0_inv = Eigen::Quaterniond(x0(3), x0(0), x0(1), x0(2)).inverse();
      Eigen::Quaterniond qx_inv = Eigen::Quaterniond(x(3), x(0), x(1), x(2));
      dx.segment<3>(idx + 0) = 2.0 * positify(q0_inv * qx_inv).vec();
      if (!((q0_inv * qx_inv).w() >= 0)) {
        dx.segment<3>(idx + 0) = 2.0 * -positify(q0_inv * qx_inv).vec();
      }
    }
  }

  // [step1] 
  Eigen::Map<Eigen::VectorXd>(residuals, n) = marginalization_info->linearized_residuals + 
                                              marginalization_info->linearized_jacobians * dx;
  // [step2] 
  if (jacobians) {
    for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++) {
      if (jacobians[i]) {
        int size = marginalization_info->keep_block_size[i];
        int local_size = marginalization_info->localSize(size);
        int idx = marginalization_info->keep_block_idx[i] - m;
        // 
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
                jacobian(jacobians[i], n, size);  
        jacobian.setZero();
        jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);
      }
    }
  }
  return true;
}


