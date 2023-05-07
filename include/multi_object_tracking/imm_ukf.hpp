#pragma once

#include <cstdlib>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <fstream>

#include "multi_object_tracking/ukf.hpp"
#include "multi_object_tracking/read_param.hpp"


class IMM_UKF
{
  public:
    IMM_UKF(int id, std::vector<std::string>& model, int state_v, int mea_v, Eigen::MatrixXd& P, std::vector<Eigen::MatrixXd> Q, std::vector<Eigen::MatrixXd> R,Eigen::MatrixXd& interact_probablity)
    {
      track_id_ = id;
      //std::cout<<"build IMM_UKF"<<std::endl;
      try {
        if (model.size()==0) {
          throw "valid input !";
        } else {
          for (size_t i = 0; i < model.size(); ++i) {
            //UKF ukf(model[i], state_v, mea_v, Q[i], R[i],P,i);
            imm_ukf_.emplace_back(model[i], state_v, mea_v, Q[i], R[i],P);
            //std::cout<<"model ukf "<<model[i]<<std::endl;
          }
          n_x_ = state_v;
          n_z_ = mea_v;

          model_size = model.size();

          x_merge_ = Eigen::VectorXd(n_x_);
          p_merge_ = Eigen::MatrixXd(n_x_,n_x_);

          model_pro_ = Eigen::VectorXd(model_size);
          c_ = Eigen::VectorXd(model_size);

          for (int i = 0; i < model_size; ++i) {
            model_pro_(i) = 1.0/(float)model_size;
          }
          interact_pro_ = Eigen::MatrixXd(model_size,model_size);
          interact_pro_ = interact_probablity;

          model_X_.resize(model_size);
          model_P_.resize(model_size);

          for (size_t i = 0; i < model.size(); ++i) {
            model_P_[i] = (P);
          }
          X_hat_.resize(model_size);
          P_hat_.resize(model_size);
          for (size_t i = 0; i < model.size(); ++i) {
            X_hat_[i] = Eigen::VectorXd(n_x_);
            P_hat_[i] = Eigen::MatrixXd(n_x_,n_x_);
            X_hat_[i].fill(0.0);
            P_hat_[i].fill(0.0);
          }
        }
      } catch (const char* msg) {
        std::cerr<<msg<<std::endl;
      }
    };

    ~IMM_UKF()
    {
    };

    void IMM_Initialization(Eigen::VectorXd& Z, float time, float velo, float angles);

    bool IsInitialized() {
      return isinitialized;
    }

    void InputInteract();

    void PredictionZmerge(float time);

    void UpdateProbability(std::vector<Eigen::VectorXd>& Z, const Eigen::VectorXd& beta, const float& last_beta);

    void UpdateProbability(Eigen::VectorXd& Z);

    void MixProbability();

    void Process(std::vector<Eigen::VectorXd>& Z, const Eigen::VectorXd& beta, const float& last_beta, float& time);

    Eigen::VectorXd getMixState();

    Eigen::MatrixXd getMixCovariance();

    Eigen::MatrixXd GetS();

    Eigen::VectorXd GetZpre();

  private:

    size_t model_size = 0;
    int n_x_ = 5;
    int n_z_ = 2;

    int track_id_ = -1;

    bool isinitialized = false;
    std::vector<UKF> imm_ukf_;
    std::vector<Eigen::VectorXd> model_X_;
    std::vector<Eigen::MatrixXd> model_P_;

    std::vector<Eigen::VectorXd> X_hat_;
    std::vector<Eigen::MatrixXd> P_hat_;

    float pi_ = 3.1415926;
    Eigen::MatrixXd interact_pro_;
    Eigen::VectorXd model_pro_;
    Eigen::VectorXd x_merge_;
    Eigen::MatrixXd p_merge_;
    Eigen::VectorXd c_;

    Eigen::MatrixXd S_merge_;
    Eigen::VectorXd Zpre_merge_;
    std::ofstream filewrite;
};
