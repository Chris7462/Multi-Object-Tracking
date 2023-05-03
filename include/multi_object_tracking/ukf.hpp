#pragma once

#include <iostream>
#include <cstdlib>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unordered_map>


class UKF{
  public:
    UKF(std::string smodel, int state_n, int mea_n, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd P){
      if(model_hash_.count(smodel))
        model_ = model_hash_[smodel];
      n_z_ = mea_n;
      n_x_ = state_n;

      x_ = Eigen::VectorXd(n_x_);
      x_.fill(0.0);

      P_ = Eigen::MatrixXd(n_x_, n_x_);
      P_ = P;
      //噪声
      Q_ = Eigen::MatrixXd(n_x_, n_x_);
      Q_ = Q;

      R_ = Eigen::MatrixXd(n_z_, n_z_);
      R_ = R;

      S_ = Eigen::MatrixXd(n_z_,n_z_);
      S_.fill(0.0);

      Zminus_ = Eigen::VectorXd(n_z_);
      Zminus_.fill(0.0);

    };

    ~UKF(){
    };

    void Initialization(Eigen::VectorXd& X, Eigen::MatrixXd& P, float time);

    bool Isinitalized();

    void MakeSigmaPoints();

    void Prediction(float ntime);

    void PredictionZ(Eigen::VectorXd& X,Eigen::MatrixXd& P, float ntime);

    void Update( std::vector<Eigen::VectorXd>& Z, const Eigen::VectorXd& beta, const float& last_beta);

    void Update( Eigen::VectorXd& Z );

    void Process(Eigen::VectorXd& X, std::vector<Eigen::VectorXd>& Z, const Eigen::VectorXd& beta,const float& last_beta, Eigen::MatrixXd& P, float time);

    Eigen::VectorXd Get_state();

    Eigen::MatrixXd Get_covariance();

    Eigen::VectorXd Get_Zminus();

    Eigen::MatrixXd Get_S();

    Eigen::VectorXd Get_PredictionZ();

  private:
    bool is_initalized_ = false;
    int n_z_;
    int n_aug_;
    int n_x_;

    float lamda_;
    float pretime;

    bool weight_make_ = false;

    std::unordered_map<std::string, int> model_hash_ = {{"CV",1},{"CTRV",2},{"CTRA",3}};
    int model_ = 1;

    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;

    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;

    Eigen::VectorXd pre_weight;
    Eigen::VectorXd mea_weight;

    Eigen::MatrixXd sigma_points;
    Eigen::MatrixXd sigma_points_pre;

    Eigen::MatrixXd Z_sigma_;

    Eigen::MatrixXd S_;
    Eigen::VectorXd Zminus_;
    Eigen::VectorXd z_pre_;
};
