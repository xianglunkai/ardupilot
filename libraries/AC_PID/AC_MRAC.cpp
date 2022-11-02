#include "AC_MRAC.h"

#include <cmath>
#include <string>
#include <vector>

#include <eigen/include/eigen3/Eigen/Dense>
#include <eigen/include/eigen3/Eigen/LU>

#include <AP_Math/matrix_operations.h>

const AP_Param::GroupInfo AC_MRAC::var_info[] = {

    // @Param: WN
    // @Description: Reference model response bandwidth
    // @Units: rad/s
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("WN", 1, AC_MRAC, wn_, 10),

    // @Param: B0
    // @Description: Control input gain
    // @Range: 0.1 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("B0", 3, AC_MRAC, b0_, 1.0),

    // @Param: GAMAX
    // @Description: Adaptive state learning coef
    // @Range: 0.1 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("COEx", 4, AC_MRAC, adaption_state_learning_coef_, 0.01),

    // @Param: GAMAF
    // @Description: Adaptive feedforward learning coef
    // @Range: 0.1 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("COEu", 5, AC_MRAC, adaption_input_learning_coef_, 0.01),

    // @Param: GAMAN
    // @Description: Adaptive nonlinear learning coef
    // @Range: 0.1 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("COEn", 6, AC_MRAC, adaption_nonlinear_learning_coef_, 0.001),

    // @Param: OCCL
    // @Description: Adaptive linear estimation optimal correction coefficient
    // @Range: 0.1 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("OCCl", 7, AC_MRAC, linear_optimal_correction_coef_, 0.5),

    // @Param: OCCN
    // @Description: Adaptive nonlinear estimation optimal correction coefficient
    // @Range: 0.1 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("OCCn", 8, AC_MRAC, nonlinear_optimal_correction_coef_, 1.0),

    // @Param: GAINX
    // @Description: Initial value of feedback gain
    // @Range: 0.1 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("Kx", 9, AC_MRAC, adaption_state_gain_, 0),

    // @Param: GAINF
    // @Description: Initial value of feedforward gain
    // @Range: 0.1 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("Ku", 10, AC_MRAC, adaption_input_gain_, 0),

    // @Param: TYPE
    // @Description: Controller type 0: mrac 1: least square
    // @User: Standard
    AP_GROUPINFO("TYPE", 11, AC_MRAC, type_, 0),

    // @Param: LM
    // @Description: Maximum output value
    // @Range: 0.1 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LM", 12, AC_MRAC, limit_, 1.0),

    AP_GROUPEND
};

  AC_MRAC::AC_MRAC(float wn, float b0, float dt)
  {
    ts_ = dt;
    wn_.set_and_default(wn);
    b0_.set_and_default(b0);
  }

 void AC_MRAC::set_dt(const double dt)
 {
     ts_ = dt;
 }

double AC_MRAC::Control(const double command, const Matrix state) {

  if(!initialized_){
    initialized_ = true;
    Init();
    SetInitialReferenceState(state);
    SetInitialActionState(state);
  }

  // check if the current sampling time is valid and the reference/adaption
  // model well set up during the initialization
  if (ts_ <= 0.0 || !reference_model_enabled_ || !adaption_model_enabled_) {
    std::cout << "MRAC: model build failed; will work as a unity compensator. The "
              "reference_model building status: "
           << reference_model_enabled_
           << "; The adaption_model building status: "
           << adaption_model_enabled_ << "; Current sampling time: " << ts_ << std::endl;;
    return command;  // treat the mrac as a unity proportional controller
  }
  // update the state in the real actuation system
  state_action_.col(0) = state;
  
  // update the desired command in the real actuation system
  input_desired_(0, 0) = command;

  // update the nonlinear compoent
  nonlinear_action_(0,0) = 1;
  nonlinear_action_(1,0) = state(0,0);
  nonlinear_action_(2,0) = 2 * sq(nonlinear_action_(1,0)) - 1;
  nonlinear_action_(3,0) = 4 * pow(nonlinear_action_(1,0), 3) - 3 * nonlinear_action_(1,0); 

  // update the state in the reference system
    UpdateReference();

  // update the adaption laws including state adaption, command adaption and
  // nonlinear components adaption

    if(type_.get() == 1) {
      UpdateLeastSquareAdaption();
    } else {
      UpdateAdaption(&gain_state_adaption_, state_action_,
                  gamma_state_adaption_ * gamma_ratio_state_);

      UpdateAdaption(&gain_input_adaption_, input_desired_,
                    gamma_input_adaption_ * gamma_ratio_input_);

      UpdateNonlinearAdaption(&gain_nonlinear_adaption_, nonlinear_action_,
                    gamma_nonlinear_adaption_ * gamma_ratio_nonlinear_);
    }

  // update the generated control based on the adaptive law
  float control_unbounded =
      gain_state_adaption_.col(0).transpose() * state_action_.col(0) +
      gain_input_adaption_(0, 0) * input_desired_(0, 0) -
      gain_nonlinear_adaption_.col(0).transpose() * nonlinear_action_.col(0);

  float control = 0.0f;
  if (is_zero(limit_.get())) {
     control = control_unbounded;
  }else{
     control = constrain_value(control_unbounded, -limit_, +limit_);
  }

  // update the previous value for next iteration
  gain_state_adaption_.col(1) = gain_state_adaption_.col(0);
  gain_input_adaption_.col(1) = gain_input_adaption_.col(0);
  gain_nonlinear_adaption_.col(1) = gain_nonlinear_adaption_.col(0);

  state_reference_.col(1) = state_reference_.col(0);
  state_action_.col(1)    = state_action_.col(0);

  input_desired_.col(1)   = input_desired_.col(0);
  nonlinear_action_.col(1) = nonlinear_action_.col(0);


  control_previous_ = control;

  // log debug
  debug_info_.target = state_reference_(0,0);
  debug_info_.actual = state(0,0);
  debug_info_.error  = state_reference_(0,0) - state(0,0);
  debug_info_.P      = gain_state_adaption_(0,0);
  debug_info_.I      = gain_input_adaption_(0,0);
  debug_info_.D      = gain_nonlinear_adaption_(0,0);
  debug_info_.FF     = gain_nonlinear_adaption_.col(0).transpose() * nonlinear_action_.col(0);
  
  return control;
}

void AC_MRAC::Reset() {
    // reset the overall states
    ResetStates();
    // reset the adaptive gains
    ResetGains();
    // reset all the externally-setting, non-conf control parameters
    gamma_ratio_state_ = 1.0;
    gamma_ratio_input_ = 1.0;
    gamma_ratio_nonlinear_ = 1.0f;

    initialized_ = false;
}

void AC_MRAC::ResetStates() {
    // reset the inputs and outputs of the closed-loop MRAC controller
    control_previous_ = 0.0;
    input_desired_.setZero(1, 2);
    
    // reset the internal states, anti-windup compensations and status
    state_action_.setZero(model_order_, 2);

    state_reference_.setZero(model_order_, 2);
    nonlinear_action_.setZero(nonlinear_order_,2);
}

void AC_MRAC::ResetGains() {
    gain_state_adaption_.setZero(model_order_, 2);
    gain_input_adaption_ = Matrix::Ones(1, 2);
    gain_nonlinear_adaption_.setZero(nonlinear_order_, 2);

    gain_state_adaption_(0,1) = adaption_state_learning_coef_.get();
    gain_input_adaption_(0,1) = adaption_input_learning_coef_.get();
}

void AC_MRAC::Init() {
  control_previous_ = 0.0;

  // Initalize proj operation
  proj_oper_.init(0.5, 1.0f);

  // Initialize the system states
  input_desired_        = Matrix::Zero(1, 2);
  state_action_         = Matrix::Zero(model_order_, 2);


  state_reference_  = Matrix::Zero(model_order_, 2);
  nonlinear_action_ = Matrix::Zero(nonlinear_order_,2);

 
  // Initialize the adaptive control gains
  gain_state_adaption_ = Matrix::Zero(model_order_, 2);
  gain_input_adaption_ = Matrix::Ones(1, 2);
  gain_nonlinear_adaption_ = Matrix::Zero(nonlinear_order_, 2);
   
  gain_state_adaption_init_ = Matrix::Zero(model_order_, 1);
  gain_input_adaption_init_ = Matrix::Ones(1, 1);
  gain_nonlinear_adaption_init_ = Matrix::Zero(nonlinear_order_, 1);


  // Initialize the adaptive convergence gains and anti-windup gains
  gamma_state_adaption_ = Matrix::Zero(model_order_, model_order_);
  gamma_input_adaption_ = Matrix::Zero(1, 1);
  gamma_nonlinear_adaption_ = Matrix::Zero(1, 1);

  // Initialize the reference model parameters
  matrix_a_reference_ = Matrix::Zero(model_order_, model_order_);
  matrix_b_reference_ = Matrix::Zero(model_order_, 1);
  reference_model_enabled_ =
      (SetReferenceModel() && BuildReferenceModel());

  // Initialize the adaption model parameters
  matrix_p_adaption_ = Matrix::Zero(model_order_, model_order_);
  matrix_b_adaption_ = Matrix::Zero(model_order_, 1);
  adaption_model_enabled_ =
      (SetAdaptionModel() && BuildAdaptionModel());

  EstimateInitialGains();

  gain_state_adaption_.col(1) = gain_state_adaption_init_;
  gain_input_adaption_.col(1) = gain_input_adaption_init_;
  gain_nonlinear_adaption_.col(1) = gain_nonlinear_adaption_init_;

}

bool AC_MRAC::SetReferenceModel() {
  const double Epsilon = 0.000001;
  if (((wn_ < Epsilon && model_order_ == 1)) ||
      ((wn_ < Epsilon &&
        model_order_ == 2))) {

        std::cout << "mrac controller error: reference model natural frequency parameter: " <<
         wn_ <<
        " in configuration file are not reasonable with respect to the " <<
        "reference model order: " <<
        model_order_ << std::endl;
    
    return false;
  }
 
  wn_reference_   = wn_;

  return true;
}

bool AC_MRAC::SetAdaptionModel() {

  for (int i = 0; i < model_order_; ++i) {
     gamma_state_adaption_(i, i) = adaption_state_learning_coef_;           
  }

   for (int j = 0; j < model_order_; ++j) {
      matrix_p_adaption_(j, j) = 1;
    }

  gamma_input_adaption_(0, 0)    = adaption_input_learning_coef_;
  gamma_nonlinear_adaption_(0,0) = adaption_nonlinear_learning_coef_;
  
  return true;
}

bool AC_MRAC::BuildReferenceModel() {
  if (model_order_ > 2) {
  
      std::cout << "mrac controller error: reference model order " <<
                     model_order_ << " is beyond the designed range" << std::endl;

    return false;
  }
  if (model_order_ == 1) {
    matrix_a_reference_(0, 0) = - wn_reference_;
    matrix_b_reference_(0, 0) = wn_reference_;
  } else if (model_order_ == 2) {
    matrix_a_reference_(0, 1) = 1.0;
    matrix_a_reference_(1, 0) = - wn_reference_ * wn_reference_;
    matrix_a_reference_(1, 1) = -2 * zeta_reference_ * wn_reference_;
    matrix_b_reference_(1, 0) = wn_reference_ * wn_reference_;
  }
  return true;
}

bool AC_MRAC::BuildAdaptionModel() {
  if (model_order_ > 2) {

        std::cout << "mrac controller error: adaption model order " <<
                     model_order_ << " is beyond the designed range" << std::endl;
        return false;
  }

  if (model_order_ == 1) {
    matrix_b_adaption_(0, 0) = b0_;
  } else if (model_order_ == 2) {
    matrix_b_adaption_(1, 0) = b0_;
  }

  if (!CheckLyapunovPD(matrix_a_reference_, matrix_p_adaption_)) {
    const std::string error_msg =
        "Solution of the algebraic Lyapunov equation is not symmetric positive "
        "definite";
    std::cout << error_msg << std::endl;
    return false;
  }

  return true;
}

bool AC_MRAC::CheckLyapunovPD(const Matrix matrix_a,
                                     const Matrix matrix_p) const {
  Matrix matrix_q = -matrix_p * matrix_a - matrix_a.transpose() * matrix_p;
  Eigen::LLT<Matrix> llt_matrix_q(matrix_q);
  // if matrix Q is not symmetric or the Cholesky decomposition (LLT) failed
  // due to the matrix Q are not positive definite
  return (matrix_q.isApprox(matrix_q.transpose()) &&
          llt_matrix_q.info() != Eigen::NumericalIssue);
}

void AC_MRAC::EstimateInitialGains() {


  if (model_order_ == 1){
    gain_state_adaption_init_(0,0) = adaption_state_gain_;
    gain_input_adaption_init_(0,0) = adaption_input_gain_;
  } else if (model_order_ == 2 ) {
    gain_state_adaption_init_.col(0) << adaption_state_gain_,adaption_state_gain_;
    gain_input_adaption_init_(0,0) = adaption_input_gain_;
  } else {
    std::cout << "No pre-known actuation dynamics; the initial states of the "
             "adaptive gains are set as zeros" << std::endl;
  }
}

void AC_MRAC::UpdateReference() {
  Matrix matrix_i = Matrix::Identity(model_order_, model_order_);
  state_reference_.col(0) =
      (matrix_i - ts_ * 0.5 * matrix_a_reference_).inverse() *
      ((matrix_i + ts_ * 0.5 * matrix_a_reference_) * state_reference_.col(1) +
       ts_ * 0.5 * matrix_b_reference_ *
           (input_desired_(0, 0) + input_desired_(0, 1)));
}

void AC_MRAC::UpdateAdaption(Matrix *law_adp, const Matrix state_adp,
                                    const Matrix gain_adp) {
  Matrix state_error = state_action_ - state_reference_;

  Matrix dgain = 
      -0.5  * gain_adp * 
          (state_adp.col(0) * (state_error.col(0).transpose()) +
           state_adp.col(1) * (state_error.col(1).transpose())) *
          matrix_p_adaption_ * matrix_b_adaption_ +
      0.5  * gain_adp * linear_optimal_correction_coef_ * 
          (state_adp.col(0) * state_adp.col(0).transpose() +
           state_adp.col(1) * state_adp.col(1).transpose()) * 
          law_adp->col(1) * 
          matrix_b_adaption_.transpose() * 
          matrix_p_adaption_ * matrix_a_reference_.inverse() *
          matrix_b_adaption_;

   law_adp->col(0) = law_adp->col(1) + ts_ * dgain;
}

void AC_MRAC::UpdateNonlinearAdaption(Matrix *law_adp, const Matrix state_adp,
                                  const Matrix gain_adp) 
{

  Matrix state_error = state_action_ - state_reference_;

  Matrix dgain = 
    0.5 * gain_adp(0,0) * 
        (state_adp.col(0) * (state_error.col(0).transpose()) +
          state_adp.col(1) * (state_error.col(1).transpose())) *
        matrix_p_adaption_ * matrix_b_adaption_ +
    0.5 * gain_adp(0,0) * nonlinear_optimal_correction_coef_ * 
        (state_adp.col(0) * state_adp.col(0).transpose() +
          state_adp.col(1) * state_adp.col(1).transpose()) * 
        law_adp->col(1) * 
        matrix_b_adaption_.transpose() * 
        matrix_p_adaption_ * matrix_a_reference_.inverse() *
        matrix_b_adaption_;

  // Projective operations
  dgain = proj_oper_.update(law_adp->col(1), dgain);

  law_adp->col(0) = law_adp->col(1) + ts_ * dgain;
}

void AC_MRAC::UpdateLeastSquareAdaption()
{
  // Update gain_input_adaption
  gain_input_adaption_(0,0) = matrix_b_reference_(0,0)/matrix_b_adaption_(0,0);

  // Update gain_state_adaption
  Matrix model_error = matrix_a_reference_ * state_action_.col(0) + matrix_b_reference_ * input_desired_.col(0) - 
  (state_action_.col(0) - state_action_.col(1))/ts_;


  gain_state_adaption_.col(0) =  gain_state_adaption_.col(1) 
                              + ts_ * gamma_state_adaption_(0,0) * gamma_ratio_state_
                              * state_action_.col(0) * model_error.transpose() 
                              * matrix_b_adaption_ * (matrix_b_adaption_.transpose() * matrix_b_adaption_).inverse();
  

  // Update nonlinear gain
  gain_nonlinear_adaption_.col(0) =  gain_nonlinear_adaption_.col(1) 
                                  - ts_ * gamma_nonlinear_adaption_(0,0) * gamma_ratio_nonlinear_
                                  * nonlinear_action_.col(0) * model_error.transpose() 
                                  * matrix_b_adaption_ * (matrix_b_adaption_.transpose() * matrix_b_adaption_).inverse();
   

}


void AC_MRAC::SetInitialReferenceState(
    const Matrix &state_reference_init) {
  if (state_reference_init.rows() != model_order_ ||
      state_reference_init.cols() != 1) {
    std::cout << "failed to set the initial reference states, due to the given "
             "state size: "
          << state_reference_init.rows() << " x " << state_reference_init.cols()
          << " doesn't match the model order: " << model_order_ << std::endl;
  } else {
    state_reference_.col(1) = state_reference_init;
  }
}

void AC_MRAC::SetInitialActionState(const Matrix &state_action_init) {
  if (state_action_init.rows() != model_order_ ||
      state_action_init.cols() != 1) {
    std::cout << "failed to set the initial action states, due to the given "
             "state size: "
          << state_action_init.rows() << " x " << state_action_init.cols()
          << " doesn't match the model order: " << model_order_ << std::endl;
  } else {
    state_action_.col(1) = state_action_init;
  }
}

void AC_MRAC::SetInitialCommand(const double command_init) {
  input_desired_(0, 1) = command_init;
}

void AC_MRAC::SetInitialStateAdaptionGain(
    const Matrix &gain_state_adaption_init) {
  if (gain_state_adaption_init.rows() != model_order_ ||
      gain_state_adaption_init.cols() != 1) {
    std::cout << "failed to set the initial state adaption gains, due to the given "
             "state size: "
          << gain_state_adaption_init.rows() << " x "
          << gain_state_adaption_init.cols()
          << " doesn't match the model order: " << model_order_ << std::endl;
  } else {
    gain_state_adaption_.col(1) = gain_state_adaption_init;
  }
}

void AC_MRAC::SetInitialInputAdaptionGain(
    const double gain_input_adaption_init) {
  gain_input_adaption_(0, 1) = gain_input_adaption_init;
}

void AC_MRAC::SetInitialNonlinearAdaptionGain(
    const Matrix &gain_nonlinear_adaption_init) {
  gain_nonlinear_adaption_.col(1) = gain_nonlinear_adaption_init;
}

void AC_MRAC::SetStateAdaptionRate(const double ratio_state) {
  if (ratio_state < 0.0) {
    std::cout << "failed to set the state adaption rate, due to new ratio < 0; the "
             "current ratio is still: "
          << gamma_ratio_state_ << std::endl;
  } else {
    gamma_ratio_state_ = ratio_state;
  }
}

void AC_MRAC::SetInputAdaptionRate(const double ratio_input) {
  if (ratio_input < 0.0) {
    std::cout << "failed to set the input adaption rate, due to new ratio < 0; the "
             "current ratio is still: "
          << gamma_ratio_input_ << std::endl;
  } else {
    gamma_ratio_input_ = ratio_input;
  }
}

void AC_MRAC::SetNonlinearAdaptionRate(const double ratio_nonlinear) {
  if (ratio_nonlinear < 0.0) {
     std::cout << "failed to set the nonlinear adaption rate, due to new ratio < 0; "
             "the current ratio is still: "
          << gamma_ratio_nonlinear_ << std::endl;
  } else {
    gamma_ratio_nonlinear_ = ratio_nonlinear;
  }
}


double AC_MRAC::StateAdaptionRate() const { return gamma_ratio_state_; }

double AC_MRAC::InputAdaptionRate() const { return gamma_ratio_input_; }

double AC_MRAC::NonlinearAdaptionRate() const {
  return gamma_ratio_nonlinear_;
}

Matrix AC_MRAC::CurrentReferenceState() const {
  return state_reference_;
}

Matrix AC_MRAC::CurrentStateAdaptionGain() const {
  return gain_state_adaption_;
}

Matrix AC_MRAC::CurrentInputAdaptionGain() const {
  return gain_input_adaption_;
}

Matrix AC_MRAC::CurrentNonlinearAdaptionGain() const {
  return gain_nonlinear_adaption_;
}




