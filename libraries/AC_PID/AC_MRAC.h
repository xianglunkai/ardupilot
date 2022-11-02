/**
 * @file mrac controller.h
 * @brief Defines the MracController class.
 */

#pragma once

#include <vector>
#include <eigen/include/eigen3/Eigen/Core>
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger.h>

#include "AC_Projection.h"
#include "AP_PIDInfo.h"

using Matrix  = Eigen::MatrixXd;
using Vectorx = Eigen::VectorXd;

/**
 * @class MracController
 * @brief A mrac controller for actuation system (throttle/brake and steering)
 */
class AC_MRAC {
 public:

    AC_MRAC(float wn, float b0, float dt);

  /**
   * @brief initialize mrac controller
   */
  void Init(void);


  /**
   * @brief set mrac controller compute internal
   * @param dt sampling time interval
   */
  void set_dt(const double dt);

  /**
   * time constant, natural frequency and damping ratio
   * @return Status parameter initialization status
   */
    bool SetReferenceModel();

  /**
   * state adaptive gain, desired adaptive gain and nonlinear-component adaptive
   * gain
   * @return Status parameter initialization status
   */
   bool SetAdaptionModel();

  /**
   * @brief build mrac (1st or 2nd) order reference model in the discrete-time
   form, with the bilinear transform (trapezoidal integration) method
   * @return Status reference model initialization status
   */
   bool BuildReferenceModel();

  /**
   * @brief build mrac (1st or 2nd) order adaptive dynamic model in the
   * discrete-time form
   * @return Status adaption model initialization status
   */
  bool BuildAdaptionModel();

  /**
   * @brief check if the solution of the algebraic Lyapunov Equation is
   * symmetric positive definite
   * @param matrix_a reference model matrix
   * @param matrix_p Lyapunov function matrix
   * @return indicator of the symmetric positive definite matrix
   */
  bool CheckLyapunovPD(const Eigen::MatrixXd matrix_a,
                       const Eigen::MatrixXd matrix_p) const;

  /**
   * @brief estimate the initial states of the adaptive gains via known
   * actuation dynamics approximation
   */
  void EstimateInitialGains();

  /**
   * @brief execute the reference state interation with respect to the designed
   inputs in discrete-time form, with the bilinear transform (trapezoidal
   integration) method
   */
  void UpdateReference();

  /**
   * @brief execute the adaption interation with respect to the designed law in
   discrete-time form, with the bilinear transform (trapezoidal integration)
   method
   * @param law_adp adaptive law at k and k-1 steps
   * @param state_adp state used in the adaptive law at k and k-1 steps
   * @param gain_adp adaptive gain for the given adaptive law
   */
  void UpdateAdaption(Eigen::MatrixXd *law_adp, const Eigen::MatrixXd state_adp,
                      const Eigen::MatrixXd gain_adp);


  void UpdateNonlinearAdaption(Eigen::MatrixXd *law_adp, const Eigen::MatrixXd state_adp,
                      const Eigen::MatrixXd gain_adp);


  void UpdateLeastSquareAdaption();

  /**
   * @brief reset all the variables (including all the states, gains and
   * externally-setting control parameters) for mrac controller
   */
  void Reset();

  /**
   * @brief reset internal states for mrac controller
   */
  void ResetStates();

  /**
   * @brief reset adaptive gains for mrac controller
   */
  void ResetGains();

  /**
   * @brief compute control value based on the original command
   * @param command original command as the input of the actuation system
   * @param state actual output state of the actuation system
   * @param dt sampling time interval
   * @param input_limit physical or designed bound of the input
   * @param input_rate_limit physical or designed bound of the input
   * changing-rate
   * @return control value based on mrac controller architecture
   */
  virtual double Control(const double command, const Eigen::MatrixXd state);


  /**
   * @brief set initial values for state components in reference model dynamics
   * @param state_reference_init initial reference states
   */
  void SetInitialReferenceState(const Eigen::MatrixXd &state_reference_init);

  /**
   * @brief set initial values for state components in actual actuator dynamics
   * @param state_reference_init initial action states
   */
  void SetInitialActionState(const Eigen::MatrixXd &state_action_init);

  /**
   * @brief set initial command (desired input)
   * @param command_init initial desired input
   */
  void SetInitialCommand(const double command_init);

  /**
   * @brief set initial values of state adaption gains for mrac control
   * @param gain_state_adaption_init initial state adaption gains
   */
  void SetInitialStateAdaptionGain(
      const Eigen::MatrixXd &gain_state_adaption_init);

  /**
   * @brief set initial value of input adaption gain for mrac control
   * @param gain_input_adaption_init initial input adaption gain
   */
  void SetInitialInputAdaptionGain(const double gain_input_adaption_init);

  /**
   * @brief set initial value of nonlinear adaption gain for mrac control
   * @param gain_nonlinear_adaption_init initial nonlinear adaption gain
   */
  void SetInitialNonlinearAdaptionGain(
      const Eigen::MatrixXd &gain_nonlinear_adaption_init);

  /**
   * @brief set convergence ratio for state components in adaptive dynamics
   * @param ratio_state convergence ratio for state adaption
   */
  void SetStateAdaptionRate(const double ratio_state);

  /**
   * @brief set convergence ratio for input components in adaptive dynamics
   * @param ratio_input convergence ratio for input adaption
   */
  void SetInputAdaptionRate(const double ratio_input);

  /**
   * @brief set convergence ratio for nonlinear components in adaptive dynamics
   * @param ratio_nonlinear convergence ratio for additional nonlinear adaption
   */
   void SetNonlinearAdaptionRate(const double ratio_nonlinear);

  /**
   * @brief get convergence ratio for state components in adaptive dynamics
   * @return state adaption ratio
   */
  double StateAdaptionRate() const;

  /**
   * @brief get convergence ratio for input components in adaptive dynamics
   * @return input adaption ratio
   */
  double InputAdaptionRate() const;

  /**
   * @brief get convergence ratio for nonlinear components in adaptive dynamics
   * @return nonlinear adaption ratio
   */
  double NonlinearAdaptionRate() const;

  /**
   * @brief get saturation status for reference system
   * @return saturation status
   */
  int ReferenceSaturationStatus() const;

  /**
   * @brief get saturation status for control system
   * @return saturation status
   */
  int ControlSaturationStatus() const;

  /**
   * @brief get current state for reference system
   * @return current state
   */
  Eigen::MatrixXd CurrentReferenceState() const;

  /**
   * @brief get current state adaptive gain for mrac control
   * @return current state adaptive gain
   */
  Eigen::MatrixXd CurrentStateAdaptionGain() const;

  /**
   * @brief get current input adaptive gain for mrac control
   * @return current input adaptive gain
   */
  Eigen::MatrixXd CurrentInputAdaptionGain() const;

  /**
   * @brief get current nonlinear adaptive gain for mrac control
   * @return current nonlinear adaptive gain
   */
  Eigen::MatrixXd CurrentNonlinearAdaptionGain() const;

  const AP_PIDInfo& get_debug_info(void) const { return debug_info_; }

   // parameter var table
  static const struct AP_Param::GroupInfo var_info[];

 protected:
  // indicator if the reference/adaption model is valid
  bool reference_model_enabled_ = false;
  bool adaption_model_enabled_ = false;

  // indicator if clamp the adaption laws
  bool adaption_clamping_enabled = false;

  // The order of the reference/adaption model
  static const int model_order_ = 1;
  static const int nonlinear_order_ = 4;

  //Reference system coefficients in continuous-time domain
  double wn_reference_ = 0.0;
  double zeta_reference_ = 0.0;

  double ts_ = 0.01;  // By default, control sampling time is 0.01 sec

  // Adaption system coefficients
  // State adaption gain
  Eigen::MatrixXd gamma_state_adaption_;
  // Desired command adaption gain
  Eigen::MatrixXd gamma_input_adaption_;
  // Nonlinear dynamics adaption gain
  Eigen::MatrixXd gamma_nonlinear_adaption_;

  // Adjustable ratio of the state adaption gain
  double gamma_ratio_state_ = 1.0;
  // Adjustable ratio of the desired command adaption gain
  double gamma_ratio_input_ = 1.0;
  // Adjustable ratio of the nonlinear dynamics adaption gain
  double gamma_ratio_nonlinear_ = 1.0;


  // Reference system matrix in continuous-time domain
  Eigen::MatrixXd matrix_a_reference_;  // Matrix A in reference models
  Eigen::MatrixXd matrix_b_reference_;  // Matrix B in reference models


  // Adaption system matrix in discrete-time domain
  Eigen::MatrixXd matrix_p_adaption_;  // Matrix P in adaption models
  Eigen::MatrixXd matrix_b_adaption_;  // Matrix B in adaption models

  // Adaption system input variables in discrete-time domain
  Eigen::MatrixXd input_desired_;            // Updated desired command vector r
  Eigen::MatrixXd state_action_;             // Updated actuation states vector xp
  Eigen::MatrixXd nonlinear_action_;
  

  Eigen::MatrixXd state_reference_;          // Reference states vector xm

  Eigen::MatrixXd gain_state_adaption_;      // State adaption vector   theta_x
  Eigen::MatrixXd gain_input_adaption_;      // Desired command adaption vector theta_r
  Eigen::MatrixXd gain_nonlinear_adaption_;  // Nonlinear adaption vector 
  

  Eigen::MatrixXd gain_state_adaption_init_;      // Initial state adaption
  Eigen::MatrixXd gain_input_adaption_init_;      // Initial input adaption
  Eigen::MatrixXd gain_nonlinear_adaption_init_;  // Initial nonlinear adaption


  // Mrac control output in the last step
  double control_previous_ = 0.0;
  bool initialized_{false};

  Projection proj_oper_;

  AP_PIDInfo debug_info_;

  private:   
    // Common parameters
    AP_Float wn_;                                // default[10]
    AP_Float b0_;                                // default[1.0]                            

    AP_Float adaption_state_learning_coef_;      // default[0.0001]
    AP_Float adaption_input_learning_coef_;       // default[0.0001]
    AP_Float adaption_nonlinear_learning_coef_;  // default[0.0001]

    AP_Float linear_optimal_correction_coef_;   
    AP_Float nonlinear_optimal_correction_coef_;

    AP_Float adaption_state_gain_;
    AP_Float adaption_input_gain_;

    AP_Int8  type_;
    AP_Float limit_;
};


