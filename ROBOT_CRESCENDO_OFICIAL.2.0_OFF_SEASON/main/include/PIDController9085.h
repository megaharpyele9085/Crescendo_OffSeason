// PID 9085

#pragma once

#include <functional>
#include <limits>

#include <wpi/SymbolExports.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include "units/time.h"

namespace frc2 {

/**
 * Implements a PID control loop.
 */
class WPILIB_DLLEXPORT PIDController9085
    : public wpi::Sendable,
      public wpi::SendableHelper<PIDController9085> {
 public:
  /**
   * Allocates a PIDController with the given constants for Kp, Ki, and Kd.
   *
   * @param Kp      The proportional coefficient.
   * @param Ki      The integral coefficient.
   * @param Kd      The derivative coefficient.
   * @param period  The period between controller updates in seconds. The
   *                default is 20 milliseconds. Must be non-zero and positive.
   * @param n_pole  Pole for derivative filter.
   * @param Ti      The integral time constant.
   * @param Td      The time constant of filter derivative.
   */
  PIDController9085(double Kp, double Ki, double Kd, double n_pole,
                    units::second_t period = 20_ms, units::second_t Ti = 1_ms, units::second_t Td = 2_ms);

                    

  ~PIDController9085() override = default;

  PIDController9085(const PIDController9085&) = default;
  PIDController9085& operator=(const PIDController9085&) = default;
  PIDController9085(PIDController9085&&) = default;
  PIDController9085& operator=(PIDController9085&&) = default;

  /**
   * Sets the PID Controller gain parameters.
   *
   * Sets the proportional, integral, and differential coefficients.
   *
   * @param Kp Proportional coefficient
   * @param Ki Integral coefficient
   * @param Kd Differential coefficient
   */
  void SetPIDGain(double Kp, double Ki, double Kd);

  /**
   * Sets the proportional coefficient of the PID controller gain.
   *
   * @param Kp proportional coefficient
   */
  void SetP(double Kp);

  /**
   * Sets the PID Controller gain parameters and constant periods.
   *
   * Sets the proportional, integral, differential coefficients, time Ti, time Td and derivative pole.
   *
   * @param Kp      Proportional coefficient.
   * @param Ki      Integral coefficient.
   * @param Kd      Differential coefficient.
   * @param n_pole  Pole for derivative filter.
   * @param Ti      The integral time constant.
   * @param Td      The time constant of filter derivative.
   */
  void SetPID(double Kp, double Ki, double Kd,
                             double n_pole, units::second_t Ti,
                             units::second_t Td);

  /**
   * Sets the integral coefficient of the PID controller gain.
   *
   * @param Ki integral coefficient
   */
  void SetI(double Ki);

  /**
   * Sets the differential coefficient of the PID controller gain.
   *
   * @param Kd differential coefficient
   */
  void SetD(double Kd);

  /**
   * Gets the proportional coefficient.
   *
   * @return proportional coefficient
   */
  double GetP() const;

  /**
   * Gets the integral coefficient.
   *
   * @return integral coefficient
   */
  double GetI() const;

  /**
   * Gets the differential coefficient.
   *
   * @return differential coefficient
   */
  double GetD() const;

  /**
   * Gets the period of this controller.
   *
   * @return The period of the controller.
   */
  units::second_t GetPeriod() const;

  /**
   * Sets the setpoint for the PIDController.
   *
   * @param setpoint The desired setpoint.
   */
  void SetSetpoint(double setpoint);

  /**
   * Returns the current setpoint of the PIDController.
   *
   * @return The current setpoint.
   */
  double GetSetpoint() const;

  /**
   * Enables continuous input.
   *
   * Rather then using the max and min input range as constraints, it considers
   * them to be the same point and automatically calculates the shortest route
   * to the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  void EnableContinuousInput(double minimumInput, double maximumInput);

  /**
   * Disables continuous input.
   */
  void DisableContinuousInput();

  /**
   * Returns true if continuous input is enabled.
   */
  bool IsContinuousInputEnabled() const;

  /**
   * Sets the minimum and maximum values for output controller.
   *
   *
   * @param minimum_out The minimum value of the integrator.
   * @param max_out The maximum value of the integrator.
   */
  void SetOutputRange(double minimum_out, double max_out);

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param setpoint The new setpoint of the controller.
   */
  double Calculate(double measurement, double setpoint);

  /**
   * Reset the previous error, the integral term, and disable the controller.
   */
  void Reset();


  /**
   *  @return The Error.
   */
  double GetError();


  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  // Factor for "proportional" control
  double m_Kp;

  // Factor for "integral" control
  double m_Ki;

  // Factor for "derivative" control
  double m_Kd;

  // The period (in seconds) of the control loop running this controller
  units::second_t m_period;

  double m_max_out = 1.0;

  double m_minimum_out = -1.0;

  double m_maximumInput = 0;

  double m_minimumInput = 0;

  // Do the endpoints wrap around? eg. Absolute encoder
  bool m_continuous = false;

  double m_setpoint = 0;
  double m_measurement = 0;
  double m_positionError = 0;
  double m_n_pole = 0;
  double u_P = 0;
  double u_I = 0;
  double u_D = 0;
  double e_I = 0;
  double output = 0;
  double u_I_ant = 0;
  double measurement_ant = 0;
  double u_ant = 0;
  double u_D_ant = 0;
  units::second_t m_Ti = 1_ms;
  units::second_t m_Td = 2_ms;
};

}  // namespace frc2

namespace frc {

using frc2::PIDController9085;

}  // namespace frc
