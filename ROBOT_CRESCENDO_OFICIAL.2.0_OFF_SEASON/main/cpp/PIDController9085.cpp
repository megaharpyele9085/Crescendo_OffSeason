// PID 9085

#include "PIDController9085.h"

#include <algorithm>
#include <cmath>

#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>  

#include "frc/MathUtil.h"
#include "wpimath/MathShared.h"

using namespace frc2;

PIDController9085::PIDController9085(double Kp, double Ki, double Kd, double n_pole,
                                     units::second_t period, units::second_t Ti, units::second_t Td)
:m_Kp(Kp), m_Ki(Ki), m_Kd(Kd), m_n_pole(n_pole), m_period(period), m_Ti(Ti), m_Td(Td) {
  if (period <= 0_s || Ti <= 0_s) {
    wpi::math::MathSharedStore::ReportError(
        "Controller period and Ti must be a non-zero positive number, got {}!",
        period.value());
    m_period = 20_ms;
    m_Ti = 1_ms;
    wpi::math::MathSharedStore::ReportWarning(
        "Controller period defaulted to 20ms and Ti to 1ms.");
  }
  static int instances = 0;
  instances++;

  wpi::math::MathSharedStore::ReportUsage(
      wpi::math::MathUsageId::kController_PIDController2, instances);
  wpi::SendableRegistry::Add(this, "PIDController9085", instances);
}

void PIDController9085::SetPIDGain(double Kp, double Ki, double Kd) {
  m_Kp = Kp;
  m_Ki = Ki;
  m_Kd = Kd;
}

void PIDController9085::SetPID(double Kp, double Ki, double Kd,
                             double n_pole, units::second_t Ti, units::second_t Td) {
  m_Kp = Kp;
  m_Ki = Ki;
  m_Kd = Kd;
  m_n_pole = n_pole;
  m_Td = Td;

 if (Ti <= 0_s) {
    wpi::math::MathSharedStore::ReportError(
        "Ti must be a non-zero positive number, got {}!",
        Ti.value());
        m_Ti = 1_ms;
    wpi::math::MathSharedStore::ReportWarning(
        "Ti defaulted to 1ms.");
  }
}

void PIDController9085::SetP(double Kp) {
  m_Kp = Kp;
}

void PIDController9085::SetI(double Ki) {
  m_Ki = Ki;
}

void PIDController9085::SetD(double Kd) {
  m_Kd = Kd;
}

double PIDController9085::GetP() const {
  return m_Kp;
}

double PIDController9085::GetI() const {
  return m_Ki;
}

double PIDController9085::GetD() const {
  return m_Kd;
}

units::second_t PIDController9085::GetPeriod() const {
  return m_period;
}

void PIDController9085::SetSetpoint(double setpoint) {
  m_setpoint = setpoint;

  if (m_continuous) {
    double errorBound = (m_maximumInput - m_minimumInput)*0.065; //erro maximo admitido=6.5%
  
    if ((abs)(m_setpoint - m_measurement) >= errorBound){
      m_positionError = m_setpoint - m_measurement;  
    } else{
      m_positionError = 0;
    }
  
  } else {
    m_positionError = m_setpoint - m_measurement;
  }
}

double PIDController9085::GetSetpoint() const {
  return m_setpoint;
}


void PIDController9085::EnableContinuousInput(double minimumInput,
                                          double maximumInput) {
  m_continuous = true;
  m_minimumInput = minimumInput;
  m_maximumInput = maximumInput;
}

void PIDController9085::DisableContinuousInput() {
  m_continuous = false;
}

bool PIDController9085::IsContinuousInputEnabled() const {
  return m_continuous;
}

void PIDController9085::SetOutputRange(double minimum_out,
                                       double max_out) {
  m_minimum_out = minimum_out;
  m_max_out = max_out;
}

double PIDController9085::Calculate(double measurement, double setpoint) {
  m_measurement = measurement;
  m_setpoint = setpoint;

  if (m_continuous){
    double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
    m_positionError =
        frc::InputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
  } else {
    m_positionError = m_setpoint - m_measurement;
  }

  if (u_ant >= m_max_out || u_ant <= m_minimum_out){    //anti wind-up
    e_I = 0;
  } else {
    e_I = m_positionError;
  }
    
  u_P = m_Kp*m_positionError;
  u_I = u_I_ant + ((m_Ki*m_period.value()/m_Ti.value())*e_I);
  u_D = ((m_Td.value()/(m_Td.value()+(m_n_pole*m_period.value())))*u_D_ant) - 
        (((m_Kd*m_n_pole*m_Td.value())/(m_Td.value()+(m_n_pole*m_period.value())))*measurement) +
        (((m_Kd*m_n_pole*m_Td.value())/(m_Td.value()+(m_n_pole*m_period.value())))*measurement_ant);

  output = std::clamp((u_P + u_I + u_D), m_minimum_out, m_max_out); //saturador

  u_I_ant = u_I;
  measurement_ant = measurement;
  u_ant = output;
  u_D_ant = u_D;

  return output;
}

void PIDController9085::Reset() {
  m_positionError = 0;
  measurement_ant = 0;
  u_I_ant = 0; 
  u_ant = 0;
  u_D_ant = 0;
}

double PIDController9085::GetError() {
  return m_positionError;
}


void PIDController9085::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("PIDController9085");
  builder.AddDoubleProperty(
      "p", [this] { return GetP(); }, [this](double value) { SetP(value); });
  builder.AddDoubleProperty(
      "i", [this] { return GetI(); }, [this](double value) { SetI(value); });
  builder.AddDoubleProperty(
      "d", [this] { return GetD(); }, [this](double value) { SetD(value); });
  builder.AddDoubleProperty(
      "setpoint", [this] { return GetSetpoint(); },
      [this](double value) { SetSetpoint(value); });
}
