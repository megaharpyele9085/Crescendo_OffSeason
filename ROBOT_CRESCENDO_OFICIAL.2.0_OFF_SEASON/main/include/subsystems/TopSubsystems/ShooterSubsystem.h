// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

//Phoenix 6 Library
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/MotionMagicVoltage.hpp"

//Phoenix 5 Library
// #include "ctre/Phoenix.h"

//Shooter Constants
#include <Constants.h>

//Filter library
#include "frc/filter/SlewRateLimiter.h"

#include "frc/motorcontrol/MotorControllerGroup.h"

//ShuffleBoard Default Libraries
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <units/velocity.h>
#include <units/acceleration.h>

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;


  void LaunchShooterCommand(double LaunchShooter_SpeedParameter);

  void CathShooterCommand(double CatchShooter_SpeedParameter);

  void LaunchAmpVelocity();

  void StopShooterCommand();

  double GetRightShooterVelocity();

  double GetLeftShooterVelocity();

 private:
// Components (e.g. motor controllers and sensors) should generally be
// declared private and exposed only through public methods.

//The Shooter Motor Phoenix 5
// ctre::phoenix::motorcontrol::can::WPI_TalonFX rightShooterMotor{ShooterConstants::rightShooterMotorId};
// ctre::phoenix::motorcontrol::can::WPI_TalonFX leftShooterMotor{ShooterConstants::leftShooterMotorId};


//The Shooter Motor Phoenix 6 Test
ctre::phoenix6::hardware::TalonFX rightShooterMotor{ShooterConstants::rightShooterMotorId};
ctre::phoenix6::hardware::TalonFX leftShooterMotor{ShooterConstants::leftShooterMotorId};

// Creating the motion magic object voltage , Phoenix 6
ctre::phoenix6::controls::MotionMagicVelocityVoltage m_mmReqLeft{0_tps};
ctre::phoenix6::controls::MotionMagicVelocityVoltage m_mmReqRight{0_tps};


//The Shooter Filter
//right Shooter filter   
frc::SlewRateLimiter<units::scalar> rightShooter_Filter{0.05 / 20_ms};

//left intake filter
frc::SlewRateLimiter<units::scalar> LeftShooter_Filter{0.05 / 20_ms};

//The speeds to be aplied
double LaunchShooterSpeed;
double CatchShooterSpeed;
double RightSpeed = 10;
double LeftSpeed = 10;

// ctre::phoenix6::controls::VelocityDutyCycle RightSpeedDutyCycle;
// ctre::phoenix6::controls::VelocityDutyCycle LeftSpeedDutyCycle;

};
