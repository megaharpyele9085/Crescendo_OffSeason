// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "subsystems/LimeLightBackSubsystem.h"

//Shuffle Board Default Libraries
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

//Rev Driver Library
#include "rev/CanSparkMax.h"

//CanCoder Library
#include "ctre/Phoenix.h"

//Constants
#include "Constants.h"

//Pid Libary
#include <frc/controller/PIDController.h>
#include "PIDController9085.h"

//Phoenix 6 library 
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/configs/Configurator.hpp"
#include "ctre/phoenix6/configs/Configs.hpp"

#include <algorithm>
#include <cmath>

class GiraffeSubsystem : public frc2::SubsystemBase {
 public:

  GiraffeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetDesiredAngle(double desiredAngle_Parameter);

  //One Method which is going to test this subsystem at Test Init/Periodic
  //method by a test  motor potency
  void SetTestPotUpGiraffe();

  //One Method which is going to test this subsystem at Test Init/Periodic
  //method by a test  motor potency
  void SetTestPotDownGiraffe();

  void StopGiraffeMotor();

  double GetAbsoluteGiraffeAngle();

  double GetErrorPID();

  double GetAbsoluteEncoderRealRange();

  double GetCalculate();

  bool IsGiraffeOnTop();

  double SetPointAngCalculate(double InputX);

  //The variable which is going to be my setPoint by the default  parameter
 double desiredAngle;

 //The Pid Giraffe object
 frc2::PIDController9085 GiraffePid{GiraffePid_P,GiraffePid_I,GiraffePid_D, 20, 20_ms, 0.39364_s, 0.01_s};

 private:
// Components (e.g. motor controllers and sensors) should generally be
// declared private and exposed only through public methods.

//The Giraffe motor object
rev::CANSparkMax GiraffeMotor{GiraffeConstants::GiraffeMotorId,rev::CANSparkLowLevel::MotorType::kBrushless};

//The Giraffe CanCoder Object
ctre::phoenix6::hardware::CANcoder GiraffeCanCoder{CanSensorConstants::GiraffeCanCoderID};

//LimeLightBack
LimeLightBackSubsystem Lime_Back{};


double GiraffePid_P = -0.65;
double GiraffePid_I = 0.0;
double GiraffePid_D = 0;

double GiraffeSpeeds;

double GiraffeError;

};
