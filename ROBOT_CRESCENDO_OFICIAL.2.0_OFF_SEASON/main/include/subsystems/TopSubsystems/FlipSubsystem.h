// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

#include <frc2/command/SubsystemBase.h>

//Rev Drivers Library
#include "rev/CanSparkMax.h"

//CanCoder Library_Phoenix 5
#include "ctre/Phoenix.h"


// //CanCoder Library_Phoenix 6
// #include "ctre/phoenix6/CANcoder.hpp"

//Pid Libary
#include <frc/controller/PIDController.h>

//Shuffle Board Default Libraries
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

class FlipSubsystem : public frc2::SubsystemBase {
 public:
  FlipSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void MaxAnlgeToAmp(double desiredMaxAngleToAmp_Parameter);
  
  void MinAngleToRobot(double desiredMinAngleToRobot_Parameter);
  
  void StopFlipMotor();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //The Flip Motor object
  rev::CANSparkMax flipMotor{FlipConstants::flipMotorId,rev::CANSparkLowLevel::MotorType::kBrushless};
 
  //The Flip CanCoder Object - Phoenix 5
  ctre::phoenix::sensors::CANCoder flipCanCoder{CanSensorConstants::flipCanCoderID};

  //
  // ctre::phoenix6::hardware::CANcoder flipCanCoder{CanSensorConstants::flipCanCoderID,"Flip_Cancoder"};  

  //The flip's PID
  frc::PIDController flipPid{0.01,0,0};

  //The desired angle to be initialized when we call a anlge paramter of the
  //Max or Min range  method
  double desiredMaxAngleToAmp;
  double desiredMinAngleToRobot;



};
