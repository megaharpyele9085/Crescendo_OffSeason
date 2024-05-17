// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "rev/CanSparkMax.h"

#include "Constants.h"

//Pid Libary
#include <frc/controller/PIDController.h>

//ShuffleBoard Default Libraries
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void UpArmToChain(double desiredMaxHigh_Parameter);

  void DownArmToRobot(double desiredMinHigh_Parameter);

  void StopElevator();
 
  void ResetElevatorEncoders();

  void UpRightElevator();

  void UpLeftElevator();

  void DownRightElevator();

  void DownLeftElevator();

  void StopLeftElevator();

  void StopRightElevator();

  double GetLeftElevatorPosition();

  double GetRightElevatorPosition();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

//Creating the two motors wich we're gonna use at this subsystem
rev::CANSparkMax leftElevatorMotor{ElevatorConstants::leftElevatorMotorId,rev::CANSparkLowLevel::MotorType::kBrushless};
rev::CANSparkMax rightElevatorMotor{ElevatorConstants::rightElevatorMotorId,rev::CANSparkLowLevel::MotorType::kBrushless};



//Right Elevator Motor Encoder
rev::SparkRelativeEncoder leftElevatorEncoder;

//Right Elevator Motor Encoder
rev::SparkRelativeEncoder rightElevatorEncoder;

double rightElevatorPid_P;
double rightElevatorPid_I;
double rightElevatorPid_D;


double leftElevatorPid_P;
double leftElevatorPid_I;
double leftElevatorPid_D;


//The Pid object to reach the desired postion and stay
frc::PIDController rightElevatorPid{rightElevatorPid_P,rightElevatorPid_I,rightElevatorPid_D};
frc::PIDController leftElevatorPid{leftElevatorPid_P,leftElevatorPid_I,leftElevatorPid_D};

//The desired position to be initialized when we call a position paramter of the
//Up(Max) or Down(Min)  method
double desiredMaxHigh;
double desiredMinHigh;


};
