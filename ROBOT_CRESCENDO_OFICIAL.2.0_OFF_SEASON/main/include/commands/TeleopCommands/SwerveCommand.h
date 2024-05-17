// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>


//The subsystem library which we're going to use
#include <subsystems/SwerveSubsystem.h>
#include <frc/controller/PIDController.h>

//Function library - C++
#include <functional>

//Default C++
#include <iostream>
#include <algorithm>

//Filter
#include <frc/filter/SlewRateLimiter.h>

//DriveStation
#include <frc/DriverStation.h>

//ChassisSpeed library
#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/kinematics/SwerveModuleState.h>

//control librarie
#include "subsystems/ControlSubsystem.h"

//LimeLights Subsystems 
#include "subsystems/LimeLightFrontSubsystem.h"
#include "subsystems/LimeLightBackSubsystem.h"

//Flip Subsystems
#include "subsystems/TopSubsystems/FlipSubsystem.h"

//Intake Subsystem
#include "subsystems/TopSubsystems/IntakeSubsystem.h"

//Giraffe Subsystem
#include "subsystems/TopSubsystems/GiraffeSubsystem.h"

//Shooter Subsystem
#include "subsystems/TopSubsystems/ShooterSubsystem.h"

//Elevator Subsystem
#include "subsystems/TopSubsystems/ElevatorSubsystem.h"

//Ultrasonic Subsystem
#include "subsystems/UltrasonicSubsystem.h"


//ShuffleBoard Default Libraries
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SwerveCommand
    : public frc2::CommandHelper<frc2::Command, SwerveCommand> {
 public:

  //The default class ctor, the parameter classes is the function for spds and FieldOriented
  explicit SwerveCommand(SwerveSubsystem*swerve_SubsystemParameter,
                         ControlSubsystem*control_SubsystemParameter,
                         IntakeSubsystem*intake_SubsystemParameter,
                         ShooterSubsystem*shooter_SubsystemParameter,
                         //FlipSubsystem*flip_SubsystemParameter,
                         GiraffeSubsystem*giraffe_SubsystemParameter,
                         ElevatorSubsystem*elevator_SubsystemParameter,
                         //UltrasonicSubsystem*ultrasonic_SubsystemParameter,
                         LimeLightFrontSubsystem*limeLightFront_SubsystemParameter,
                         LimeLightBackSubsystem*limeLightBack_SubsystemParameter
                        );
  
 

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;
  
  bool IsFinished() override;


  private :

  //The subsystems wich we're going to use in swerve Command
  SwerveSubsystem* swerve_Subsystem;

  //The Intake Subsystem to be initialized in cpp
  IntakeSubsystem* intake_Subsystem;

  //The flip Subsystem to be initialized in cpp
 // FlipSubsystem* flip_Subsystem;

  
  //The flip Subsystem to be initialized in cpp
  ShooterSubsystem* shooter_Subsystem;

  //The giraffe Subsystem to be initialized in cpp
  GiraffeSubsystem* giraffe_Subsystem;

//The LimeLightFront Subsystem to be initialized in cpp
  LimeLightFrontSubsystem* limeLightFront_Subsystem;

//The LimeLightBack Subsystem to be initialized in cpp
  LimeLightBackSubsystem* limeLightBack_Subsystem;

//The Elevator Subsystem to be initialized in cpp
  ElevatorSubsystem* elevator_Subsystem;

  
//The Ultrasonic Subsystem to be initialized in cpp
  //UltrasonicSubsystem* ultrasonic_Subsystem;

//The Control Subsystem to be initialized in cpp
  ControlSubsystem* control;



 //The limiter for the max X axis speed
 frc::SlewRateLimiter <units::scalar> xAxisLimiter 
 {DriveConstants::kTeleDriveMaxAccelerationUnitsPerSecond/1.0_s};

   //The limiter for the max Y axis speed
 frc::SlewRateLimiter <units::scalar> yAxisLimiter
 {DriveConstants::kTeleDriveMaxAccelerationUnitsPerSecond/1.0_s};

    //The limiter for the max rotation speed
 frc::SlewRateLimiter <units::scalar> rAxisLimiter 
 {DriveConstants::kTeleDriveMaxAngularAccelerationUnitsPerSecond/1.0_s};

//LimeLight Actual Target High to be equal to 3 differents Constants
double ActualAprilTargetHigh = 0.0; 


//The returnable variable condition of right source position
bool SourcePosition_Good = true;
bool SourcePosition_NotGood= false;  

//the variable that will return true if you are at the blue alliance, and false if you are at red aliance
bool AllianceBlue;

//The Pid's for Tx and Distance of LimeLight, to input in 
//swerve x , y axis and rot . 

//The Default P I D vaariables for the Tx  PidContoller angle in the Limelight
double LimeTxPid_P = -0.012;
double LimeTxPid_I= 0;
double LimeTxPid_D = 0;



//The Default  gains for the X  PidContoller axis of the Target
double LimeDistXPid_P;
double LimeDistXPid_I;
double LimeDistXPid_D;

//The Default  gains for the Y PidContoller axis of the Target
double LimeDistYPid_P;
double LimeDistYPid_I;
double LimeDistYPid_D;

//The Default  gains for the Rot PidContoller axis of the Target
double HeadingRotPid_P;
double HeadingRotPid_I;
double HeadingRotPid_D;

//The default SetPoints for each structure**********************************************

//-------------Robot Angle Ones-------------------//
double AmpRobotAngle;
double SourceRobotAngle;
double ChainRobotAngle;

//------------Back_Tx Ones------------------//
double BackTxSource;
double BackTxAmp;

double BackTxChain_minus_one;
double BackTxChain_zero;
double BackTxChain_one;

//-------------Giraffe_Angle Ones-----------//
double Giraffe_SourceAngle;
double Giraffe_AmpAngle;

//***************************************************************************************

//For Tx  Rot tracking
frc2::PIDController9085 LimeLightPid_Tx{LimeTxPid_P, LimeTxPid_I, LimeTxPid_D, 20, 0.39364_s, 0.01_s};

//For distance in X axis
frc2::PIDController9085 LimeLightPidDist_X{LimeDistXPid_P, LimeDistXPid_I, LimeDistXPid_D, 20, 0.39364_s, 0.01_s};

//For distance in Y axis
frc2::PIDController9085 LimeLightPidDist_Y{LimeDistYPid_P,LimeDistYPid_I, LimeDistYPid_D, 20, 0.39364_s, 0.01_s};

//For the Heading to rot
frc2::PIDController9085 HeadingPidRot{HeadingRotPid_P, HeadingRotPid_I, HeadingRotPid_D, 20, 0.39364_s, 0.01_s}; 

};