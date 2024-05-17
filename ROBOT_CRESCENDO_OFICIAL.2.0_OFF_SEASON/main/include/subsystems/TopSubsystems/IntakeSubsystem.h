// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include  <frc2/command/CommandPtr.h>

// //Put it back when the ctre victors turn into rev CanSparksMx
#include "rev/CanSparkMax.h"  


//Put it back when the  rev CanSparkMax turn into ctre Victors
#include "ctre/Phoenix.h"

//Shooter Constants
#include <Constants.h>

//MotorGroup Library
#include "frc/motorcontrol/MotorControllerGroup.h"

//Shuffle Board Default Libraries
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>


//The  Intake filter
#include "frc/filter/SlewRateLimiter.h"


class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;


  //The default method to  catch notes from the ground    
  void CatchIntakeCommand(double rightCatchIntake_SpeedParamter,double leftCatchintake_SpeedParameter);

  //The Default method to spit notes from the intake
  void SpitIntakeCommand(double rightSpitIntake_SpeedParamter,double leftSpitIntake_SpeedParameter);

  void StopIntakeCommand();
 
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

 
  // //Creating the two Rev Intake motors wich we're gonna use at this subsystem
  // rev::CANSparkMax leftIntakeMotor{IntakeConstants::leftIntakeMotorId,rev::CANSparkLowLevel::MotorType::kBrushed};
  // rev::CANSparkMax rightIntakeMotor{IntakeConstants::rightIntakeMotorId,rev::CANSparkLowLevel::MotorType::kBrushed};

  //Creating the two CTRE(RED-MOTOR) and REV(NEO550-MOTOR) Intake motors wich we're gonna use  
  //at this subsystem
  rev::CANSparkMax leftIntakeMotor{IntakeConstants::leftIntakeMotorId,rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax rightIntakeMotor{IntakeConstants::rightIntakeMotorId, rev::CANSparkLowLevel::MotorType::kBrushless};

 
  //rihgt Intake filter   
  frc::SlewRateLimiter<units::scalar> rightIntake_Filter{0.08 / 20_ms};

  //left intake filter
  frc::SlewRateLimiter<units::scalar> LeftIntake_Filter{0.08 / 20_ms};


};
