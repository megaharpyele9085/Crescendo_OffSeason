// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TopSubsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem()
{

//-----------------------------Default Falcon Configs-----------------------------------------------

//The Left and Right motor config object to apply at the motors 6
ctre::phoenix6::configs::TalonFXConfiguration leftShooterMotorConfiguration{};
ctre::phoenix6::configs::TalonFXConfiguration rightShooterMotorConfiguration{};


rightShooterMotorConfiguration.MotorOutput.Inverted = BooleansConsts::RightShooterMotorReversed;
leftShooterMotorConfiguration.MotorOutput.Inverted = BooleansConsts::LeftShooterMotorReversed;




//-----------------------------MOTION_MAGIC_CONFIGS-------------------------------------------

ctre::phoenix6::configs::MotionMagicConfigs &mm_Left = leftShooterMotorConfiguration.MotionMagic;
ctre::phoenix6::configs::MotionMagicConfigs &mm_Right = rightShooterMotorConfiguration.MotionMagic;

mm_Left.MotionMagicCruiseVelocity = 90;
mm_Right.MotionMagicCruiseVelocity = 90;
mm_Left.MotionMagicAcceleration = 250;
mm_Right.MotionMagicAcceleration = 250;
mm_Left.MotionMagicJerk = 200;
mm_Right.MotionMagicJerk = 200;


 ctre::phoenix6::configs::Slot0Configs &slot0_left = leftShooterMotorConfiguration.Slot0;
 ctre::phoenix6::configs::Slot0Configs &slot0_right = rightShooterMotorConfiguration.Slot0;



  slot0_left.kP = 0.25;
  slot0_left.kI = 0.0001;
  slot0_left.kD = 0.00001;
  slot0_left.kV = 0.12;
  slot0_left.kS = 0.12; // Approximately 0.25V to get the mechanism moving

  slot0_right.kP = 0.25;
  slot0_right.kI = 0.0001;
  slot0_right.kD = 0.00001;
  slot0_right.kV = 0.12;
  slot0_right.kS = 0.12; // Approximately 0.25V to get the mechanism moving

//   ctre::phoenix6::configs::FeedbackConfigs &fdb_left = leftShooterMotorConfiguration.Feedback;
//   ctre::phoenix6::configs::FeedbackConfigs &fdb_right = rightShooterMotorConfiguration.Feedback;

//   fdb_left.SensorToMechanismRatio = 1.0;
//   fdb_right.SensorToMechanismRatio = 1.0;

rightShooterMotor.GetConfigurator().Apply(rightShooterMotorConfiguration);
leftShooterMotor.GetConfigurator().Apply(leftShooterMotorConfiguration);

}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() 
{

frc::SmartDashboard::PutNumber("LeftShooterVelocity",GetLeftShooterVelocity());
frc::SmartDashboard::PutNumber("RightShooterVelocity",GetRightShooterVelocity());

//Temps and currents
frc::SmartDashboard::PutNumber("LeftFALCON Temp (C deg)",(leftShooterMotor.GetDeviceTemp().GetValueAsDouble()));
frc::SmartDashboard::PutNumber("LeftFALCON Current (A)",(leftShooterMotor.GetStatorCurrent().GetValueAsDouble()));

frc::SmartDashboard::PutNumber("RightFALCON Temp (C deg)",(rightShooterMotor.GetDeviceTemp().GetValueAsDouble()));
frc::SmartDashboard::PutNumber("RightFALCON Current (A)",(rightShooterMotor.GetStatorCurrent().GetValueAsDouble()));

}

void ShooterSubsystem::LaunchShooterCommand(double launchShooter_SpeedParameter)
{
  // RightSpeed = launchShooter_SpeedParameter;
  // LeftSpeed = launchShooter_SpeedParameter;

  //Motion Magic setControl Method
  rightShooterMotor.SetControl(m_mmReqRight.WithVelocity(static_cast<units::turns_per_second_t>(launchShooter_SpeedParameter)));
  leftShooterMotor.SetControl(m_mmReqLeft.WithVelocity(static_cast<units::turns_per_second_t>(launchShooter_SpeedParameter)));
  // rightShooterMotor.Set(rightShooter_Filter.Calculate(launchShooter_SpeedParameter));
  // leftShooterMotor.Set(LeftShooter_Filter.Calculate(launchShooter_SpeedParameter));
  // rightShooterMotor.SetControl(RightSpeedDutyCycle);
  // leftShooterMotor.SetControl(LeftSpeedDutyCycle);
}

void ShooterSubsystem::CathShooterCommand(double CatchShooter_SpeedParameter)
{

  // RightSpeed = CatchShooter_SpeedParameter;
  // LeftSpeed = CatchShooter_SpeedParameter;

  //Motion Magic setControl Method
  // rightShooterMotor.SetControl(m_mmReqRight.WithPosition(50_tr).WithSlot(0));
  // leftShooterMotor.SetControl(m_mmReqLeft.WithPosition(50_tr).WithSlot(0));


  rightShooterMotor.Set(rightShooter_Filter.Calculate(-CatchShooter_SpeedParameter));
  leftShooterMotor.Set(LeftShooter_Filter.Calculate(-CatchShooter_SpeedParameter));

  // rightShooterMotor.SetControl(RightSpeedDutyCycle);
  // leftShooterMotor.SetControl(LeftSpeedDutyCycle);

}

void ShooterSubsystem::LaunchAmpVelocity()
{

  rightShooterMotor.SetControl(m_mmReqRight.WithVelocity(static_cast<units::turns_per_second_t>(-ShooterConstants::LaunchAmp_RightVelocity)));
  leftShooterMotor.SetControl(m_mmReqLeft.WithVelocity(static_cast<units::turns_per_second_t>(-ShooterConstants::LaunghAmp_LeftVelocity)));

}

double ShooterSubsystem::GetRightShooterVelocity()
{

 return rightShooterMotor.GetVelocity().GetValueAsDouble();

}

double ShooterSubsystem::GetLeftShooterVelocity()
{

 return leftShooterMotor.GetVelocity().GetValueAsDouble();

}


void ShooterSubsystem::StopShooterCommand()
{
  leftShooterMotor.StopMotor();
  rightShooterMotor.StopMotor();
  
}

