// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoCommands/LimeBackAutoCommand.h"

LimeBackAutoCommand::LimeBackAutoCommand(SwerveSubsystem*swerve_SubsystemParam, LimeLightBackSubsystem*limeLightBack_SubsystemParam, double TyValueParam, double TxValueParam):
swerve_Subsystem(swerve_SubsystemParam),
limeLightBack_Subsystem(limeLightBack_SubsystemParam)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerve_SubsystemParam);
  AddRequirements(limeLightBack_SubsystemParam);
  TyValue = TyValueParam;
  TxValue = TxValueParam; 
}

// Called when the command is initially scheduled.
void LimeBackAutoCommand::Initialize() {
  TimerToEnd.Start();
}

// Called repeatedly when this Command is scheduled to run
void LimeBackAutoCommand::Execute() {
  if(TxValue < 0.1 && TxValue > -0.1){
  SpeedXChassis = SpeedTxPID.Calculate(limeLightBack_Subsystem->GetTxBack(), TxValue)*2;
  }
  else{
    SpeedXChassis = 0;
  }
  if(TyValue < 0.1 && TyValue > -0.1){
  SpeedYChassis = SpeedTyPID.Calculate(limeLightBack_Subsystem->GetTyBack(), TyValue)*2;
  }
  else{
    SpeedYChassis = 0;
  }
  swerve_Subsystem->Drive(static_cast<units::meters_per_second_t>(SpeedYChassis), static_cast<units::meters_per_second_t>(SpeedXChassis), 0_rad_per_s, true);
}

// Called once the command ends or is interrupted.
void LimeBackAutoCommand::End(bool interrupted) {
  TimerToEnd.Stop();
  TimerToEnd.Reset();
}

// Returns true when the command should end.
bool LimeBackAutoCommand::IsFinished() {
  if(TimerToEnd.Get() > 1_s){
  if(SpeedXChassis < 0.05 && SpeedXChassis > -0.05){
    if(SpeedYChassis < 0.05 && SpeedYChassis > -0.05){
      return true;
    }
  }
  else{
  return false;
  }
  }
  else{
    return false;
  }
}
