// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoCommands/LimeNoteAutoCommand.h"

LimeNoteAutoCommand::LimeNoteAutoCommand(SwerveSubsystem*swerve_SubsystemParam, LimeLightFrontSubsystem*limeFront_SubsystemParam): swerve_Subsystem(swerve_SubsystemParam), limeFront_Subsystem(limeFront_SubsystemParam) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(limeFront_SubsystemParam);
  AddRequirements(swerve_SubsystemParam);
}

// Called when the command is initially scheduled.
void LimeNoteAutoCommand::Initialize() {
  // DriveRotPid.EnableContinuousInput(-24,24);
  // DriveYPid.EnableContinuousInput(-15, 5);
  DriveRotPid.Reset();
  DriveYPid.Reset();
  // HeadingPID.Reset();
}

// Called repeatedly when this Command is scheduled to run
void LimeNoteAutoCommand::Execute() {
  double velo = DriveYPid.Calculate(limeFront_Subsystem->GetTyFront(), -17)*3;
  double veloX = DriveRotPid.Calculate(limeFront_Subsystem->GetTxFront(), 0)*3;
  double rot = HeadingPID.Calculate(swerve_Subsystem->GetHeading(), 0)*2;
  if(limeFront_Subsystem->HasTargetFront() == true){
  swerve_Subsystem->Drive(static_cast<units::meters_per_second_t>(velo), 0_mps, static_cast<units::radians_per_second_t>(veloX), false);
  }
  else{
    swerve_Subsystem->Drive(0_mps, 0_mps, 0_rad_per_s, false);
  }
  
}

// Called once the command ends or is interrupted.
void LimeNoteAutoCommand::End(bool interrupted) {
  swerve_Subsystem->StopModules();
}

// Returns true when the command should end.
bool LimeNoteAutoCommand::IsFinished() {
  // if(limeFront_Subsystem->GetTyFront() < -14){
  //   if(limeFront_Subsystem->GetTxFront() > -1 && limeFront_Subsystem->GetTxFront() < 1){
  //   return true;
  //   }
  //   else{
  //     return false;
  //   }
  // }
  // else{
  // return false;
  // }
  if(limeFront_Subsystem->HasTargetFront() == false){
    return true;
  }
  else{
    return false;
  }
}
