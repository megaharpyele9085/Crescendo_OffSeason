// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoCommands/AutoFindFronTargetCommand.h"

AutoFindFronTargetCommand::AutoFindFronTargetCommand(SwerveSubsystem*swerve_SubsystemParam, LimeLightFrontSubsystem*limeLightFront_SubsystemParam, bool IsinvertedRotParam):
swerve_Subsystem(swerve_SubsystemParam),
limeLightFront_Subsystem(limeLightFront_SubsystemParam)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(limeLightFront_SubsystemParam);
  AddRequirements(swerve_SubsystemParam);
  IsinvertedRot = IsinvertedRotParam;
}

// Called when the command is initially scheduled.
void AutoFindFronTargetCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoFindFronTargetCommand::Execute() {
  if(limeLightFront_Subsystem->HasTargetFront() == false){
    IsinvertedRot ? swerve_Subsystem->Drive(0_mps, 0_mps, -2_rad_per_s, true) : swerve_Subsystem->Drive(0_mps, 0_mps, 2_rad_per_s, true);
  }
  else{
    swerve_Subsystem->Drive(0_mps, 0_mps, 0_rad_per_s, true);
  }
}

// Called once the command ends or is interrupted.
void AutoFindFronTargetCommand::End(bool interrupted) {
  swerve_Subsystem->StopModules();
}

// Returns true when the command should end.
bool AutoFindFronTargetCommand::IsFinished() {
  if(limeLightFront_Subsystem->HasTargetFront() == true){
    return true;
  }
  else{
    return false;
  }
}
