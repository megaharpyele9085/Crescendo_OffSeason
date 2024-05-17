// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoCommands/AutoFindBackTargetCommand.h"

AutoFindBackTargetCommand::AutoFindBackTargetCommand(SwerveSubsystem*swerve_SubsystemParam, LimeLightBackSubsystem*limeLightBack_SubsystemParam, bool IsRotInvertedParam):
swerve_Subsystem(swerve_SubsystemParam),
limeLightBack_Subsystem(limeLightBack_SubsystemParam)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerve_SubsystemParam);
  AddRequirements(limeLightBack_SubsystemParam);
  IsRotInverted = IsRotInvertedParam;
}

// Called when the command is initially scheduled.
void AutoFindBackTargetCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoFindBackTargetCommand::Execute() {
  if(limeLightBack_Subsystem->HasBackTarget() == false){
   IsRotInverted ? swerve_Subsystem->Drive(0_mps, 0_mps, 2_rad_per_s, true) : swerve_Subsystem->Drive(0_mps, 0_mps, -2_rad_per_s, true);
  }
  else{
    swerve_Subsystem->Drive(0_mps, 0_mps, 0_rad_per_s, true);
  }
}

// Called once the command ends or is interrupted.
void AutoFindBackTargetCommand::End(bool interrupted) {
  swerve_Subsystem->StopModules();
}

// Returns true when the command should end.
bool AutoFindBackTargetCommand::IsFinished() {
  if(limeLightBack_Subsystem->HasBackTarget() == true){
    return true;
  }
  else{
    return false;
  }
}
