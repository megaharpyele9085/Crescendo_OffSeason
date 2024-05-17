// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoCommands/AutoYTimerCommand.h"

AutoYTimerCommand::AutoYTimerCommand(units::second_t SecondsParam, SwerveSubsystem*swerve_SubsystemParam, bool isInverted):
swerve_Subsystem(swerve_SubsystemParam)
{
  // Use addRequirements() here to declare subsystem dependencies.
  secondsY = SecondsParam;
  isYInverted = isInverted;
  AddRequirements(swerve_SubsystemParam);
}

// Called when the command is initially scheduled.
void AutoYTimerCommand::Initialize() {
  TimeToGoY.Start();
}

// Called repeatedly when this Command is scheduled to run
void AutoYTimerCommand::Execute() {
  double rot = HeadingPID.Calculate(swerve_Subsystem->GetHeading(), 0)*2;
  if(TimeToGoY.Get() < secondsY){
   isYInverted ? swerve_Subsystem->Drive(-0.5_mps, 0_mps, static_cast<units::radians_per_second_t>(rot), false) : swerve_Subsystem->Drive(0.25_mps, 0_mps, static_cast<units::radians_per_second_t>(rot), false) ;
  }
  else{
    swerve_Subsystem->Drive(0_mps, 0_mps, 0_rad_per_s, false);
  }
}

// Called once the command ends or is interrupted.
void AutoYTimerCommand::End(bool interrupted) {
  swerve_Subsystem->StopModules();
}

// Returns true when the command should end.
bool AutoYTimerCommand::IsFinished() {
  if(TimeToGoY.Get() > secondsY){
    return true;
  }
  else{
  return false;
  }
}
