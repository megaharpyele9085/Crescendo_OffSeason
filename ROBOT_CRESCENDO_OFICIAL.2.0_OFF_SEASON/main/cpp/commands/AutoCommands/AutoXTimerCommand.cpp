// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoCommands/AutoXTimerCommand.h"

AutoXTimerCommand::AutoXTimerCommand(units::second_t secondsParam, SwerveSubsystem*swerve_SubsystemParam,  double veloParam, bool IsXInvertedParam):
swerve_Subsystem(swerve_SubsystemParam)
 {
  // Use addRequirements() here to declare subsystem dependencies.
  secondsX = secondsParam;
  xVelo = veloParam;
  IsXInverted = IsXInvertedParam;
  AddRequirements(swerve_SubsystemParam);
}

// Called when the command is initially scheduled.
void AutoXTimerCommand::Initialize() {
  TimeToGoX.Start();
}

// Called repeatedly when this Command is scheduled to run
void AutoXTimerCommand::Execute() {
  double rot = HeadingPID.Calculate(swerve_Subsystem->GetHeading(), 0)*2;
  if(TimeToGoX.Get() < secondsX){
  if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){ 
  IsXInverted ? swerve_Subsystem->Drive(0_mps, static_cast<units::meters_per_second_t>(xVelo), static_cast<units::radians_per_second_t>(rot), true) : 
  swerve_Subsystem->Drive(0_mps, static_cast<units::meters_per_second_t>(-xVelo), static_cast<units::radians_per_second_t>(rot), true);
  }
  else{
     IsXInverted ? swerve_Subsystem->Drive(0_mps, static_cast<units::meters_per_second_t>(-xVelo), static_cast<units::radians_per_second_t>(rot), true) : 
     swerve_Subsystem->Drive(0_mps, static_cast<units::meters_per_second_t>(xVelo), static_cast<units::radians_per_second_t>(rot), true);
  }
  }
  else{
    swerve_Subsystem->Drive(0_mps, 0_mps, 0_rad_per_s, true);
  }
}

// Called once the command ends or is interrupted.
void AutoXTimerCommand::End(bool interrupted) {
  swerve_Subsystem->StopModules();
}

// Returns true when the command should end.
bool AutoXTimerCommand::IsFinished() {
  if(TimeToGoX.Get() > secondsX){
    return true;
  }
  else{
  return false;
  }
}
