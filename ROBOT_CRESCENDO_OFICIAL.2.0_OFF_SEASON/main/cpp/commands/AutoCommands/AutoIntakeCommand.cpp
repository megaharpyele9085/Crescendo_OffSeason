// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoCommands/AutoIntakeCommand.h"

AutoIntakeCommand::AutoIntakeCommand(IntakeSubsystem*intake_SubsystemParam, SwerveSubsystem*swerve_SubsystemParam): intake_Subsystem(intake_SubsystemParam),
swerve_Subsystem(swerve_SubsystemParam) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intake_SubsystemParam);
  AddRequirements(swerve_SubsystemParam);
}

// Called when the command is initially scheduled.
void AutoIntakeCommand::Initialize() {
  TimeToCatch.Start();
}

// Called repeatedly when this Command is scheduled to run
void AutoIntakeCommand::Execute() 
{
  double rot  = HeadingPID.Calculate(swerve_Subsystem->GetHeading(), 0)*2;
  if(TimeToCatch.Get() < 0.3_s){
  intake_Subsystem->CatchIntakeCommand(IntakeConstants::rightCatchIntakeSpeed,IntakeConstants::leftCatchIntakeSpeed);
  swerve_Subsystem->Drive(-0.5_mps, 0_mps, 0_rad_per_s, false);
}
else {
  intake_Subsystem->SpitIntakeCommand(-IntakeConstants::rightCatchIntakeSpeed, -IntakeConstants::leftCatchIntakeSpeed);
    swerve_Subsystem->Drive(0_mps, 0_mps, 0_rad_per_s, true);
    intake_Subsystem->StopIntakeCommand();
}
}

// Called once the command ends or is interrupted.
void AutoIntakeCommand::End(bool interrupted) 
{
  intake_Subsystem->StopIntakeCommand();
  swerve_Subsystem->StopModules();
}

// Returns true when the command should end.
bool AutoIntakeCommand::IsFinished() {
  if(TimeToCatch.Get() > 0.5_s){
    return true;
  }
  else{
    return false;
  }
}
