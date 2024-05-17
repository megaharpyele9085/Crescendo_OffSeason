// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopCommands/TestCommand.h"

TestCommand::TestCommand(TopSubsystemGroup& top_SubsystemParam): top_Subsystem{&top_SubsystemParam} {
  // Use addRequirements() here to declare subsystem dependencies.
  // AddRequirements(&giraffe_SubsystemParam);
}

// Called when the command is initially scheduled.
void TestCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TestCommand::Execute() {
  top_Subsystem->ToShoot();
}

// Called once the command ends or is interrupted.
void TestCommand::End(bool interrupted) {
  top_Subsystem->ToStop();
}

// Returns true when the command should end.
bool TestCommand::IsFinished() {
  return top_Subsystem->ToFinish();
}
