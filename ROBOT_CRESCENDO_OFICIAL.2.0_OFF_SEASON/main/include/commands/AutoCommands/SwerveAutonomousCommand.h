// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>

#include "subsystems/SwerveSubsystem.h"

#include <frc/trajectory/Trajectory.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SwerveAutonomousCommand
    : public frc2::CommandHelper<frc2::Command, SwerveAutonomousCommand> {
 public:
  SwerveAutonomousCommand();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  
};

namespace TrajectoryAuto { 

  frc2::CommandPtr TrajectoryAutoCommandFactory(SwerveSubsystem *const drive,
    frc::Pose2d const& start,
    std::vector<frc::Translation2d> const& waypoints,
    frc::Pose2d const& end);

}


