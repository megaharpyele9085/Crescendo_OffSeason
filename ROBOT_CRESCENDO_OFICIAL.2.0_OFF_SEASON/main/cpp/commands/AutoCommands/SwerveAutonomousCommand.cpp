// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoCommands/SwerveAutonomousCommand.h"
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

SwerveAutonomousCommand::SwerveAutonomousCommand() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SwerveAutonomousCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SwerveAutonomousCommand::Execute() {}

// Called once the command ends or is interrupted.
void SwerveAutonomousCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool SwerveAutonomousCommand::IsFinished() {
  return false;
}

frc2::CommandPtr TrajectoryAuto::TrajectoryAutoCommandFactory(SwerveSubsystem *const drive,
    frc::Pose2d const& start,
    std::vector<frc::Translation2d> const& waypoints,
    frc::Pose2d const& end){

   
   frc::ProfiledPIDController<units::radians> thetaController {
   drive->kPThetaController, 0,0,
   AutoConstants::kThetaControllerConstraints};

   thetaController.EnableContinuousInput(units::degree_t(-180.0), units::degree_t(180.0));

  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
     //Construct command to follow trajectory
     
      frc2::SwerveControllerCommand<4> swerveControllerCommand{
      frc::TrajectoryGenerator::GenerateTrajectory(start, 
      waypoints, 
      end,
      config),
      [drive]() ->frc::Pose2d {return drive->GetPose2d();},

      DriveConstants::kDriveKinematics,

      frc::PIDController{drive->KPXaxisController,0,0},
      frc::PIDController{drive->KPYaxisController,0,0}, 
      thetaController,

        // [drive]() -> frc::Rotation2d "desiredRotation"
        [drive](std::array<frc::SwerveModuleState, 4> states) -> void
        {drive->SetModulesState(states); },
        {drive}};


       return std::move(swerveControllerCommand).ToPtr();

}
