// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/TeleopCommands/SwerveCommand.h"

#include "functional"
#include "algorithm"

#include <frc2/command/InstantCommand.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include <frc2/command/SwerveControllerCommand.h>

RobotContainer::RobotContainer()
{

  swerve_Subsystem.SetDefaultCommand(std::move(swerve_Command));

  // Just Descomment this Command when the Giraffe angle defined and working.
  // giraffe_Subsystem.SetDefaultCommand(std::move(autoSpeakerAngleCommand));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{

  //--------------------------------SHOOTER------------------------------------------------------

  m_TopController.RightTrigger().OnTrue(frc2::InstantCommand([&]() -> void
                                                             { shooter_Subsystem.CathShooterCommand(CatchShooterSpeedSpeaker); },
                                                             {&shooter_Subsystem})
                                            .ToPtr());

  m_TopController.RightTrigger().OnFalse(frc2::InstantCommand([&]() -> void
                                                              { shooter_Subsystem.StopShooterCommand(); },
                                                              {&shooter_Subsystem})
                                             .ToPtr());

  m_TopController.LeftTrigger().OnTrue(frc2::InstantCommand([&]() -> void
                                                            { shooter_Subsystem.LaunchShooterCommand(LaunchShooterSpeedSpeaker); },
                                                            {&shooter_Subsystem})
                                           .ToPtr());

  m_TopController.LeftTrigger().OnFalse(frc2::InstantCommand([&]() -> void
                                                             { shooter_Subsystem.StopShooterCommand(); },
                                                             {&shooter_Subsystem})
                                            .ToPtr());
  //-------------------------------DEFINING_AMP SHOOTER POTS------------------------------------------
  m_TopController.A().OnTrue(TestCommand(top_Subsystem
  ).ToPtr());

  m_TopController.A().OnFalse(frc2::InstantCommand([&]() -> void
                                                   { shooter_Subsystem.StopShooterCommand(); },
                                                   {&shooter_Subsystem})
                                  .ToPtr());
  //-------------------------------------INTAKE--------------------------------------------------------------

  m_TopController.RightBumper().OnTrue(frc2::InstantCommand([&]() -> void
                                                            { intake_Subsystem.CatchIntakeCommand(rightCatchIntakeSpeed, leftCatchIntakeSpeed); },
                                                            {&intake_Subsystem})
                                           .ToPtr());

  m_TopController.RightBumper().OnFalse(frc2::InstantCommand([&]() -> void
                                                             { intake_Subsystem.StopIntakeCommand(); },
                                                             {&intake_Subsystem})
                                            .ToPtr());

  m_TopController.LeftBumper().OnTrue(frc2::InstantCommand([&]() -> void
                                                           {intake_Subsystem.SpitIntakeCommand(rightSpitIntakeSpeed, leftSpitIntakeSpeed); },
                                                           {&intake_Subsystem})
                                          .ToPtr());

  m_TopController.LeftBumper().OnFalse(frc2::InstantCommand([&]() -> void
                                                            { intake_Subsystem.StopIntakeCommand(); },
                                                            {&intake_Subsystem})
                                           .ToPtr());

  
  //--------------------------------ELEVATOR_PID/TESTE-------------------------------------------------------------

  //**********PID_ELEVATOR***************//

  // OBS all methods of elevator subsystem is alredy inverted on ElevatorSubsystem.cpp
  //  m_TopController.Start().OnTrue(frc2::InstantCommand([&]()-> void
  //                                                {elevator_Subsystem.UpArmToChain(potUpElevator);},
  //                                                {&elevator_Subsystem}
  //                                                )
  //                                      .ToPtr());

  // Take care with this OnFalse Method Call.Just use after the Up to Chain Pid method work nicely,
  // and you discomment this OneFalse

  // m_TopController.Start().OnFalse(frc2::InstantCommand([&]()-> void
  //                                               {elevator_Subsystem.StopElevator();},
  //                                               {&elevator_Subsystem}
  //                                               )
  //                                     .ToPtr());

  // m_TopController.Back().OnTrue(frc2::InstantCommand([&]()-> void
  //                                               {elevator_Subsystem.DownArmToRobot(potDownElevator);},
  //                                               {&elevator_Subsystem}
  //                                               )
  //                                     .ToPtr());

  // m_TopController.Back().OnFalse(frc2::InstantCommand([&]()-> void
  //                                               {elevator_Subsystem.StopElevator();},
  //                                               {&elevator_Subsystem}
  //                                               )
  //                                     .ToPtr());

  //-------------------------------Amp Velocity Button--------------------------------------------------------------
//  m_TopController.A().OnTrue(frc2::InstantCommand([&]() -> void
//                                                             {shooter_Subsystem.LaunchAmpVelocity();},
//                                                             {&shooter_Subsystem})
//                                            .ToPtr());

//  m_TopController.A().OnFalse(frc2::InstantCommand([&]() -> void
//                                                             {shooter_Subsystem.StopShooterCommand();},
//                                                             {&shooter_Subsystem})
//                                            .ToPtr());                                         

}

void RobotContainer::AutonomousInit()
{
  swerve_Command.Cancel();
  
  swerve_Subsystem.ResetEncoders();
  swerve_Subsystem.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d{0_deg}));
  
  swerve_Subsystem.ZeroHeading();
}

void RobotContainer::TeleopInit()
{

  swerve_Command.Schedule();
  swerve_Subsystem.ResetEncoders();
  swerve_Subsystem.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d{0_deg}));

  elevator_Subsystem.ResetElevatorEncoders();
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand()
{

  return pathplanner::PathPlannerAuto("Center Blue").ToPtr();

  // switch (autoValue)
  // {

  // case 0:

  //    return frc2::SequentialCommandGroup(
  //    autoShooter_Command,
  //    limeAuto_Command,
  //    autoIntake_Command,
  //   //  Lime_ShortBackCommand,
  //    autoShooter_Command
  //   //  timer_AutoCommandNonInverted
  //    ).ToPtr();
  //   // frc2::SequentialCommandGroup(
  //   // autoSpeakerAngleCommand,
  //   // autoShooter_Command,
  //   // /*trajetoria para frente aqui*/,
  //   // limeAuto_Command,
  //   // frc2::ParallelCommandGroup(/*trajetoria curta aqui*/, autoIntake_Command),
  //   // /*trajetoria para trás aqui*/,
  //   // autoSpeakerAngleCommand,
  //   // autoShooter_Command,
  //   // /*trajetoria para frente aqui*/,
  //   // limeAuto_Command,
  //   // frc2::ParallelCommandGroup(/*trajetoria curta aqui*/, autoIntake_Command),
  //   // /*trajetoria para trás aqui*/,
  //   // autoSpeakerAngleCommand,
  //   // autoShooter_Command,
  //   // limeAuto_Command,
  //   // frc2::ParallelCommandGroup(/*trajetoria curta aqui*/, autoIntake_Command),
  //   // autoSpeakerAngleCommand,
  //   // autoShooter_Command,
  //   // /*trajetoria para o lado aqui*/,
  //   // limeAuto_Command,
  //   // frc2::ParallelCommandGroup(/*trajetoria curta aqui*/, autoIntake_Command),
  //   // autoSpeakerAngleCommand,
  //   // autoShooter_Command,
  //   // /*trajetoria para o lado aqui*/,
  //   // limeAuto_Command,
  //   // frc2::ParallelCommandGroup(/*trajetoria curta aqui*/, autoIntake_Command),
  //   // autoSpeakerAngleCommand,
  //   // autoShooter_Command
  //   // ).ToPtr();
  //   break;
  // case 1:
  //   return frc2::SequentialCommandGroup(
  //              autoShooter_Command,
  //              timer_AutoCommandNonInverted,
  //              frc2::WaitCommand(0.5_s),
  //              autoIntake_Command,
  //              timer_AutoCommandInverted,
  //              autoShooter_Command
  //              )
  //       .ToPtr();
  //   break;
  // // case 2:
  // //   return frc2::SequentialCommandGroup(
  // //              autoShooter_Command,
  // //              limeAuto_Command,
  // //              frc2::ParallelCommandGroup(timer_AutoCommandNonInverted, autoIntake_Command),
  // //              autoShooter_Command)
  // //       .ToPtr();
  // //   break;
  // // case 3:
  // //   return frc2::SequentialCommandGroup(
  // //              autoShooter_Command,
  // //              Dist_LongForwardCommand,
  // //              limeAuto_Command,
  // //              frc2::ParallelCommandGroup(Dist_ShortCommand, autoIntake_Command),
  // //              Dist_LongBackwardCommand,
  // //              autoShooter_Command,
  // //              limeAuto_Command,
  // //              frc2::ParallelCommandGroup(Dist_ShortCommand, autoIntake_Command),
  // //              autoShooter_Command)
  // //       .ToPtr();
  // //   break;
  // // case 4:
  // //   return frc2::SequentialCommandGroup(
  // //              autoShooter_Command,
  // //              Dist_LongForwardCommand,
  // //              limeAuto_Command,
  // //              frc2::ParallelCommandGroup(Dist_ShortCommand, autoIntake_Command),
  // //              Dist_LongBackwardCommand,
  // //              autoShooter_Command,
  // //              Dist_LongForwardCommand,
  // //              Dist_ShortRightSideCommand,
  // //              limeAuto_Command,
  // //              frc2::ParallelCommandGroup(Dist_ShortCommand, autoIntake_Command),
  // //              Dist_ShortLeftSideCommand,
  // //              Dist_LongBackwardCommand,
  // //              autoShooter_Command,
  // //              limeAuto_Command,
  // //              frc2::ParallelCommandGroup(Dist_ShortCommand, autoIntake_Command),
  // //              autoShooter_Command)
  // //       .ToPtr();
  // //   break;
  // // }

  // // return autoSelect.GetSelected();
  // case 2:
  //    return frc2::SequentialCommandGroup(
  //    autoShooter_Command,
  //    limeAuto_Command,
  //    autoIntake_Command,
  //    Lime_ShortBackCommand,
  //    autoShooter_Command,
  //    timer_AutoXCommandNonInvertedThreeNotes,
  //    timer_AutoCommandNonInvertedThreeNotes, 
  //    limeAuto_Command,
  //    autoIntake_Command,
  //    timer_AutoCommandInvertedThreeNotes,
  //    timer_AutoXCommandInvertedThreeNotes,
  //    Lime_ShortBackCommand,
  //    autoShooter_Command
  //    ).ToPtr();
  //    break;
  //    case 3:
  //    return frc2::SequentialCommandGroup(autoFindBackNonInverted_Command,
  //    autoShooter_Command,
  //    autoFindFrontNonInverted_Command,
  //    limeAuto_Command,
  //    autoIntake_Command,
  //    Lime_ShortSideackCommand,
  //    autoShooter_Command
  //    ).ToPtr();
  //    break;
  //    case 4: 
  //    return frc2::SequentialCommandGroup(autoFindBackInverted_Command,
  //    autoShooter_Command,
  //    autoFindFrontInverted_Command,
  //    limeAuto_Command,
  //    autoIntake_Command,
  //    Lime_ShortSideackCommand,
  //    autoShooter_Command
  //    ).ToPtr();
  //    break;
  //    case 5:
  //    return frc2::SequentialCommandGroup(autoFindBackNonInverted_Command,
  //    autoShooter_Command,
  //    autoFindFrontNonInverted_Command,
  //    limeAuto_Command,
  //    autoIntake_Command,
  //    Lime_ShortSideackCommand,
  //    autoShooter_Command,
  //    timer_AutoXCommandInvertedThreeNotes,
  //    timer_AutoCommandNonInvertedThreeNotes,
  //    limeAuto_Command,
  //    autoIntake_Command,
  //    timer_AutoCommandInvertedThreeNotes,
  //    timer_AutoXCommandNonInvertedThreeNotes,
  //    Lime_ShortSideackCommand,
  //    autoShooter_Command
  //    ).ToPtr();
  //    break;
  //    case 6:
  //    return frc2::SequentialCommandGroup(autoFindBackInverted_Command,
  //    autoShooter_Command,
  //    autoFindFrontInverted_Command,
  //    limeAuto_Command,
  //    autoIntake_Command,
  //    Lime_ShortSideackCommand,
  //    autoShooter_Command,
  //    timer_AutoXShortCommandNonInvertedThreeNotes,
  //    timer_AutoCommandNonInvertedThreeNotes,
  //    limeAuto_Command,
  //    autoIntake_Command,
  //    timer_AutoCommandNonInvertedThreeNotes,
  //    timer_AutoXCommandInvertedThreeNotes,
  //    Lime_ShortSideackCommand,
  //    autoShooter_Command).ToPtr();
  //    break;
  //    case 7:
  //    return frc2::SequentialCommandGroup(
  //     autoShooter_Command
  //    ).ToPtr();
  //    break;
//  }
}
void RobotContainer::SelectAuto()
{
  frc::SmartDashboard::SetDefaultNumber("Autonomous select", autoValue);
  autoValue = frc::SmartDashboard::GetNumber("Autonomous select", autoValue);

  // Elevator Up Speeds
  frc::SmartDashboard::SetDefaultNumber("UpElevatorPot", potUpElevator);
  potUpElevator = frc::SmartDashboard::GetNumber("UpElevatorPot", potUpElevator);

  // Elevator Down Speeds
  frc::SmartDashboard::SetDefaultNumber("DownElevatorPot", potDownElevator);
  potDownElevator = frc::SmartDashboard::GetNumber("DownElevatorPot", potDownElevator);
}
