// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include<frc/Joystick.h>
#include "Constants.h"

#include "subsystems/SwerveSubsystem.h"

#include "subsystems/LimeLightFrontSubsystem.h"

#include "subsystems/LimeLightBackSubsystem.h"

#include "subsystems/ControlSubsystem.h"

#include "subsystems/TopSubsystems/ShooterSubsystem.h"

#include "subsystems/TopSubsystems/ElevatorSubsystem.h"

#include "subsystems/TopSubsystems/FlipSubsystem.h"

#include "subsystems/TopSubsystems/GiraffeSubsystem.h"

//Ultrasonic Subsystem
#include "subsystems/UltrasonicSubsystem.h"

#include "frc2/command/Command.h"

#include "frc2/command/button/JoystickButton.h"

#include <frc/XboxController.h>


//Auto Commands Library;
#include "commands/AutoCommands/SwerveAutonomousCommand.h"

//SwerveCommand Library
#include "commands/TeleopCommands/SwerveCommand.h"

//AutoShooterCommand
#include "commands/AutoCommands/AutoShooterCommand.h"

//AutoIntakeCommand
#include "commands/AutoCommands/AutoIntakeCommand.h"

//parallel command group
#include <frc2/command/ParallelCommandGroup.h>

#include <frc/smartdashboard/SendableChooser.h>

//include Lime AutonomousCommand
#include "commands/AutoCommands/LimeNoteAutoCommand.h"

// #include "commands/AutoCommands/AutoDistCommand.h"

#include "commands/AutoCommands/LimeBackAutoCommand.h"

#include "commands/AutoCommands/AutoYTimerCommand.h"

#include "commands/AutoCommands/AutoXTimerCommand.h"

#include "commands/AutoCommands/AutoFindBackTargetCommand.h"

#include "commands/AutoCommands/AutoFindFronTargetCommand.h"

#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <pathplanner/lib/auto/NamedCommands.h>

#include "commands/TeleopCommands/TestCommand.h"

#include "subsystems/TopSubsystemGroup.h"

//#include "commands/TeleopCommands/TeleopShooterCommand.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  std::optional<frc2::CommandPtr> GetAutonomousCommand();


    void AutonomousInit();
    void TeleopInit();

    void SelectAuto();

 private:

  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  // Xbox Top Controller
  frc2::CommandXboxController m_TopController{OperatorConstants::kTopControllerPort};

  // The robot's subsystems are defined here...

  //XboxSubsystem Controller
  ControlSubsystem Control;  

  //SwerveSubsystem Object  
  SwerveSubsystem swerve_Subsystem;

  TopSubsystemGroup top_Subsystem{&intake_Subsystem, &shooter_Subsystem, &giraffe_Subsystem, &limeLightBack_Subsystem};

  //LimeLight-Front Subsystem
  LimeLightFrontSubsystem limeLightFront_Subsystem;

  //LimeLight-Back Subsystem
  LimeLightBackSubsystem limeLightBack_Subsystem;

  //IntakeSubsystem
  IntakeSubsystem intake_Subsystem;

  //Shooter Subsystem
  ShooterSubsystem shooter_Subsystem;

  //Elevator Subsystem
  ElevatorSubsystem elevator_Subsystem;

  //Ultrasonic Subsystem
  //UltrasonicSubsystem ultrasonic_Subsystem;

  //Flip Subsystem
  //FlipSubsystem flip_Subsystem;

  //Giraffe Subsystem
  GiraffeSubsystem giraffe_Subsystem;  

  //Default Swerve Command
  SwerveCommand swerve_Command{
    &swerve_Subsystem,
    &Control,
    &intake_Subsystem,
    &shooter_Subsystem,
    //&flip_Subsystem,
    &giraffe_Subsystem,
    &elevator_Subsystem,
    //&ultrasonic_Subsystem,
    &limeLightFront_Subsystem,
    &limeLightBack_Subsystem
  };

  //TeleopShooterCommand Shooter_Command{&shooter_Subsystem, &intake_Subsystem};

  // AutoDistCommand Dist_ShortCommand{&swerve_Subsystem, 0.5, 0};
  // AutoDistCommand Dist_LongForwardCommand{&swerve_Subsystem, 2, 0};
  // AutoDistCommand Dist_LongBackwardCommand{&swerve_Subsystem, -2, 0};
  // AutoDistCommand Dist_ShortLeftSideCommand{&swerve_Subsystem, 0, -0.5};
  // AutoDistCommand Dist_ShortRightSideCommand{&swerve_Subsystem, 0, 0.5};

  LimeBackAutoCommand Lime_ShortBackCommand{&swerve_Subsystem, &limeLightBack_Subsystem, -1.81, -9.5};
    LimeBackAutoCommand Lime_ShortSideackCommand{&swerve_Subsystem, &limeLightBack_Subsystem, -3.81, 0};
  
  AutoYTimerCommand timer_AutoCommandNonInverted{0.2_s, &swerve_Subsystem, false};
  AutoYTimerCommand timer_AutoCommandNonInvertedThreeNotes{1_s, &swerve_Subsystem, false};

  AutoYTimerCommand timer_AutoCommandInverted{0.45_s, &swerve_Subsystem, true};
  AutoYTimerCommand timer_AutoCommandInvertedThreeNotes{1.6_s, &swerve_Subsystem, true};

  AutoXTimerCommand timer_AutoXCommandNonInvertedThreeNotes{2_s, &swerve_Subsystem, -0.5, false};
  AutoXTimerCommand timer_AutoXShortCommandNonInvertedThreeNotes{0.7_s, &swerve_Subsystem, -0.5, false};
  AutoXTimerCommand timer_AutoXCommandInvertedThreeNotes{2_s, &swerve_Subsystem, -0.5, true};

  AutoShooterCommand autoShooter_Command{
  &shooter_Subsystem, 
  &limeLightBack_Subsystem, 
  &swerve_Subsystem,
  &giraffe_Subsystem, 
  &intake_Subsystem
  };

  AutoFindBackTargetCommand autoFindBackNonInverted_Command{&swerve_Subsystem, &limeLightBack_Subsystem, false};
  AutoFindBackTargetCommand autoFindBackInverted_Command{&swerve_Subsystem, &limeLightBack_Subsystem, true};

  AutoFindFronTargetCommand autoFindFrontNonInverted_Command{&swerve_Subsystem, &limeLightFront_Subsystem, false};
  AutoFindFronTargetCommand autoFindFrontInverted_Command{&swerve_Subsystem, &limeLightFront_Subsystem, true};

  AutoIntakeCommand autoIntake_Command{&intake_Subsystem, &swerve_Subsystem};

  LimeNoteAutoCommand limeAuto_Command{&swerve_Subsystem, &limeLightFront_Subsystem};

  //  frc::ProfiledPIDController<units::radians> thetaController {
  //  swerve_Subsystem.kPThetaController, 0,0,
  //  AutoConstants::kThetaControllerConstraints};


  //0 = 2 notes center with lime
  //1 = 2 notes center with timer
  //2 = 3 notes center with lime
  //3 = 2 notes left with lime
  //4 = 2 notes right with lime
  //5 = 3 notes right blue with lime
  //6 = 3 notes left with lime
  int autoValue = 2;

  //Shooter speaker speeds to apply in shuffle
  double LaunchShooterSpeedSpeaker = 90;
  double CatchShooterSpeedSpeaker = 0.15;

  //Shooter amp speeds
  double LaunchShooterAmp = 17;

  //Intake speeds to aplly in shuffle  
  double rightCatchIntakeSpeed  = -0.5; //
  double leftCatchIntakeSpeed = 0.25;  //

  double rightSpitIntakeSpeed  = 0.5; //
  double leftSpitIntakeSpeed  = -0.25; //

  //The default  Elevator Pots when Start or Back button is pressed
  double potUpElevator = 0.1;
  double potDownElevator = -0.1;

  void ConfigureBindings();
};
