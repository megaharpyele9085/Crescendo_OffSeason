// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopCommands/SwerveCommand.h"
#include "Constants.h"

SwerveCommand::SwerveCommand(SwerveSubsystem *swerve_SubsystemParameter,
                             ControlSubsystem *Control_SubsystemParameter,
                             IntakeSubsystem *intake_SubsystemParameter,
                             ShooterSubsystem *shooter_SubsystemParameter,
                            // FlipSubsystem *flip_SubsystemParameter,
                             GiraffeSubsystem *giraffe_SubsystemParameter,
                             ElevatorSubsystem *elevator_SubsystemParameter,
                            // UltrasonicSubsystem *ultrasonic_SubsystemParameter,
                             LimeLightFrontSubsystem *limeLightFront_SubsystemParameter,
                             LimeLightBackSubsystem *limeLightBack_SubsystemParameter) : swerve_Subsystem{swerve_SubsystemParameter},
                                                                                         control{Control_SubsystemParameter},
                                                                                         intake_Subsystem{intake_SubsystemParameter},
                                                                                         shooter_Subsystem{shooter_SubsystemParameter},
                                                                                         //flip_Subsystem{flip_SubsystemParameter},
                                                                                         giraffe_Subsystem{giraffe_SubsystemParameter},
                                                                                         elevator_Subsystem{elevator_SubsystemParameter},
                                                                                         //ultrasonic_Subsystem{ultrasonic_SubsystemParameter},
                                                                                         limeLightFront_Subsystem{limeLightFront_SubsystemParameter},
                                                                                         limeLightBack_Subsystem{limeLightBack_SubsystemParameter}
{
  // Ranging the LimeLight Tx and distance Pids Controllers
  // LimeLightPid_Tx.EnableContinuousInput(-29.80, 29.80);
  HeadingPidRot.EnableContinuousInput(0, 360);

  // Adds the specified Subsystem requirement to the command. The scheduler will prevent
  //  two commands that require the same subsystem from being scheduled simultaneously.
  //  Note that the scheduler determines the requirements of a command when it is scheduled,
  //  so this method should normally be called from the command's constructor.

  AddRequirements(swerve_SubsystemParameter);

  //AddRequirements(flip_SubsystemParameter);

  AddRequirements(Control_SubsystemParameter);

  AddRequirements(intake_SubsystemParameter);

  AddRequirements(shooter_SubsystemParameter);

  AddRequirements(giraffe_SubsystemParameter);

  AddRequirements(elevator_SubsystemParameter);

  //AddRequirements(ultrasonic_SubsystemParameter);

  AddRequirements(limeLightFront_SubsystemParameter);

  AddRequirements(limeLightBack_SubsystemParameter);
}

// Called when the command is initially scheduled.
void SwerveCommand::Initialize()
{

  // Reset all the limes Pid's
  LimeLightPid_Tx.Reset();
  LimeLightPidDist_Y.Reset();
  giraffe_Subsystem->GiraffePid.Reset();
}

// Called repeatedly when this Command is scheduled to run
void SwerveCommand::Execute()
{

  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
  {
    AllianceBlue = true;
  }
  else
  {
    AllianceBlue = false;
  }

  //---------------------------ShuffleBoard Values-------------------------------------------
  // Getting the Dist in the Y axis in the ShuffleBoard;
  frc::SmartDashboard::PutNumber("BackLime_YDist", limeLightBack_Subsystem->GetDistBack_Y(ActualAprilTargetHigh));

  // Getting the Dist in the X axis in the ShuffleBoard;
  frc::SmartDashboard::PutNumber("BackLime_XDist", limeLightBack_Subsystem->GetDistBack_X());

  frc::SmartDashboard::PutNumber("RightElevatorMotor Position", elevator_Subsystem->GetRightElevatorPosition());

  frc::SmartDashboard::PutNumber("LeftElevatorMotor Position", elevator_Subsystem->GetLeftElevatorPosition());


  //-------------------------------LIME_PID_GAINS----------------------------------------------


  LimeLightPid_Tx.SetPID(LimeTxPid_P, LimeTxPid_I, LimeTxPid_D, 20.0, 0.39364_s, 0.01_s);
  //**************************************************************************

  // Lime DIST_YAxis-P_I_D in shuffleboard ************************************
  frc::SmartDashboard::SetDefaultNumber("limeDistYaxisPID_P", LimeDistYPid_P);           //*
  LimeDistYPid_P = frc::SmartDashboard::GetNumber("limeDistYaxisPID_P", LimeDistYPid_P); //*
                                                                                         //                                                                                   //*
  frc::SmartDashboard::SetDefaultNumber("limeDistYaxisPID_I", LimeDistYPid_I);
  LimeDistYPid_I = frc::SmartDashboard::GetNumber("limeDistYaxisPID_I", LimeDistYPid_I);

  frc::SmartDashboard::SetDefaultNumber("limeDistYaxisPID_D", LimeTxPid_D);
  LimeDistYPid_D = frc::SmartDashboard::GetNumber("limeDistYaxisPID_D", LimeTxPid_D);

  LimeLightPidDist_Y.SetPID(LimeDistYPid_P, LimeDistYPid_I, LimeDistYPid_D, 20.0, 0.39364_s, 0.01_s);
  //****************************************************************************

  // Lime Dist in X axis Pid in shuffleboard*************************************
  frc::SmartDashboard::SetDefaultNumber("limeDistXaxisPID_P", LimeDistXPid_P);
  LimeDistXPid_P = frc::SmartDashboard::GetNumber("limeDistXaxisPid_P", LimeDistXPid_P);

  frc::SmartDashboard::SetDefaultNumber("limeDistXaxisPID_I", LimeDistXPid_I);
  LimeDistXPid_I = frc::SmartDashboard::GetNumber("limeDistXaxisPID_I", LimeDistXPid_I);

  frc::SmartDashboard::SetDefaultNumber("limeDistXaxisPID_D", LimeDistXPid_D);
  LimeDistXPid_D = frc::SmartDashboard::GetNumber("limeDistXaxisPID_D", LimeDistXPid_D);

  LimeLightPidDist_X.SetPID(LimeDistXPid_P, LimeDistXPid_I, LimeDistXPid_D, 20.0, 0.39364_s, 0.01_s);
  //*****************************************************************************

  // Heading Rot Pid in shuffleboard***************************************************
  frc::SmartDashboard::SetDefaultNumber("HeadingRotPID_P", HeadingRotPid_P);
  HeadingRotPid_P = frc::SmartDashboard::GetNumber("HeadingRotPID_P", HeadingRotPid_P);

  frc::SmartDashboard::SetDefaultNumber("HeadingRotPID_I", HeadingRotPid_I);
  HeadingRotPid_I = frc::SmartDashboard::GetNumber("HeadingRotPID_I", HeadingRotPid_I);

  frc::SmartDashboard::SetDefaultNumber("HeadingRotPID_D", HeadingRotPid_D);
  HeadingRotPid_D = frc::SmartDashboard::GetNumber("HeadingRotPID_D", HeadingRotPid_D);

  HeadingPidRot.SetPID(HeadingRotPid_P, HeadingRotPid_I, HeadingRotPid_D, 20.0, 0.39364_s, 0.01_s);

 // frc::SmartDashboard::PutNumber("ERROR GIRAFE PID", giraffe_Subsystem->GetErrorPID());
  //  frc::SmartDashboard::PutNumber("Calculate Giraffe return", giraffe_Subsystem->GetCalculate());

  frc::SmartDashboard::PutBoolean("Alliance is Blue?", AllianceBlue);

  //-------------------------------SHUFFLE_SETPOINTS----------------------------------------------*

  //*********Source SetPoints*********

  // //Source Robot Angle
  // frc::SmartDashboard::SetDefaultNumber("Source_RobotAngle_SetPoint",SourceRobotAngle);
  // SourceRobotAngle = frc::SmartDashboard::GetNumber("Source_RobotAngle_SetPoint",SourceRobotAngle);

  // //Source BackTx
  // frc::SmartDashboard::SetDefaultNumber("Source_BackTx_SetPoint",BackTxSource);
  // BackTxSource = frc::SmartDashboard::GetNumber("Source_BackTx_SetPoint",BackTxSource);

  // //Source Giraffe angle
  // frc::SmartDashboard::SetDefaultNumber("Source_GiraffeAngle_SetPoint",Giraffe_SourceAngle);
  // Giraffe_SourceAngle = frc::SmartDashboard::GetNumber("Source_GiraffeAngle_SetPoint",Giraffe_SourceAngle);

  // frc::SmartDashboard::SetDefaultNumber("Amp_GiraffeAngle_SetPoint",Giraffe_AmpAngle);
  // Giraffe_AmpAngle = frc::SmartDashboard::GetNumber("Amp_GiraffeAngle_SetPoint",Giraffe_AmpAngle);

  //********Amp SetPoints**************

  // //Amp Robot Angle
  // frc::SmartDashboard::SetDefaultNumber("Amp_RobotAngle_SetPoint",AmpRobotAngle);
  // AmpRobotAngle = frc::SmartDashboard::GetNumber("Amp_RobotAnlge_SetPoint",AmpRobotAngle);

  // //Amp BackTx
  // frc::SmartDashboard::SetDefaultNumber("Amp_BackTx_SetPoint",BackTxAmp);
  // BackTxAmp = frc::SmartDashboard::GetNumber("Amp_BackTx_SetPoint",BackTxAmp);

  // //Amp Giraffe angle
  // frc::SmartDashboard::SetDefaultNumber("Amp_GiraffeAngle_SetPoint",Giraffe_AmpAngle);
  // Giraffe_AmpAngle = frc::SmartDashboard::GetNumber("Amp_GiraffeAngle_SetPoint",Giraffe_AmpAngle);

  //*********Chain SetPoints*************

  // //Horizontal setPoints_TESTEE(tx)

  // // -1
  // frc::SmartDashboard::SetDefaultNumber("BackTxChain_minus_one",BackTxChain_minus_one);
  // BackTxChain_minus_one = frc::SmartDashboard::GetNumber("BackTxChain_minus_one",BackTxChain_minus_one);

  // //  0
  // frc::SmartDashboard::SetDefaultNumber("BackTxChain_zero",BackTxChain_zero);
  // BackTxChain_zero = frc::SmartDashboard::GetNumber("BackTxChain_zero",BackTxChain_zero);

  // //  1
  // frc::SmartDashboard::SetDefaultNumber("BackTxChain_one",BackTxChain_one);
  // BackTxChain_one = frc::SmartDashboard::GetNumber("BackTxChain_one",BackTxChain_one);

  //*********Giraffe SetPoints*************

  //--------------------------------DEFINING_BUTTONS----------------------------------------------------------------
  // Defining the driver buttons and analogics

  // Default joysticks and buttons for manual Swerve drive
  double xAxis = control->GetDriveX();
  double yAxis = control->GetDriveY();
  double rAxis = control->GetRotX();

  // This Boolean variable is going to be setted only in the default Amp and source
  // Positions with the robot non fied reference!
  bool FieldIsActive;

  // Default Joysticks and buttons for Top subsystems

  // Right Trigger to elevator
  double rightTrigger = control->GetRightTrigger();

  // Left Trigger for to elevator
  double leftTrigger = control->GetLeftTrigger();

  // right bumper to elevator
  bool rightBumper = control->GetRightBumper();

  // left bumper to elevator
  bool leftBumper = control->GetLeftBumper();

  // Y Button for source/human auto command
  bool Ybutton = control->GetYbutton();

  // A Button for  Chain auto command
  bool Abutton = control->GetAbutton();

  // X Button for Note auto command
  bool Xbutton = control->GetXbutton();

  // B Button for Amp auto command
  bool Bbutton = control->GetBbutton();

  // Start Button for reset Pigeon;
  bool StartButton = control->GetStartButton();

  // Test Buttons
  bool Y_TestButton = control->GetYTestButton();
  bool A_TestButton = control->GetATestButton();
  bool X_TestButton = control->GetXTestButton();
  double RightTrigger_TestButton = control->GetRightTriggerTestButton();
  double LeftTrigger_TestButton = control->GetLeftTriggerTestButton();

  //---------------------------MAKING THE LL METHODS SMALLER----------------------------------------------------------------

  // Going to Loop my actual Actual target high value, to be updated when i press a button.
  // at te same time that i make the method smaller to be setted in the logic
  double GetBackDist_Yaxis = limeLightBack_Subsystem->GetDistBack_Y(ActualAprilTargetHigh);

  // Going to Loop my actual Actual target high value, to be updated when i press a button.
  // at te same time that i make the method smaller to be setted in the logic
  double GetBackDist_Xaxis = limeLightBack_Subsystem->GetDistBack_X();

  // The actual robot angle
  double robot_Heading = swerve_Subsystem->GetHeading();

  // Front Lime methods

  // Get the front Id one
  double GetFrontId = limeLightFront_Subsystem->GetAprilId();
  // Get the TxFront one
  double TxFront = limeLightFront_Subsystem->GetTxFront();

  // Back Lime methods

  // Get the Back Id one
  double GetBackId = limeLightBack_Subsystem->GetAprilId();
  // GetTxBack one
  double TxBack = limeLightBack_Subsystem->GetTxBack();

  //********************************OFICIAL_LOGIC********************************************
  // Reseting the robot position by the Start Button
  if (StartButton == true)
  {

    swerve_Subsystem->ZeroHeading();
  }

  // Default Button for the source auto postion
  else if (Ybutton == true)
  {
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
      SourceRobotAngle = 200;
    }
    else
    {
      SourceRobotAngle = 20;
    }

    FieldIsActive = false;
    ActualAprilTargetHigh = LimeConstants::BackSourceHigh_toFloor;

    yAxis = -control->GetDriveY() / 3;
    xAxis = LimeLightPid_Tx.Calculate(TxBack, BackTxSource);
    rAxis = HeadingPidRot.Calculate(robot_Heading, SourceRobotAngle);

    giraffe_Subsystem->SetDesiredAngle(GiraffeConstants::SourceAngle);
  }
  else if (Bbutton == true)
  {
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
      AmpRobotAngle = LimeConstants::BlueAmpHeading;
    }
    else
    {
      AmpRobotAngle = LimeConstants::RedAmpHeading;
    }

    FieldIsActive = false;
    ActualAprilTargetHigh = LimeConstants::BackAmpHigh_toFloor;

    yAxis = -control->GetDriveY() * DriveConstants::SlowYaxisSpeed;
    xAxis = LimeLightPid_Tx.Calculate(TxBack, LimeConstants::BackTxAmp);
    rAxis = HeadingPidRot.Calculate(robot_Heading, AmpRobotAngle);

    giraffe_Subsystem->SetDesiredAngle(GiraffeConstants::AmpAngle);
  }
  else if(Abutton == true){
    rAxis = LimeLightPid_Tx.Calculate(TxFront, 0);
    giraffe_Subsystem->SetDesiredAngle(GiraffeConstants::DefaultAngle);
    FieldIsActive = false;
    xAxis = 0;
  }
  else if (Xbutton == true)
  {
    // if(limeLightBack_Subsystem->HasBackTarget() == true){
    rAxis = LimeLightPid_Tx.Calculate(TxBack, -9.5);
    giraffe_Subsystem->SetDesiredAngle(giraffe_Subsystem->SetPointAngCalculate(limeLightBack_Subsystem->GetTyBack()));
    FieldIsActive = true;
    // }
    // else{
    //   rAxis = 0;
    // }
  }

  else if (Y_TestButton == true)
  {
    giraffe_Subsystem->SetTestPotDownGiraffe();
  }
  else if (A_TestButton == true)
  {
    giraffe_Subsystem->SetTestPotUpGiraffe();
  }
  else if (X_TestButton == true)
  {
    giraffe_Subsystem->SetDesiredAngle(GiraffeConstants::BatAngle);
  }

  else
  {
    
    giraffe_Subsystem->SetDesiredAngle(GiraffeConstants::DefaultAngle);
    // giraffe_Subsystem->StopGiraffeMotor();
    FieldIsActive = true;
    ActualAprilTargetHigh = 0.0;
  }

  if (rightTrigger > 0.05)
  {
    if (elevator_Subsystem->GetRightElevatorPosition() > -0.1)
    {
      elevator_Subsystem->DownRightElevator();
    }
    else
    {
      elevator_Subsystem->StopRightElevator();
    }
  }
  else if (rightBumper == true)
  {
    if (elevator_Subsystem->GetRightElevatorPosition() < 70)
    {
      elevator_Subsystem->UpRightElevator();
    }
    else
    {
      elevator_Subsystem->StopRightElevator();
    }
  }
  else if(RightTrigger_TestButton > 0.05){
    elevator_Subsystem->DownRightElevator();
  }
  else
  {
    elevator_Subsystem->StopRightElevator();
  }

  if (leftTrigger > 0.05)
  {
    if (elevator_Subsystem->GetLeftElevatorPosition() > -0.1)
    {
      elevator_Subsystem->DownLeftElevator();
    }
    else
    {
      elevator_Subsystem->StopLeftElevator();
    }
  }
  else if (leftBumper == true)
  {
    if (elevator_Subsystem->GetLeftElevatorPosition() < 70)
    {
      elevator_Subsystem->UpLeftElevator();
    }
    else
    {
      elevator_Subsystem->StopLeftElevator();
    }
  }
   else if(LeftTrigger_TestButton > 0.05){
    elevator_Subsystem->DownLeftElevator();
  }
  else
  {
    elevator_Subsystem->StopLeftElevator();
  }


  //----------------------------------SWERVE_MANUAL----------------------------------------------

  // Apllying DeadBand
  xAxis = std::abs(xAxis) > OIconstants::kDeadband ? xAxis : 0.0;
  yAxis = std::abs(yAxis) > OIconstants::kDeadband ? yAxis : 0.0;
  rAxis = std::abs(rAxis) > OIconstants::kDeadband ? rAxis : 0.0;

  // Make the drive smoother
  xAxis = xAxisLimiter.Calculate(xAxis) * DriveConstants::kTeleDriveMaxSpeedMetersPerSecond;
  yAxis = yAxisLimiter.Calculate(yAxis) * DriveConstants::kTeleDriveMaxSpeedMetersPerSecond;
  rAxis = rAxisLimiter.Calculate(rAxis) * DriveConstants::kTeleDriveMaxAngularSpeedRadiansPerSecond;

  swerve_Subsystem->Drive(static_cast<units::meters_per_second_t>(yAxis),
                          static_cast<units::meters_per_second_t>(xAxis),
                          static_cast<units::radians_per_second_t>(rAxis),
                          FieldIsActive);
}

// Called once the command ends or is interrupted.
void SwerveCommand::End(bool interrupted)
{

  swerve_Subsystem->StopModules();
}

// Returns true when the command should end.
bool SwerveCommand::IsFinished()
{

  return false;
}

