// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Constants.h"

class ControlSubsystem : public frc2::SubsystemBase {
 public:
  ControlSubsystem();

  //Get the Left X axis of  XboxController 
  double GetDriveX();

  //Get the Left Y axis of  XboxController 
  double GetDriveY();

  //Get the Right X axis of  XboxController
  double GetRotX();

  //Get the Right trigger of drive XboxController
  double GetRightTrigger(); 

  //Get the Left trigger of drive XboxController
  double GetLeftTrigger(); 

  //Method which returns the field Oriented condition , in function with right and left bumpers 
  bool GetFieldOriented();

  //Get the Y button of XboxController
  bool GetYbutton();

  //Get the A button of XboxController
  bool GetAbutton();

  //Get the X button of XboxController
  bool GetXbutton();

  //Get the B button of XboxController
  bool GetBbutton();

  //Get the Right Bumper of XboxController
  bool GetRightBumper();

  //Get the Left Bumper of XboxController
  bool GetLeftBumper();

  //Get the Start Button of XboxController
  bool GetStartButton();

  //TESTE XBOX CONFIGS
 
  //Get the bool value of the A button
  bool GetATestButton();
  
  //Get the bool value of the Y button
  bool GetYTestButton();

 //Get the bool value of the Y button
  bool GetXTestButton();

  //Get the bool value of the Y button
  bool GetLeftBumperTestButton();
 
  //Get the bool value of the Y button
  bool GetRightBumperTestButton();

  //Get the bool value of the Y button
  double GetLeftTriggerTestButton();

  //Get the bool value of the Y button
  double GetRightTriggerTestButton();

  //Field Oriented Condition
  bool kField;
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::XboxController xbox{OperatorConstants::kDriverControllerPort};

  //test controller
  frc::XboxController test_xbox{OperatorConstants::kManualTestControllerPort};

  
};
