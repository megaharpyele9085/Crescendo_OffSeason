// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


//Ultrasonic Sensor Library
#include <frc/AnalogPotentiometer.h>

//All the Ultrasonic Constants
#include "Constants.h"  

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/SubsystemBase.h>


class UltrasonicSubsystem : public frc2::SubsystemBase {
  public:
  UltrasonicSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  //The method to retun my actual Sensor Distance
  double GetUltraSonicDist();


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //The Ultrassonic object 
  frc::AnalogPotentiometer Ultrasonic{
                                UltrasonicConstants::UltrasonicChannel,
                                UltrasonicConstants::UltrasonicMaxRange,
                                UltrasonicConstants::UltrasonicOffSet                               
                                     };
  
};
