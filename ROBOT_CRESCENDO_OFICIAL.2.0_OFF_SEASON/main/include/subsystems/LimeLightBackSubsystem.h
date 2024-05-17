// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

class LimeLightBackSubsystem : public frc2::SubsystemBase {
 public:

  LimeLightBackSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  double GetTxBack();

  double GetTyBack();

  double GetTaBack();

  double GetSkewBack();

  double GetDistBack_Y(double BackTargetHightoFloor_parameter);

  double GetDistBack_X();
  
  double GetTvBack();

  double GetAprilId();

  bool   HasBackTarget();

  void SetPipeLine(double desiredPipe);

 private:
  //Limelight declaration
  std::shared_ptr<nt::NetworkTable> networkTableBack = nt::NetworkTableInstance::GetDefault()
		.GetTable("limelight-back");

  //Tx , Ty , Ta and Tskew values
  double txBack;
  double tyBack;
  double taBack;
  double tsBack;
  double tvBack;

  bool hasBack;

  //the TxDist Returnable variable
  double DistBack_X;

  //The consts values to be setted on Y_Axis Dist.

    //Dist calculate constants,
  // to calculate our distance in 90 degress you may have to minus the high 
  //of the target - the high of the limelight from the floor, after that you just have to change your
  //ty value to radians, and at lest divide the catOpos to the tan of your ty value in radians.
 
  //OBS* when you put the Lime at 90 degrees, in  our robot, the Ty angle is  15.52

  // to calculate the dist of your limelight in diferents angles you may have to take a note of your
  // ty value from the 90 degress and then change your LL angle take a look of your ty value again 
  // and make sure your actual value be the 90 degress value again with a math operation and repite the 
  // above process
 
  //OBS* the Tan function doesn't work with non radians values

  //Enter in the return method
  double DistBack_Y;


  // The target high(above the floor) in your arena minus
  // The LL high(above the floor), on your robot

  //In inches                        
  const double BackLimeHigh_toFloor = 0.22;

  // //In inches                           source high
  // const double BackTargetHigh_toFloor = 48.03;


  double BackTargetHigh_toFloor;

  //the actual high of the rectangle triangle
  // double catOpostoBack = BackTargetHigh_toFloor- BackLimeHigh_toFloor;
  double catOpostoBack_H;  

  // The incremental Y and X angle to be put in the dist formula 
  double const TyIncrementalAngleBack = 27.716;
  double const TxIncrementalAngleBack = 0.0;

  
  
  

};
