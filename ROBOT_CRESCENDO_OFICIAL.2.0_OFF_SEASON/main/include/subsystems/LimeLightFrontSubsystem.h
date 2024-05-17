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

#include "Constants.h"


class LimeLightFrontSubsystem : public frc2::SubsystemBase {
 public:
  LimeLightFrontSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  double GetTxFront();

  double GetTyFront();

  double GetTaFront();

  double GetSkewFront();

  double GetDistFront();

  double GetTvFront();

  double GetPipe();

  double GetAprilId();
 
  void SetPipeLine(double desiredPipe);

  bool   HasTargetFront();



 private:

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  std::shared_ptr<nt::NetworkTable> networkTableFront = nt::NetworkTableInstance::GetDefault()
	.GetTable("limelight-front");
  

  //Tx , Ty , Ta and Tskew values
  double txFront;
  double tyFront; 
  double taFront;
  double tsFront;
  double tvFront;

  //PipeLine variable
  double actualPipe;

  //This one represents the Tv abs value, which can be represented by a boolean in a 
  //method
  bool hasTargetFront;

    //Dist calculate constants,
  // to calculate our distance in 90 degress you may have to minus the high 
  //of the target - the high of the limelight from the floor, after that you just have to change your
  //ty value to radians, and at lest divide the catOpos to the tan of your ty value in radians.
 
  //OBS* when you put the Lime at 90 degrees, in  our robot, the Ty angle is  15.52


  // to calculate the dist of your limelight in diferents angles you may have to take a note of your
  // ty value from the 90 degress and then change your LL angle take a look of your ty value again 
  // and make sure your atual value be the 90 degress value again with a math operation and repite the 
  // above process
 
  //OBS* the Tan function doesn't work with non radians values

  //The actual distance of LL after all the upper operations
  double distFront = 0.0;
  
  //The  incremental LL angle                                                  -
  const double TyIncrementalAngleFront = 27.716;

  double TyIncrementalRadiansFront = 0;
                            

                            
  //In inches
  const double FrontLimeHigh_toFloor = 13.5;

  //In inches     
  const double FrontTargetHigh_toFloor = 48.03;

  //the actual high of the rectangle triangle
  double const catOpostoFront = FrontTargetHigh_toFloor- FrontLimeHigh_toFloor;
  
  //Ty
   double tyradFront;

};
