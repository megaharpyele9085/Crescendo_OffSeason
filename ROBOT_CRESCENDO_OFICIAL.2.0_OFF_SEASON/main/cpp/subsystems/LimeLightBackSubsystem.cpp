// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimeLightBackSubsystem.h"

LimeLightBackSubsystem::LimeLightBackSubsystem() = default;



// This method will be called once per scheduler run
void LimeLightBackSubsystem::Periodic() 
{

 frc::SmartDashboard::PutBoolean("HasBackTarget",HasBackTarget());

}

double LimeLightBackSubsystem::GetTxBack(){

 txBack = networkTableBack->GetNumber("tx", 0.0);

 return txBack;

}

double LimeLightBackSubsystem::GetTyBack(){

 tyBack = networkTableBack->GetNumber("ty", 0.0);

 return tyBack;

}

double LimeLightBackSubsystem::GetTaBack(){

 taBack = networkTableBack->GetNumber("ta", 0.0);

 return taBack;

}

double LimeLightBackSubsystem::GetSkewBack(){

 tsBack = networkTableBack->GetNumber("targetSkew", 0.0);

 return tsBack;

}

double LimeLightBackSubsystem::GetTvBack(){

 tvBack = networkTableBack->GetNumber("tv", 0.0);

 return tvBack;

}

bool LimeLightBackSubsystem::HasBackTarget()
{

 if (GetTvBack() == 1)
 {
    hasBack = true;
 }else{
    hasBack = false;
 }
  
  return hasBack;
}
                                                
double LimeLightBackSubsystem::GetDistBack_Y(double BackTargetHightoFloor_parameter)

{
 
  BackTargetHigh_toFloor = BackTargetHightoFloor_parameter;

  catOpostoBack_H = BackTargetHigh_toFloor - BackLimeHigh_toFloor;

  double Ty_Degrees = TyIncrementalAngleBack + GetTyBack();
  double Ty_Radians = Ty_Degrees * (std::numbers::pi / 180.0);

  //The resultant dist between the central point of LimeLight and our target is given by the
  //opposite side of the projectable triangle   divided by the tan of the actual angle(-)

  //Distance formula
  DistBack_Y = (catOpostoBack_H)/tan(Ty_Radians);


  
  return DistBack_Y ;
}

double LimeLightBackSubsystem::GetDistBack_X()
{

  double Tx_Degrees = TxIncrementalAngleBack + GetTxBack();
  double Tx_Radians = Tx_Degrees * (std::numbers::pi / 180.0);

 DistBack_X = tan(Tx_Radians)*GetDistBack_Y(BackTargetHigh_toFloor);

 return DistBack_X;

}

double LimeLightBackSubsystem::GetAprilId()
{

 double aprilId = networkTableBack->GetNumber("tid",0.0);

 return aprilId;

 
}

void LimeLightBackSubsystem::SetPipeLine(double desiredPipeline)
{

  networkTableBack->PutNumber("pipeline",desiredPipeline);

}