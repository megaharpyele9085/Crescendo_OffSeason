// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimeLightFrontSubsystem.h"

LimeLightFrontSubsystem::LimeLightFrontSubsystem() = default;


    
// This method will be called once per scheduler run
void LimeLightFrontSubsystem::Periodic()
{
 frc::SmartDashboard::PutNumber("FrontLL_Dist",GetDistFront());
 frc::SmartDashboard::PutNumber("FrontLL_Pipe",GetPipe());
 frc::SmartDashboard::PutBoolean("HasFrontTarget?",HasTargetFront());
 
}

double LimeLightFrontSubsystem::GetTxFront(){

 txFront = networkTableFront->GetNumber("tx", 0.0);

 return txFront;
 
}

double LimeLightFrontSubsystem::GetTyFront(){

 tyFront = networkTableFront->GetNumber("ty", 0.0);

 return tyFront;

}

double LimeLightFrontSubsystem::GetTaFront(){

 taFront = networkTableFront->GetNumber("ta", 0.0);

 
 return taFront;
}

double LimeLightFrontSubsystem::GetSkewFront(){

 tsFront = networkTableFront->GetNumber("targetSkew", 0.0);
 
 return tsFront;
 
}


double LimeLightFrontSubsystem::GetTvFront(){

tvFront = networkTableFront->GetNumber("tv",0.0);

  return tvFront;

}

bool LimeLightFrontSubsystem::HasTargetFront()
{

 if(GetTvFront() == 1){

  hasTargetFront = true;

 } 
 
 else
 {
  hasTargetFront = false;
 }

  return hasTargetFront;
}

//This is a very important function which returns a inch distance between the 
//LimeLight and the Target, that depends from the different Lime and Target high(catetoOposto),
// the back or front Lime angle to increment in Ty angle.     
double LimeLightFrontSubsystem::GetDistFront(){
  
  
  double angleToGoalDegrees = TyIncrementalAngleFront + GetTyFront();
  double angleToGoalRadians = angleToGoalDegrees * (std::numbers::pi / 180.0);


  //The resultant dist between the central point of LimeLight and our target is given by the
  //opposite side of the projectable triangle   divided by the tan of the actual angle(-)

  //Distance formula
  distFront = (catOpostoFront)/tan(angleToGoalRadians);

  
  return distFront;
  

}

double LimeLightFrontSubsystem::GetPipe()
{
 
 actualPipe = networkTableFront->GetNumber("getpipe",0.0);

 return actualPipe;
}

void LimeLightFrontSubsystem::SetPipeLine(double desiredPipe)
{
  
  networkTableFront->PutNumber("pipeline",desiredPipe);
 
}

double LimeLightFrontSubsystem::GetAprilId()
{

 double aprilId = networkTableFront->GetNumber("tid",0.0);

 return aprilId;
}


