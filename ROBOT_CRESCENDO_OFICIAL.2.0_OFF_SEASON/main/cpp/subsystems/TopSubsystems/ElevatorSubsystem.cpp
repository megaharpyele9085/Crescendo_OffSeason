// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TopSubsystems/ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem(): leftElevatorEncoder(leftElevatorMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42)),
rightElevatorEncoder(rightElevatorMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42))
{

rightElevatorMotor.SetInverted(BooleansConsts::lefttElevatorMotorReversed);
leftElevatorMotor.SetInverted(BooleansConsts::rightElevatorMotorReversed);

rightElevatorMotor.SetSmartCurrentLimit(0, 30);
leftElevatorMotor.SetSmartCurrentLimit(0, 30);

rightElevatorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
leftElevatorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

rightElevatorMotor.BurnFlash();
leftElevatorMotor.BurnFlash();
};

void ElevatorSubsystem::UpRightElevator(){
  rightElevatorMotor.Set(0.3);
}

void ElevatorSubsystem::UpLeftElevator(){
  leftElevatorMotor.Set(0.3);
}

void ElevatorSubsystem::DownLeftElevator(){
  leftElevatorMotor.Set(-0.3);
}

void ElevatorSubsystem::DownRightElevator(){
  rightElevatorMotor.Set(-0.3);
}

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() 
{
 
//Right elevator Pid's config in shuffleboard
 frc::SmartDashboard::SetDefaultNumber("rightElevatorPID_P",rightElevatorPid_P);
rightElevatorPid_P = frc::SmartDashboard::GetNumber("rightElevatorPID_P",rightElevatorPid_P);


frc::SmartDashboard::SetDefaultNumber("rightElevatorPID_I",rightElevatorPid_I);
rightElevatorPid_I = frc::SmartDashboard::GetNumber("rightElevatorPID_I",rightElevatorPid_I);


frc::SmartDashboard::SetDefaultNumber("rightElevatorPID_D",rightElevatorPid_D);
rightElevatorPid_D = frc::SmartDashboard::GetNumber("rightElevatorPID_D",rightElevatorPid_D);



 //Left elevator Pid's configs in shuffleboard
frc::SmartDashboard::SetDefaultNumber("leftElevatorPID_P",leftElevatorPid_P);
leftElevatorPid_P = frc::SmartDashboard::GetNumber("leftElevatorPID_P",leftElevatorPid_P);


frc::SmartDashboard::SetDefaultNumber("leftElevatorPID_I",leftElevatorPid_I);
leftElevatorPid_I = frc::SmartDashboard::GetNumber("leftElevatorPID_I",leftElevatorPid_I);


frc::SmartDashboard::SetDefaultNumber("leftElevatorPID_D",leftElevatorPid_D);
leftElevatorPid_D = frc::SmartDashboard::GetNumber("leftElevatorPID_D",leftElevatorPid_D);

//Temps and currents
frc::SmartDashboard::PutNumber("Left Elevator MotorTemp (C deg)", leftElevatorMotor.GetMotorTemperature());
frc::SmartDashboard::PutNumber("Left Elevator Current (A)", leftElevatorMotor.GetOutputCurrent());

frc::SmartDashboard::PutNumber("Right Elevator MotorTemp (C deg)", rightElevatorMotor.GetMotorTemperature());
frc::SmartDashboard::PutNumber("Right Elevator Current (A)", rightElevatorMotor.GetOutputCurrent());


// rightElevatorMotor.Set(desiredMaxHigh);
// leftElevatorMotor.Set(desiredMinHigh);
// if(//(rightElevatorMotor.GetOutputCurrent() < ElevatorConstants::ElevatorMotorCurrent)&&
// (rightElevatorMotor.GetMotorTemperature() > ElevatorConstants::ElevatorMotorTemp)&&
// (GetRightElevatorPosition() > ElevatorConstants::ChainElevatorPositionLevel))
// {
//   desiredMaxHigh = 0;
// }
// else if((leftElevatorMotor.GetMotorTemperature() < ElevatorConstants::ElevatorMotorTemp)&&
// (GetLeftElevatorPosition() > ElevatorConstants::ChainElevatorPositionLevel)){
//  desiredMinHigh = 0;
// }


// if( (rightElevatorMotor.GetMotorTemperature() > ElevatorConstants::ElevatorMotorTemp)&&
// (GetRightElevatorPosition() < ElevatorConstants::MinElevatorPosition)){
//   desiredMaxHigh = 0;
// }
// else if((leftElevatorMotor.GetMotorTemperature() < ElevatorConstants::ElevatorMotorTemp)&&
// (GetLeftElevatorPosition() < ElevatorConstants::MinElevatorPosition)){
//   desiredMinHigh = 0;
// }

// frc::SmartDashboard::PutNumber("LeftElevator_Encoder",GetLeftElevatorPosition());
// frc::SmartDashboard::PutNumber("RightElevator_Encoder",GetRightElevatorPosition());



}

//This method is going to provide the max encoder value wich I can put
// as the max reference point to up the Elevator to the chain
void ElevatorSubsystem::UpArmToChain(double desiredMaxHigh_Parameter)
{
  desiredMaxHigh = desiredMaxHigh_Parameter;
  desiredMinHigh = desiredMaxHigh_Parameter;

// if(//(rightElevatorMotor.GetOutputCurrent() < ElevatorConstants::ElevatorMotorCurrent)&&
// (rightElevatorMotor.GetMotorTemperature() < ElevatorConstants::ElevatorMotorTemp)&&
// (GetRightElevatorPosition() < ElevatorConstants::ChainElevatorPositionLevel)){

//   rightElevatorMotor.Set(ElevatorConstants::ElevatorPot);

// }
// else{

//   rightElevatorMotor.StopMotor();

// }

// if(//(leftElevatorMotor.GetOutputCurrent() < ElevatorConstants::ElevatorMotorCurrent)&&
// (leftElevatorMotor.GetMotorTemperature() < ElevatorConstants::ElevatorMotorTemp)&&
// (GetLeftElevatorPosition() < ElevatorConstants::ChainElevatorPositionLevel)){

//   leftElevatorMotor.Set(ElevatorConstants::ElevatorPot);

// }
// else{

//   leftElevatorMotor.StopMotor();

// }

}

//This method is going to provide the minimum encoder value wich I can put
// as the in reference point to down the Elevator to the robot
void ElevatorSubsystem::DownArmToRobot(double desiredMinHigh_Parameter)
{

desiredMaxHigh = desiredMinHigh_Parameter;
desiredMinHigh = desiredMinHigh_Parameter;
  
//   desiredMinHigh = desiredMinHigh_Parameter;


//   if(//(rightElevatorMotor.GetOutputCurrent() < ElevatorConstants::ElevatorMotorCurrent)&&
// (rightElevatorMotor.GetMotorTemperature() < ElevatorConstants::ElevatorMotorTemp)&&
// (GetRightElevatorPosition() > ElevatorConstants::MinElevatorPosition)){

//   rightElevatorMotor.Set(-ElevatorConstants::ElevatorPot);

// }
// else{

//   rightElevatorMotor.StopMotor();

// }

// if(//(leftElevatorMotor.GetOutputCurrent() < ElevatorConstants::ElevatorMotorCurrent)&&
// (leftElevatorMotor.GetMotorTemperature() < ElevatorConstants::ElevatorMotorTemp)&&
// (GetLeftElevatorPosition() > ElevatorConstants::MinElevatorPosition)){

//   leftElevatorMotor.Set(-ElevatorConstants::ElevatorPot);

// }
// else{

//   leftElevatorMotor.StopMotor();

// }

}


void ElevatorSubsystem::ResetElevatorEncoders()
{
 rightElevatorEncoder.SetPosition(0.0);
 leftElevatorEncoder.SetPosition(0.0);
}

void ElevatorSubsystem::StopElevator()
{
   
 desiredMaxHigh = 0;
 desiredMinHigh = 0;

}

double ElevatorSubsystem::GetLeftElevatorPosition()
{

return leftElevatorEncoder.GetPosition();
}

double ElevatorSubsystem::GetRightElevatorPosition()
{

return rightElevatorEncoder.GetPosition();
}

void ElevatorSubsystem::StopLeftElevator(){
  leftElevatorMotor.StopMotor();
}

void ElevatorSubsystem::StopRightElevator(){
  rightElevatorMotor.StopMotor();
}
