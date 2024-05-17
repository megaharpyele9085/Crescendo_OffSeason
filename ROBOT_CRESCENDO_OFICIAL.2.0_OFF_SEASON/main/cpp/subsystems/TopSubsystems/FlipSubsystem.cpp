// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TopSubsystems/FlipSubsystem.h"

FlipSubsystem::FlipSubsystem() 
{
    // Phoenix 5
 flipCanCoder.ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360);

 flipMotor.SetInverted(BooleansConsts::flipMotorReversed);

 flipMotor.SetSmartCurrentLimit(0, 35);

 flipMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

 flipMotor.BurnFlash();

}

// This method will be called once per scheduler run
void FlipSubsystem::Periodic() 
{



//  frc::SmartDashboard::PutNumber("Flip_CanCoder",FlipCanCoder.GetAbsolutePosition());

}

void FlipSubsystem::MaxAnlgeToAmp(double desiredMaxAngleToAmp_Parameter)
{
    desiredMaxAngleToAmp = desiredMaxAngleToAmp_Parameter;

flipMotor.Set(
    flipPid.Calculate(flipCanCoder.GetAbsolutePosition(),desiredMaxAngleToAmp)
);

}

void FlipSubsystem::MinAngleToRobot(double desiredMinAngleToRobot_Parameter)
{
    desiredMinAngleToRobot = desiredMinAngleToRobot_Parameter;

flipMotor.Set(
    flipPid.Calculate(flipCanCoder.GetAbsolutePosition(),desiredMinAngleToRobot)
);

}


void FlipSubsystem::StopFlipMotor()
{
 flipMotor.StopMotor();  
}