// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TopSubsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem()
{

    leftIntakeMotor.SetInverted(BooleansConsts::leftIntakeMotorReversed);
    rightIntakeMotor.SetInverted(BooleansConsts::rightIntakeMotorReversed);
    rightIntakeMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

    rightIntakeMotor.SetSmartCurrentLimit(0, 6);
    rightIntakeMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    rightIntakeMotor.BurnFlash();
    
    leftIntakeMotor.SetSmartCurrentLimit(0, 6);
    // leftIntakeMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    leftIntakeMotor.BurnFlash();
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic()
{

    // Shuffler
    frc::SmartDashboard::PutNumber("Intake NEO550 Left MotorTemp (C deg)", leftIntakeMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Intake NEO550 Left Current (A)", leftIntakeMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Intake NEO550 Right MotorTemp (C deg)", rightIntakeMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Intake NEO550 Right Current (A)", rightIntakeMotor.GetOutputCurrent());
}

void IntakeSubsystem::CatchIntakeCommand(double rightCatchIntake_SpeedParamter, double leftCatchintake_SpeedParameter)
{

    if ((leftIntakeMotor.GetMotorTemperature() < IntakeConstants::IntakeMotor550Temp) &&
        (rightIntakeMotor.GetMotorTemperature() < IntakeConstants::IntakeMotor550Temp))
    {

        rightIntakeMotor.Set(rightIntake_Filter.Calculate(rightCatchIntake_SpeedParamter));
        leftIntakeMotor.Set(LeftIntake_Filter.Calculate(leftCatchintake_SpeedParameter));
    }
    else
    {
        StopIntakeCommand();
    }
}

void IntakeSubsystem::SpitIntakeCommand(double rightSpitIntake_SpeedParamter, double leftSpitIntake_SpeedParameter)
{

    if ((leftIntakeMotor.GetMotorTemperature() < IntakeConstants::IntakeMotor550Temp) &&
        (rightIntakeMotor.GetMotorTemperature() < IntakeConstants::IntakeMotor550Temp))
    {

        rightIntakeMotor.Set(rightIntake_Filter.Calculate(rightSpitIntake_SpeedParamter));
        leftIntakeMotor.Set(LeftIntake_Filter.Calculate(leftSpitIntake_SpeedParameter));
    }
    else
    {
        StopIntakeCommand();
    }
}

void IntakeSubsystem::StopIntakeCommand()
{

    rightIntakeMotor.StopMotor();
    leftIntakeMotor.StopMotor();
}
