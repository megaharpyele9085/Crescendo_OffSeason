// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TopSubsystems/GiraffeSubsystem.h"

GiraffeSubsystem::GiraffeSubsystem()
{

   // set units of the CANCoder to radians, with velocity being radians per second
   /* Configure CANcoder */
   ctre::phoenix6::configs::CANcoderConfiguration CanCoder_Config{};

   // Cancoder Configs
   CanCoder_Config.MagnetSensor.AbsoluteSensorRange.Unsigned_0To1;

   CanCoder_Config.MagnetSensor.SensorDirection.Clockwise_Positive;

   GiraffeCanCoder.GetConfigurator().Apply(CanCoder_Config);

   //GiraffePid.EnableContinuousInput(GiraffeConstants::GiraffeAngMin, GiraffeConstants::GiraffeAngMax);

   GiraffePid.SetOutputRange(-1, 1);

   GiraffePid.Reset();

   GiraffeMotor.SetInverted(BooleansConsts::GiraffeMotorReversed);

   GiraffeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

   GiraffeMotor.SetSmartCurrentLimit(0, 30);

   GiraffeMotor.BurnFlash();
}
// This method will be called once per scheduler run
void GiraffeSubsystem::Periodic()
{
   // get

   GiraffeError = GiraffePid.GetError();

   // Gets the Absolute giraffe angle
   frc::SmartDashboard::PutNumber("Giraffe_AbsolutePosition", GetAbsoluteGiraffeAngle());

   // Gets the real value at the shuffle board
   frc::SmartDashboard::PutNumber("Giraffe_RealRange", GetAbsoluteEncoderRealRange());

   // All the Giraffe gains and shuffler
   // frc::SmartDashboard::SetDefaultNumber("GiraffePID_P", GiraffePid_P);
   // GiraffePid_P = frc::SmartDashboard::GetNumber("GiraffePID_P", GiraffePid_P);

   frc::SmartDashboard::SetDefaultNumber("GiraffePID_I", GiraffePid_I);
   GiraffePid_I = frc::SmartDashboard::GetNumber("GiraffePID_I", GiraffePid_I);

   // frc::SmartDashboard::SetDefaultNumber("GiraffePID_D", GiraffePid_D);
   // GiraffePid_D = frc::SmartDashboard::GetNumber("GiraffePID_D", GiraffePid_D);

   frc::SmartDashboard::PutNumber("Error Giraffe", GetErrorPID());

   frc::SmartDashboard::PutNumber("Set Ang Giraffe Poli", SetPointAngCalculate(Lime_Back.GetTyBack()));

   frc::SmartDashboard::PutNumber("Set PID Ang Giraffe", GiraffePid.GetSetpoint());

   frc::SmartDashboard::PutNumber("Calculate Giraffe", GiraffeSpeeds);

   frc::SmartDashboard::PutNumber("Giraffe MotorTemp (C deg)", GiraffeMotor.GetMotorTemperature());

   frc::SmartDashboard::PutNumber("Giraffe Motor Current (A)", GiraffeMotor.GetOutputCurrent());
}

double GiraffeSubsystem::GetAbsoluteGiraffeAngle()
{

   return GiraffeCanCoder.GetAbsolutePosition().GetValueAsDouble();
}

void GiraffeSubsystem::SetDesiredAngle(double desiredAngle_Parameter)
{
   GiraffePid.SetPID(GiraffePid_P, GiraffePid_I, GiraffePid_D, 20, 0.39364_s, 0.01_s);

   // GiraffeSpeeds = GiraffePid.Calculate(GetAbsoluteEncoderRealRange(), desiredAngle);

   if ((GiraffeCanCoder.GetFault_BadMagnet().GetValue() == false) &&
       //(GiraffeMotor.GetOutputCurrent() < GiraffeConstants::GiraffeMotorCurrent) &&
       (GiraffeMotor.GetMotorTemperature() < GiraffeConstants::GiraffeMotorTemp) &&
       (GetAbsoluteEncoderRealRange() < GiraffeConstants::GiraffeAngMax &&
          GetAbsoluteEncoderRealRange() > GiraffeConstants::GiraffeAngMin))
   {
      // GiraffeMotor.Set(GiraffeSpeeds);
      desiredAngle = std::clamp(desiredAngle_Parameter, GiraffeConstants::GiraffeAngMin, GiraffeConstants::GiraffeAngMax);
   }
   else
   {
      desiredAngle = GetAbsoluteEncoderRealRange();
   }

   GiraffeSpeeds = GiraffePid.Calculate(GetAbsoluteEncoderRealRange(), desiredAngle);
   
  // if(abs(GiraffeSpeeds) < GiraffeConstants::MaxPIDErrorOut){
   //   GiraffeMotor.Set(0);
 //  }
 //  else{
      GiraffeMotor.Set(GiraffeSpeeds);
  // }
}

double GiraffeSubsystem::GetErrorPID()
{
   // return GiraffePid.Calculate();
   return GiraffePid.GetError();
}

double GiraffeSubsystem::GetCalculate()
{
   return GiraffePid.Calculate(GetAbsoluteEncoderRealRange(), desiredAngle);
}

double GiraffeSubsystem::GetAbsoluteEncoderRealRange()
{
   // Creating a range for 0 to 360 with this factory range being in usage(-0.5,0.5)
   double angle = GiraffeCanCoder.GetAbsolutePosition().GetValueAsDouble() * 360;
   if (angle < 0)
   {
      angle += 360;
   }
   if (angle > 360)
   {
      angle -= 360;
   }

   // Creating a range for 0 to 360 with this factory range being in usage(0,1)
   // double angle = GiraffeCanCoder.GetAbsolutePosition().GetValueAsDouble()*360 ;

   return (360.0 - angle);
}

void GiraffeSubsystem::SetTestPotUpGiraffe()
{
   GiraffeMotor.Set(GiraffeConstants::TestGiraffePot);
}

void GiraffeSubsystem::SetTestPotDownGiraffe()
{
   GiraffeMotor.Set(-GiraffeConstants::TestGiraffePot);
}

void GiraffeSubsystem::StopGiraffeMotor()
{
   GiraffeMotor.StopMotor();
}

bool GiraffeSubsystem::IsGiraffeOnTop(){
   if(GetAbsoluteEncoderRealRange() > 225){
      return true;
   }
   else{
      return false;
   }
}

double GiraffeSubsystem::SetPointAngCalculate(double InputX)
{
   return ((GiraffeConstants::x_quad * InputX) + (GiraffeConstants::x_um * InputX) + GiraffeConstants::x_zero);
}
