// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoCommands/AutoShooterCommand.h"

AutoShooterCommand::AutoShooterCommand(ShooterSubsystem*shooter_SubsystemParam, LimeLightBackSubsystem*limeLightBack_SubsystemParam, SwerveSubsystem*swerve_SubsystemParam, GiraffeSubsystem*Giraffe_SubsystemParam,  IntakeSubsystem*intake_SubsystemParam): 
shooter_Subsystem(shooter_SubsystemParam), 
limeLightBack_Subsystem(limeLightBack_SubsystemParam), 
swerve_Subsystem(swerve_SubsystemParam), 
Giraffe_Subsystem(Giraffe_SubsystemParam),
intake_Subsystem(intake_SubsystemParam)
 {
  // Use addRequirements() here to declare subsystem dependencies.
    AddRequirements(shooter_SubsystemParam);

    AddRequirements(swerve_SubsystemParam);

    AddRequirements(limeLightBack_SubsystemParam);

    AddRequirements(Giraffe_SubsystemParam);

    AddRequirements(intake_SubsystemParam);

}

// Called when the command is initially scheduled.
void AutoShooterCommand::Initialize() {
  LimeLightPid_Tx.Reset();
  Giraffe_Subsystem->GiraffePid.Reset();
}

// Called repeatedly when this Command is scheduled to run
void AutoShooterCommand::Execute() {
  
  // double GiraffeAngleResult = GiraffeConstants::x_quad*(limeLightBack_Subsystem->GetTyBack()*limeLightBack_Subsystem->GetTyBack())
  //  +(GiraffeConstants::x_um*limeLightBack_Subsystem->GetTyBack())+GiraffeConstants::x_zero;

  double GiraffeAngResult = Giraffe_Subsystem->SetPointAngCalculate(limeLightBack_Subsystem->GetTyBack());

  double TxVelocity = LimeLightPid_Tx.Calculate(limeLightBack_Subsystem->GetTxBack(), -9.5)*2;

  swerve_Subsystem->Drive(0_mps, 0_mps, static_cast<units::radians_per_second_t>(TxVelocity), true);
  //if(Giraffe_Subsystem->GetAbsoluteEncoderRealRange() < GiraffeConstants::GiraffeAngMax && Giraffe_Subsystem->GetAbsoluteEncoderRealRange() > GiraffeConstants::GiraffeAngMin){
  Giraffe_Subsystem->SetDesiredAngle(GiraffeAngResult);
 // }
 // else{
 //   Giraffe_Subsystem->SetDesiredAngle(Giraffe_Subsystem->GetAbsoluteEncoderRealRange());
 // }

  if(Giraffe_Subsystem->GetAbsoluteEncoderRealRange() < (GiraffeAngResult + 0.1) && Giraffe_Subsystem->GetAbsoluteEncoderRealRange() > (GiraffeAngResult - 0.1)){
   shooter_Subsystem->LaunchShooterCommand(ShooterVelocity);
    //intake_Subsystem->CatchIntakeCommand(rightIntakeVelocity, leftIntakeVelocity);
     if(shooter_Subsystem->GetLeftShooterVelocity() < -87
      && shooter_Subsystem->GetLeftShooterVelocity() > -93){
       intake_Subsystem->CatchIntakeCommand(rightIntakeVelocity, leftIntakeVelocity);
       TimeToShoot.Start();
     }
  }
  // std::printf("RODANDO");
}

// Called once the command ends or is interrupted.
void AutoShooterCommand::End(bool interrupted) {
  shooter_Subsystem->StopShooterCommand();
  intake_Subsystem->StopIntakeCommand();
  TimeToShoot.Reset();
  TimeToShoot.Stop();
  // std::printf("ACABOU");
}

// Returns true when the command should end.
bool AutoShooterCommand::IsFinished() {
  if(TimeToShoot.Get() > 1_s){
    return true;
  }
  else{
    return false;
  }
  
  
}
