// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TopSubsystemGroup.h"

TopSubsystemGroup::TopSubsystemGroup(IntakeSubsystem*intake_SubsystemParam, ShooterSubsystem*shooter_SubsystemParam, GiraffeSubsystem*giraffe_SubsystemParam, LimeLightBackSubsystem*lime_SubsystemParam): intake_Subsystem{intake_SubsystemParam}, shooter_Subsystem{shooter_SubsystemParam}, giraffe_Subsystem{giraffe_SubsystemParam}, lime_Subsystem{lime_SubsystemParam}{}


// This method will be called once per scheduler run
void TopSubsystemGroup::Periodic() {}

void TopSubsystemGroup::ToShoot(){
    double GiraffeAngResult = giraffe_Subsystem->SetPointAngCalculate(lime_Subsystem->GetTyBack());
    giraffe_Subsystem->SetDesiredAngle(GiraffeAngResult);
    if(giraffe_Subsystem->GetAbsoluteGiraffeAngle() > (GiraffeAngResult - 2) && giraffe_Subsystem->GetAbsoluteGiraffeAngle() < (GiraffeAngResult + 2)){
        shooter_Subsystem->LaunchShooterCommand(90);
    }
    // giraffe_Subsystem->SetTestPotUpGiraffe();
}

void TopSubsystemGroup::ToStop(){
    giraffe_Subsystem->StopGiraffeMotor();
    shooter_Subsystem->StopShooterCommand();
}

bool TopSubsystemGroup::ToFinish(){
    // if(giraffe_Subsystem->GetAbsoluteGiraffeAngle() > 225){
    //     return true;
    // }
    // else{
    //     return false;
    // }
    if(shooter_Subsystem->GetLeftShooterVelocity() > 89){
        return true;
    }
    else{
        return false;
    }
}