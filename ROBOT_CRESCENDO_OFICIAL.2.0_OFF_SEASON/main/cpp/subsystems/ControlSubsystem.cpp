// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ControlSubsystem.h"

ControlSubsystem::ControlSubsystem() = default;

// This method will be called once per scheduler run
void ControlSubsystem::Periodic() 
{
 
}

//Analogic Methods
double ControlSubsystem::GetDriveX(){ return xbox.GetLeftX(); }
double ControlSubsystem::GetDriveY(){ return xbox.GetLeftY(); }
double ControlSubsystem::GetRotX(){ return xbox.GetRightX(); }
double ControlSubsystem::GetRightTrigger(){return xbox.GetRightTriggerAxis();}
double ControlSubsystem::GetLeftTrigger(){return xbox.GetLeftTriggerAxis();}


//Bool DRIVE methods
bool ControlSubsystem::GetYbutton(){return xbox.GetYButton(); }
bool ControlSubsystem::GetAbutton(){return xbox.GetAButton(); }
bool ControlSubsystem::GetXbutton(){return xbox.GetXButton(); }
bool ControlSubsystem::GetBbutton(){return xbox.GetBButton(); }
bool ControlSubsystem::GetRightBumper(){return xbox.GetRightBumper();}
bool ControlSubsystem::GetLeftBumper(){return xbox.GetLeftBumper();}
bool ControlSubsystem::GetStartButton(){return xbox.GetStartButton();}

bool ControlSubsystem::GetFieldOriented(){ 
    
    
    if(xbox.GetLeftBumper()){
        kField = false;
    }
    else if(xbox.GetRightBumper()){
        kField = true;
    }
    return kField;
}

//Xbox test methods
bool ControlSubsystem::GetATestButton()
{
    return test_xbox.GetAButton();
}

bool ControlSubsystem::GetYTestButton()
{
    return test_xbox.GetYButton();
}

bool ControlSubsystem::GetXTestButton()
{
    return test_xbox.GetXButton();
}

bool ControlSubsystem::GetLeftBumperTestButton(){
    return test_xbox.GetLeftBumper();
}

bool ControlSubsystem::GetRightBumperTestButton(){
    return test_xbox.GetRightBumper();
}

double ControlSubsystem::GetRightTriggerTestButton(){
    return test_xbox.GetRightTriggerAxis();
}

double ControlSubsystem::GetLeftTriggerTestButton(){
    return test_xbox.GetLeftTriggerAxis();
}