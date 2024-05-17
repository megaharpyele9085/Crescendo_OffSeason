// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveSubsystem.h"


SwerveSubsystem::SwerveSubsystem()
    : FrontLeft{DriveConstants::kFrontLeftDriveMotor, DriveConstants::kFrontLeftTurningMotor,
                BooleansConsts::kFrontLeftDriveEncoderReversed,
                BooleansConsts::kFrontLeftTurningEncoderReversed,
                CanSensorConstants::kFrontLeftTurningSensor,
                DefAngModule::FL_DEGREE_OFFSET,
                BooleansConsts::kFrontLeftAbsoluteEncoderReversed},

      RearLeft{DriveConstants::kBackLeftDriveMotor, DriveConstants::kBackLeftTurningMotor,
               BooleansConsts::kBackLeftDriveEncoderReversed,
               BooleansConsts::kBackLeftTurningEncoderReversed,
               CanSensorConstants::kBackLeftTurningSensor,
               DefAngModule::RL_DEGREE_OFFSET,
               BooleansConsts::kBackLeftAbsoluteEncoderReversed},

      FrontRight{DriveConstants::kFrontRightDriveMotor, DriveConstants::kFrontRightTurningMotor,
                 BooleansConsts::kFrontRightDriveEncoderReversed,
                 BooleansConsts::kFrontRightTurningEncoderReversed,
                 CanSensorConstants::kFrontRightTurningSensor,
                 DefAngModule::FR_DEGREE_OFFSET,
                 BooleansConsts::kFrontRightAbsoluteEncoderReversed},

      RearRight{DriveConstants::kBackRightDriveMotor, DriveConstants::kBackRightTurningMotor,
                BooleansConsts::kBackRightDriveEncoderReversed,
                BooleansConsts::kBackRightTurningEncoderReversed,
                CanSensorConstants::kBackRightTurningSensor,
                DefAngModule::RR_DEGREE_OFFSET,
                BooleansConsts::kBackRightAbsoluteEncoderReversed},
      
      Odometry{DriveConstants::kDriveKinematics,
                 GetRotation2d(),
                 {FrontLeft.GetPosition(), FrontRight.GetPosition(),
                  RearLeft.GetPosition(), RearRight.GetPosition()},
                 frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}}

      
                        
{    
 //Set all the pigeon configs before this method.
 pigeon.GetConfigurator().Apply(pigeon_Config);


//A thread of execution is a sequence of instructions that can be executed concurrently 
//with other such sequences in multithreading environments, while sharing a same address space.
//We've to create a thread to delay in 1 sec, and in sequence we execute  the ZeroHeading action! 
//Finalizating the trhead call
    std::thread([this]{
        try { 
            std::this_thread::sleep_for(std::chrono::seconds(1)); //First execution
            ZeroHeading(); //Second execution
        } catch(const std::exception& e){

        }
    }).detach();

    pathplanner::AutoBuilder::configureHolonomic(

    [this](){ return GetPose2d(); },
    [this](frc::Pose2d HolonomicPose){ResetOdometry(HolonomicPose);},
    [this](){return this -> GetSwerveSpeed();},
    [this](frc::ChassisSpeeds robotRelativeSpeeds) {this->driveRobotRelative(robotRelativeSpeeds);},
    DriveConstants::pathFollowerConfig,
    []()
    {
    // Boolean supplier that controls when the path will be mirrored for the red alliance
    // This will flip the path being followed to the red side of the field.
    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
    },
    this
        
    );

}

void SwerveSubsystem::ZeroHeading(){
    // ahrs->Reset();
    pigeon.Reset();

}

double SwerveSubsystem::GetHeading(){

    
    // return std::remainder(ahrs->GetAngle(),360);
    return std::remainder(pigeon.GetAngle(),360);
}

frc::Rotation2d SwerveSubsystem::GetRotation2d(){
//    return ahrs->GetRotation2d().Degrees();
return pigeon.GetRotation2d().Degrees();
}

frc::Pose2d SwerveSubsystem::GetPose2d(){

     return Odometry.GetPose();
 }

void SwerveSubsystem::ResetOdometry(frc::Pose2d pose){

Odometry.ResetPosition(GetRotation2d(),
      {FrontLeft.GetPosition(), FrontRight.GetPosition(),
       RearLeft.GetPosition(), RearRight.GetPosition()},
      pose);
}

// This method will be called once per scheduler run
void SwerveSubsystem::Periodic() 
{

    Odometry.Update(GetRotation2d(), 
    {FrontLeft.GetPosition() , RearLeft.GetPosition(),
    FrontRight.GetPosition(), RearRight.GetPosition()});  

    frc::SmartDashboard::PutNumber("Robot Heading", GetHeading());
    frc::SmartDashboard::PutNumber("FL_AbsolutePostion",FrontLeft.GetAbsoluteEncoderRad());
    frc::SmartDashboard::PutNumber("RL_AbsolutePostion", RearLeft.GetAbsoluteEncoderRad());
    frc::SmartDashboard::PutNumber("FR_AbsolutePostion",FrontRight.GetAbsoluteEncoderRad());
    frc::SmartDashboard::PutNumber("RR_AbsolutePostion", RearRight.GetAbsoluteEncoderRad());
    frc::SmartDashboard::PutNumber("FL_DrivePostion",FrontLeft.GetDrivePosition());
    frc::SmartDashboard::PutNumber("RL_DrivePostion", RearLeft.GetDrivePosition());
    frc::SmartDashboard::PutNumber("FR_DrivePostion",FrontRight.GetDrivePosition());
    frc::SmartDashboard::PutNumber("RR_DrivePostion", RearRight.GetDrivePosition());
 
    frc::SmartDashboard::PutNumber("FL_RealPosition",FrontLeft.GetAbsoluteModulePostion());
    frc::SmartDashboard::PutNumber("RL_RealPosition", RearLeft.GetAbsoluteModulePostion());
    frc::SmartDashboard::PutNumber("FR_RealPosition",FrontRight.GetAbsoluteModulePostion());
    frc::SmartDashboard::PutNumber("RR_RealPosition", RearRight.GetAbsoluteModulePostion());


    //Temps and currents
    frc::SmartDashboard::PutNumber("FL_Turning Motor Temp",FrontLeft.GetMotorTurningTemp());
    frc::SmartDashboard::PutNumber("FL_Turning Motor Current",FrontLeft.GetMotorTurnigCurrent());

    frc::SmartDashboard::PutNumber("FR_Turning Motor Temp",FrontRight.GetMotorTurningTemp());
    frc::SmartDashboard::PutNumber("FR_Turning Motor Current",FrontRight.GetMotorTurnigCurrent());

    frc::SmartDashboard::PutNumber("RL_Turning Motor Temp",RearLeft.GetMotorTurningTemp());
    frc::SmartDashboard::PutNumber("RL_Turning Motor Current",RearLeft.GetMotorTurnigCurrent());

    frc::SmartDashboard::PutNumber("RR_Turning Motor Temp",RearRight.GetMotorTurningTemp());
    frc::SmartDashboard::PutNumber("RR_Turning Motor Current",RearRight.GetMotorTurnigCurrent());

    //encoder NEO turning
    
    frc::SmartDashboard::PutNumber("FL_Turning Motor Encoder", FrontLeft.GetTurningPosition());
    frc::SmartDashboard::PutNumber("FR_Turning Motor Encoder", FrontRight.GetTurningPosition());
    frc::SmartDashboard::PutNumber("RL_Turning Motor Encoder", RearLeft.GetTurningPosition());
    frc::SmartDashboard::PutNumber("RR_Turning Motor Encoder", RearRight.GetTurningPosition());




//---------

    frc::SmartDashboard::PutNumber("FL_Drive Motor Temp",FrontLeft.GetMotorDriveTemp());
    frc::SmartDashboard::PutNumber("FL_Drive Motor Current",FrontLeft.GetMotorDriveCurrent());

    frc::SmartDashboard::PutNumber("FR_Drive Motor Temp",FrontRight.GetMotorDriveTemp());
    frc::SmartDashboard::PutNumber("FR_Drive Motor Current",FrontRight.GetMotorDriveCurrent());

    frc::SmartDashboard::PutNumber("RL_Drive Motor Temp",RearLeft.GetMotorDriveTemp());
    frc::SmartDashboard::PutNumber("RL_Drive Motor Current",RearLeft.GetMotorDriveCurrent());

    frc::SmartDashboard::PutNumber("RR_Drive Motor Temp",RearRight.GetMotorDriveTemp());
    frc::SmartDashboard::PutNumber("RR_Drive Motor Current",RearRight.GetMotorDriveCurrent());

    frc::SmartDashboard::PutNumber("Robot Relative Pose X", static_cast<double>(Odometry.GetPose().X()));
     frc::SmartDashboard::PutNumber("Robot Relative Pose Y", static_cast<double>(Odometry.GetPose().Y()));

}

void SwerveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
                           bool fieldRelative, units::meter_t x_center, units::meter_t y_center)
{


frc::Rotation2d Rot2d;

wpi::array<frc::SwerveModuleState, 4> states = DriveConstants::kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetRotation2d() ):
    frc::ChassisSpeeds{xSpeed, ySpeed, rot},
    frc::Translation2d(x_center, y_center));

    SetModulesState(states);

}

void SwerveSubsystem::StopModules(){
    FrontLeft.Stop();
    RearLeft.Stop();
    FrontRight.Stop();
    RearRight.Stop();
}



void SwerveSubsystem::ResetEncoders(){
    FrontLeft.ResetEncoders();
    FrontRight.ResetEncoders();
    RearLeft.ResetEncoders();
    RearRight.ResetEncoders();
}

void SwerveSubsystem::SetModulesState( wpi::array<frc::SwerveModuleState, 4> desiredStates){
DriveConstants::kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, DriveConstants::kMpsPhysicalMaxSpeedMetersPerSecond);
     FrontLeft.SetDesiredState(desiredStates[0]);
     FrontRight.SetDesiredState(desiredStates[1]);
     RearLeft.SetDesiredState(desiredStates[2]);
     RearRight.SetDesiredState(desiredStates[3]);

     
}

void SwerveSubsystem::driveRobotRelative(const frc::ChassisSpeeds& robotRelativeSpeeds){
    frc::ChassisSpeeds targetSpeeds = frc::ChassisSpeeds::Discretize(robotRelativeSpeeds, 0.02_s);

    auto targetStates = DriveConstants::kDriveKinematics.ToSwerveModuleStates(targetSpeeds);
    SetModulesState(targetStates);
}

