// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "string"
#include <units/angle.h>
#include <units/length.h>
#include <numbers>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/acceleration.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

#include <frc/trajectory/TrapezoidProfile.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

//Path Library
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>

using namespace units;
using namespace units::angle;
using namespace units::length;
/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace BooleansConsts{
   constexpr bool kFrontLeftTurningEncoderReversed = true;
   constexpr bool kBackLeftTurningEncoderReversed = true;
   constexpr bool kFrontRightTurningEncoderReversed = true;
   constexpr bool kBackRightTurningEncoderReversed = true;

   constexpr bool kFrontLeftDriveEncoderReversed = true;
   constexpr bool kBackLeftDriveEncoderReversed = true;
   constexpr bool kFrontRightDriveEncoderReversed = true;
   constexpr bool kBackRightDriveEncoderReversed = false;

   constexpr bool kFrontLeftAbsoluteEncoderReversed = false;
   constexpr bool kBackLeftAbsoluteEncoderReversed = false;
   constexpr bool kFrontRightAbsoluteEncoderReversed = false;
   constexpr bool kBackRightAbsoluteEncoderReversed = false;

   //Top Subsystems Set Inverted conditions

   //SetInverted boolean to apply in the Elevators Motors
   constexpr bool rightElevatorMotorReversed = false;
   constexpr bool lefttElevatorMotorReversed = false;

  //SetInverted boolean to apply in the Intake Motors
   constexpr bool rightIntakeMotorReversed = false;
   constexpr bool leftIntakeMotorReversed  = false;

   //SetInverted boolean to apply in the Flip motor
   constexpr bool flipMotorReversed = false;

   //SetInverted boolean to apply in the GiraffeMotor;
   constexpr bool GiraffeMotorReversed = false;

   //SetInverted boolean to apply in the right shooter Motor;
   constexpr bool RightShooterMotorReversed = false;
   constexpr bool LeftShooterMotorReversed = false;

   //SetInverted con
}

namespace DriveConstants {

  constexpr int kFrontRightDriveMotor = 12;
  constexpr int kFrontLeftDriveMotor = 13;
  constexpr int kBackRightDriveMotor = 17;
  constexpr int kBackLeftDriveMotor = 16;

  constexpr int kFrontRightTurningMotor = 11;
  constexpr int kFrontLeftTurningMotor = 14;
  constexpr int kBackRightTurningMotor = 18;
  constexpr int kBackLeftTurningMotor = 15;

    //Max angle and drive acceleration for Teleoperated
  constexpr double kTeleDriveMaxAccelerationUnitsPerSecond = 14.0;
  constexpr double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 6.0; 

   //The variable to put in SetDesiredState in SwerveSubsystem.cpp in mps
  constexpr  auto  kMpsPhysicalMaxSpeedMetersPerSecond = 20_mps;


  //The Variable to put in limiters in SwerveCommand.cpp, in double
  constexpr  double  kDoublePhysicalMaxSpeedMetersPerSecond = 1.0;

  constexpr double kTeleDriveMaxSpeedMetersPerSecond = kDoublePhysicalMaxSpeedMetersPerSecond;

  //Perfect one 
  // constexpr double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2.5 * std::numbers::pi;
    constexpr double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 1.5 * std::numbers::pi;
  constexpr double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
  kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

  //Translation and kinematics object 

    constexpr units::meter_t kTrackWidth =
      0.7_m;  // Distance between centers of right and left wheels on robot
    constexpr units::meter_t kWheelBase =
      0.7_m;  // Distance between centers of front and back wheels on robot

    
     static frc::SwerveDriveKinematics<4>  kDriveKinematics{
    frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{-kWheelBase/ 2, -kTrackWidth / 2}};

    constexpr double SlowYaxisSpeed = 0.2;

    inline constexpr units::meters_per_second_t maxModuleSpeed = 4.5_mps;
  
   //Path Config Holonomic 
   inline constexpr pathplanner::HolonomicPathFollowerConfig pathFollowerConfig = pathplanner::HolonomicPathFollowerConfig(
    pathplanner::PIDConstants(0.0005, 0.0, 0.0), // Translation constants 
    pathplanner::PIDConstants(0.0005, 0.0, 0.0), // Rotation constants 
    maxModuleSpeed,
    0.57_m, // Drive base radius (distance from center to furthest module) 
    pathplanner::ReplanningConfig()
);


}//namespace DriveConstants

namespace CanSensorConstants{

  //Swerve Drive CanCoders
  constexpr int kFrontRightTurningSensor = 33;
  constexpr int kFrontLeftTurningSensor = 34;
  constexpr int kBackRightTurningSensor = 31;
  constexpr int kBackLeftTurningSensor = 32;

  //Flip CanCoder ID
  constexpr int flipCanCoderID = 35;

  //Giraffe CanCoder ID
  constexpr int GiraffeCanCoderID = 36;

}//namespace CansSensorConstants

namespace OperatorConstants
{
    
    constexpr int kDriverControllerPort = 0;
    constexpr int kTopControllerPort = 1;
    constexpr int kManualTestControllerPort = 2;

} // namespace OperatorConstants

namespace RLGear
{
    // angle gear relation set
    constexpr double ANGLE_GEAR_RATIO = 12.8 / 1.0;
    // drive gear relation set
    constexpr double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
}

namespace DefAngModule
{
    /* Module OFFSET Specific Constants */
        /* Front Left Module */
    constexpr double FL_DEGREE_OFFSET = 2.54027;//5.17257

    /* Rear Left Module */
    constexpr double RL_DEGREE_OFFSET = 0.49700;//2.88387

    /* Front Right Module */
    constexpr double FR_DEGREE_OFFSET = 1.28854;//5.23699

    /* Back Right Module */
    constexpr double RR_DEGREE_OFFSET = 2.78724;//1.00322
}



namespace PIDModConstants
{

    constexpr double kPTurning = 0.3;
    constexpr double kITurning = 0.0; //0.0002;
    constexpr double kDTurning = 0.0;  //0.000005;
    constexpr double kPModuleDriveController = 0.01;

}

namespace ConstantsMod
{

    constexpr double kWheelDiameterMeters = 0.1; 
    constexpr double kWheelRadiusInches = 2;
    constexpr double kWheelRadiusMeters = 0.05;
    constexpr double kDriveMotorGearRatio = 1 / 6.12;
    constexpr double kTurningMotorGearRatio = 1 / 18.0;
    constexpr double kDriveEncoderRot2Meter = kDriveMotorGearRatio * std::numbers::pi * kWheelDiameterMeters;
    constexpr double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * std::numbers::pi;
    constexpr double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    constexpr double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    
} // namespace

namespace OIconstants {
    constexpr  int kDriverYAxis = 1;
    constexpr  int kDriverXAxis = 0;
    constexpr  int kDriverRotAxis = 4;
    constexpr  int kDriverFieldOrientedButtonIdx = 1;
    constexpr  double kDeadband = 0.09;
}//namepsace OIconstants


namespace CenterConstants {
  constexpr meter_t centerx = 0.715_m / 2;
  constexpr meter_t centery = 0.715_m / 2;
}

namespace AutoConstants {
constexpr auto kMaxSpeed = 0.5_mps;
constexpr auto kMaxAcceleration = 0.5_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.8;
constexpr double kPYController = 0.8;
constexpr double kPThetaController = 0.05;


extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace PigeonConstants
{

 constexpr int  PigeonId = 1;

}//PigeonConstants


 //Top Subsystems Constants//

  //All Intake Constants
namespace IntakeConstants {

  //All the intake's Driver Id's 
  constexpr int leftIntakeMotorId =  27;
  constexpr int rightIntakeMotorId = 28;
  
  //The Intake (NEO550) temperature limit
  constexpr double IntakeMotor550Temp = 48;

  //The Intake (NEO550) current limit
  constexpr double IntakeMotor550Amp = 1.8;
  
  //All Intake speeds

  constexpr double rightCatchIntakeSpeed = -0.5; //0.35
  constexpr double leftCatchIntakeSpeed = 0.5;

  constexpr double rightSpitIntakeSpeed = 0.15;
  constexpr double leftSpitIntakeSpeed = 0.60;


}

  //All ShooterConstants
namespace ShooterConstants {

  //All the shooter's Driver Id's

  constexpr int rightShooterMotorId = 21;
  constexpr int leftShooterMotorId = 22;

  //All the shooter speeds to be used
  constexpr double LaunchShooterSpeed = 0.40;
  constexpr double CatchShooterSpeed = -0.70;

  
  //All the two differents speeds to set in Launch Amp Velocity
  constexpr double LaunchAmp_RightVelocity = -3.5;
  constexpr double LaunghAmp_LeftVelocity = -13;
}

  //All Elevator Constants
namespace ElevatorConstants {

  //All the elevator Id's

  constexpr int leftElevatorMotorId = 23;
  constexpr int rightElevatorMotorId = 24;
  

  //High and Low position constants

  //DON'T USE THIS PID SETPOINT, SEE IN THE SHUFFLE THE CORRECT VALUE
  constexpr double ChainElevatorPositionLevel = 70;

  //DON'T USE THIS PID SETPOINT, SEE IN THE SHUFFLE THE CORRECT VALUE 
  constexpr double MinElevatorPosition = 0;

  //The Test Potency to be allocated in the test  elevator  method ,
  //as the default potency.
  constexpr double ElevatorPot = 0.2; 

  //The elevator current limit
  constexpr double ElevatorMotorCurrent = 15.0;

  //The elevator temperature limit
  constexpr double ElevatorMotorTemp = 45;


}

  //All Cover Constants
namespace FlipConstants {

  //All the cover Id's
  constexpr int flipMotorId = 25;

  //DON'T USE THIS PID SETPOINT, SEE IN THE SHUFFLE THE CORRECT VALUE
  constexpr double AmpFlipPositionLevel = 10.0;

  //DON'T USE THIS PID SETPOINT, SEE IN THE SHUFFLE THE CORRECT VALUE
  constexpr double DefaultflipPosition = 4.0;


  //The Test Potency to be allocated in the test flip  method ,
  //as the default potency.
  constexpr double TestFlipPot = 0.15; 
}


namespace GiraffeConstants {

  //All the Giraffes drivers Id's
  constexpr int GiraffeMotorId = 26;

  //The giraffe current limit
  constexpr double GiraffeMotorCurrent = 35.0;

  //The giraffe temperature limit
  constexpr double GiraffeMotorTemp = 55;

  //this constants  must to be changed, this values
  //are pure ilustrative

  //Dafault angles for arena

  //DON'T USE THIS PID SETPOINT, SEE IN THE SHUFFLE THE CORRECT VALUE
  constexpr double AmpAngle = 233.0;

  //DON'T USE THIS PID SETPOINT, SEE IN THE SHUFFLE THE CORRECT VALUE
  constexpr double SourceAngle = 220.0;

  //The default position with no AprilTags
  constexpr double DefaultAngle = 220.0;

  //The default position for battery
  constexpr double BatAngle = 232.0;

  // Max and Min angle
  constexpr double GiraffeAngMax = 234;
  constexpr double GiraffeAngMin = 214;

  //The Test Potency to be allocated in the test giraffe  method ,
  //as the default potency.
  constexpr double TestGiraffePot = 0.05; 

  //x^2, x^1, x^0
  constexpr double x_quad = -0.0033;
  constexpr double x_um = 0.6003;
  constexpr double x_zero = 220.6975;
  
  constexpr double MaxPIDErrorOut = 0.005;

}


namespace UltrasonicConstants
{
 constexpr int UltrasonicChannel = 1;
 constexpr double UltrasonicMaxRange = 100;
 constexpr double UltrasonicOffSet = 0;

}//namepsace UltrasonicConstants

namespace LimeConstants {

 //All the Highs between the source,Amp and speaker the floor(METERS)
 constexpr double BackSourceHigh_toFloor = 1.22;
 constexpr double BackAmpHigh_toFloor = 1.22;
 constexpr double BackSpeakerHigh_toFloor = 1.32;

 //All the horizontal default angles for each structure
 constexpr double BackTxSource = -0.16;
 constexpr double BackTxAmp = -2.46;

 //All the defaults distances in Y axis for each structures
 constexpr double DistSource_Yaxis = 1.38;

 //All the default angle setpoints for Auto aligment for each structures
 constexpr double defaultSourceHeading = 220.14;
 constexpr double BlueAmpHeading = 90.00;
 constexpr double RedAmpHeading = 270.00;

}

