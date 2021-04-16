/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once


/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */         

constexpr double kPi = 3.1415926545897;
constexpr int ktimeoutMs = 10;

#include <ctre/Phoenix.h>
#include <units/time.h>
#include <units/angle.h>
#include <units/length.h>

// #define ButtonBox


namespace AutonmousConstants {

    constexpr units::time::second_t kUnloadMagazineTimeout = 1.0_s;


}



namespace DriverControls {

    constexpr int kMainDriverStickId = 0;
    constexpr int kOperatorControllerId = 1;
    constexpr int kButtonBoxId = 2;
}

namespace DriveTrainConstants {

    constexpr int kLeftDrive1ID = 1;
    constexpr int kRightDrive1ID = 2;
    constexpr int kLeftDrive2ID = 3;
    constexpr int kRightDrive2ID = 4;

    //constexpr double KangleKp = 0.01;

    constexpr int kPIDSlotIdx = 0;


    constexpr units::meter_t kWheelBaseWidth = 700.0_mm;

    constexpr double kF_Profiling = 0.04815;//0.05115
    constexpr double kP_Profiling = 1;//1
    constexpr double kI_Profiling = 0;
    constexpr double kD_Profiling = 0;//20

    constexpr double kUnitsPerRotations = 2048 * 10.71;
    constexpr double kMetersPerRotation = 3.14159265359 * 0.15565;
    constexpr double kMetersPerUnit = kMetersPerRotation / kUnitsPerRotations;

    constexpr double kUnits100msPerMeterSecond = ( 2048 * 10.71 ) / ( 10 * 0.4790928797);


 
    constexpr double kMPStartBuffer = 5;

}

namespace FloorIntakeConstants {

    constexpr int kIntakeMotorId = 10;
    
    constexpr int kSolonoidPortAId = 1;
    constexpr int kSolonoidPortBId = 6;

    enum Position {
        //0 = reverse, 1 = forward
        In = 0, 
        Out = 1
    };

}
  
namespace ShooterConstants {

    constexpr int kShooterAID = 5;
    constexpr int kShooterBID = 6;


    constexpr double kshooterP = 0.0;
    constexpr double kshooterI = 0.0001;
    constexpr double kshooterD = 0.0;
    constexpr double kshooterIZone = 1000;
    constexpr double kshooterMaxAccumulator = 0;

    constexpr double kshooterRampSpeed = 600;

    constexpr ctre::phoenix::motorcontrol::InvertType kbMotorInvert = InvertType::FollowMaster;

}

namespace BallHolderConstants {

    constexpr int kIndexerMotorId = 13;
    constexpr int kMagazineMotorId = 14;

    constexpr int ksensorId_BeltIn = 9;
    constexpr int ksensorId_BeltFull = 8;
    constexpr int ksensorId_OutIndexer = 7;
    constexpr int ksensorId_InIndexer = 6;
    constexpr int ksensorId_beltMiddle = 5;

    constexpr units::time::second_t kTimerMagazineDelay = 0.1_s;
    constexpr units::time::second_t kOverloadReverseLength = 0.2_s;

}       

namespace FeederConstants {


    constexpr int kPIDslotID = 0;
    constexpr double kfeederP = 0.4;
    constexpr double kfeederI = 0.001;
    constexpr double kfeederD = 1.0;
    constexpr double kfeederF = 0.06017647058823529411764705882353;
    constexpr double kfeederIzone = 1000.0;
    constexpr double kfeederMaxAccumulator = 0.0;

    constexpr int kFeederMotorId = 15;
    constexpr int kFeederSolonoidPortAId = 2;
    constexpr int kFeederSolonoidPortBId = 3;

    enum BeltPosition {
        //0 = reverse, 1 = forward
        In = 0, 
        Out = 1
    };

}

namespace ArmConstants {

    constexpr int kArmMotorID = 9;


    constexpr double Kp = 100;
    constexpr double Ki = 0.0;
    constexpr double Kd = 0.0;
    constexpr double Kf = 4.7142;

    constexpr double KmotionCruiseVelocity = 300.0;
    constexpr double KmotionAcceleration = 300.0;

    constexpr int kSolonoidPortAid = 0;
    constexpr int kSolonoidPortBid = 7;

    constexpr double armAngleOffset = 5.53;

    enum Lock_Position {
        //0 = reverse, 1 = forward
        Lock = 0, 
        Unlock = 1
    };


} 

// UNIT_ADD(units::angle, armSensorUnit, armSensorUnits, armUnits, unit<std::ratio<381, 1250>, units::degree>)
namespace units::angle {

    using armEncoderUnits = units::unit<std::ratio<10, 538>, degree, std::ratio<0>, std::ratio<553, 100>>;
    using armEncoderUnits_t = units::unit_t<units::angle::armEncoderUnits>;
    
}

namespace VisionConstants {

    constexpr int klidarPort = 0;
    //Port height
    constexpr units::millimeter_t kheightOfTarget = 2012_mm;

    //Ground to pivot point
    constexpr units::millimeter_t kheightOfRobot = 242.405_mm;
    //Pivot to center of camera
    constexpr units::millimeter_t klengthOfArm = 818.195_mm;
    //Pivot to edge of bumper
    constexpr units::millimeter_t klengthPivotToBumper = 617_mm;

    constexpr units::degree_t kangleOfCamera = 5.47_deg;
    // angle from pivot to camera
    constexpr units::degree_t kangleOffsetCamera = 20.21_deg;
    
    enum LEDState {
        // 0	use the LED Mode set in the current pipeline
        // 1	force off
        // 2	force blink
        // 3	force on

        currentPipeline = 0,
        forceOff = 1,
        forceBlink = 2,
        forceOn = 3

    };
    


}