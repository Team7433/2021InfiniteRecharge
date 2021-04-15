// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoTarget.h"
#include "units/math.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
AutoTarget::AutoTarget(std::function<units::meter_t()> distanceM, Arm* arm, Shooter* shooter, bool update, bool VelocityCompensation, DriveTrain* driveTrain) {

  //setting lambda function using line of best fit equation to figure out the perfect angle and velocity for shooter/arm for the perfect shot
    m_angle = [this, distanceM] {return 15.5504_deg + (units::degree_t(130.439 / (2.46224 + distanceM().to<double>()))); };
    if (VelocityCompensation) {m_velocity = [this, distanceM, driveTrain] {return (10648.9 + 1447.44 * distanceM().to<double>()) + ((driveTrain->getLeftVelocity() + driveTrain->getRightVelocity())/2).to<double>()*250; };}
    else {m_velocity = [this, distanceM] {return 10648.9 + 1447.44 * distanceM().to<double>(); };}

    AddCommands(
      frc2::InstantCommand(
        [this, distanceM] {
          //logging this on smartdashboard

          // frc::SmartDashboard::PutNumber("AutoTarget/TargetSpeed", m_velocity());
          // frc::SmartDashboard::PutString("AutoTarget/TargetArmAngle", units::angle::to_string(m_angle()));

        }
      ), // Instand Command

      frc2::ParallelCommandGroup(
        //setting arm angle and shooter velocity
        SetArmAngle(arm, m_angle, update),
        RunShooter(shooter, m_velocity, update)
      ) //Parallel Command Group

    ); // Add Commands
}


AutoTarget::AutoTarget(std::function<units::meter_t()> distanceM, std::function<units::degree_t()> gyroTarget, Arm* arm, Shooter* shooter, Gyro* gyro, DriveTrain* drivetrain) {

  m_angle = [this, distanceM] {return 15.5504_deg + (units::degree_t(130.439 / (2.46224 + distanceM().to<double>()))); };
  m_velocity = [this, distanceM] {return 10648.9 + 1447.44 * distanceM().to<double>(); };


  AddCommands(
    frc2::InstantCommand(
        [this, distanceM] {
          //logging this on smartdashboard
          // frc::SmartDashboard::PutNumber("AutoTarget/TargetSpeed", m_velocity());
          // frc::SmartDashboard::PutString("AutoTarget/TargetArmAngle", units::angle::to_string(m_angle()));
        }
      ), // Instand Command

      frc2::ParallelCommandGroup(
        //setting arm angle and shooter velocity
        SetArmAngle(arm, m_angle),
        RunShooter(shooter, m_velocity),
        TurnToTarget(gyro, drivetrain, gyroTarget) 
      ) //Parallel Command Group


  ); //Add Commands

}

// AutoTarget::AutoTarget(std::function<units::meter_t()> distanceM, std::function<units::degree_t()> gyroTarget, std::function<double()> forwardOutput,Arm* arm, Shooter* shooter, Gyro* gyro, DriveTrain* drivetrain) {

//   m_angle = [this, distanceM] {return 15.5504_deg + (units::degree_t(130.439 / (2.46224 + distanceM().to<double>()))); };
//   m_velocity = [this, distanceM] {return 10648.9 + 1447.44 * distanceM().to<double>(); };

//   AddCommands(
//     frc2::InstantCommand(
//         [this, distanceM] {
//           //logging this on smartdashboard
//           frc::SmartDashboard::PutNumber("AutoTarget/TargetSpeed", m_velocity());
//           frc::SmartDashboard::PutString("AutoTarget/TargetArmAngle", units::angle::to_string(m_angle()));
//         }
//       ), // Instand Command

//       frc2::ParallelCommandGroup(
//         //setting arm angle and shooter velocity
//         SetArmAngle(arm, m_angle),
//         RunShooter(shooter, m_velocity),
//         GyroDrive(gyro, drivetrain, gyroTarget, forwardOutput)
//       ) //Parallel Command Group


//   ); //Add Commands

// }


//overides for arm and shooter without turn to target or gyro drive
AutoTarget::AutoTarget(units::meter_t DistanceM, Arm* arm, Shooter* shooter) : AutoTarget([DistanceM] { return DistanceM; }, arm, shooter, false) {}
AutoTarget::AutoTarget(Vision* vision, Arm* arm, Shooter* shooter, bool update) : AutoTarget([vision] { return vision->getPortDistance(); }, arm, shooter, update) {}

//overide for turn to target
AutoTarget::AutoTarget(Vision* vision, Arm* arm, Shooter* shooter, Gyro* gyro, DriveTrain* drivetrain) : AutoTarget([vision] {std::cout <<"autoTarget being called\n"; return vision->getPortDistance();}, [vision, gyro] { ; return units::degree_t(gyro->GetYaw() + vision->getPowerPortHorizontalAngle() - units::math::atan(160_mm / vision->getPortDistance()));}, arm, shooter, gyro, drivetrain) {}

//overide for gyrodrive
// AutoTarget::AutoTarget(std::function<double()> forwardOutput, Vision* vision, Arm* arm, Shooter* shooter, Gyro* gyro, DriveTrain* drivetrain) : AutoTarget([vision] {return vision->getPortDistance();}, [vision, gyro] {return gyro->GetYaw() + vision->getPowerPortHorizontalAngle() - units::math::atan(160_mm / vision->getPortDistance());}, forwardOutput, arm, shooter, gyro, drivetrain) {}
  
