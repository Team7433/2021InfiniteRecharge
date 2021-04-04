// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoTarget.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
AutoTarget::AutoTarget(std::function<units::meter_t()> distanceM, Arm* arm, Shooter* shooter) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
    AddCommands(
      frc2::InstantCommand(
        [this, distanceM] {
          //using line of best fit equation to figure out the perfect angle and velocity for shooter/arm for the perfect shot
          m_angle = [this, distanceM] {return 15.5504_deg + (units::degree_t(130.439 / (2.46224 + atof(units::length::to_string(distanceM()).c_str())))); };
          m_velocity = [this, distanceM] {return 10648.9 + 1447.44 * atof(units::length::to_string(distanceM()).c_str()); };
          //logging this on smartdashboard
          frc::SmartDashboard::PutNumber("AutoTarget/TargetSpeed", m_velocity());
          frc::SmartDashboard::PutString("AutoTarget/TargetAngle", units::angle::to_string(m_angle()));
        }
      ),

      frc2::ParallelCommandGroup(
        //setting arm angle and shooter velocity
        SetArmAngle(arm, m_angle),
        RunShooter(shooter, m_velocity)
      )

    );
}

AutoTarget::AutoTarget(units::meter_t DistanceM, Arm* arm, Shooter* shooter) : AutoTarget([DistanceM] { return DistanceM; }, arm, shooter) {}
AutoTarget::AutoTarget(Vision* vision, Arm* arm, Shooter* shooter) : AutoTarget([vision] { return vision->getPortDistance(); }, arm, shooter) {}