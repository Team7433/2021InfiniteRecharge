/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <AHRS.h>

#include <units/angle.h>

class Gyro : public frc2::SubsystemBase {
 public:
  Gyro();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  void Periodic();
  void Reset();
  units::degree_t GetYaw();
  units::degree_t GetPitch();
  units::degree_t GetRoll();
  units::degree_t GetClosestError(units::degree_t target);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  AHRS* m_gyro;
};
