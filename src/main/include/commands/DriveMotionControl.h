// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveTrain.h"
#include "subsystems/Gyro.h"

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/math.h>
#include <units/time.h>

#include "Constants.h"

class DriveMotionControl
    : public frc2::CommandHelper<frc2::CommandBase, DriveMotionControl> {
  public:
    DriveMotionControl(DriveTrain *, Gyro *,  units::meter_t targetDistance,
                                              units::meters_per_second_t startVelocity,
                                              units::meters_per_second_t endVelocity,
                                              units::meters_per_second_t maxVelocity,
                                              units::meters_per_second_squared_t maxAcceleration,
                                              units::degree_t targetAngle);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
  private:
  
    Gyro* m_gyro;
    DriveTrain* m_driveTrain;
  
    units::meter_t m_targetDistance;
    units::meters_per_second_t  m_startVelocity;
    units::meters_per_second_t  m_endVelocity;
    units::meters_per_second_t  m_maxVelocity;
    units::meters_per_second_squared_t  m_maxAcceleration;
    units::degree_t m_targetAngle;

    units::meter_t m_leftStartingEncoder; 
    units::meter_t m_rightStartingEncoder;

    units::meter_t m_currentDistance = 0.0_m;
    units::meters_per_second_t m_currentVelocity = 0.0_mps;

    units::meters_per_second_squared_t getAccelaration(units::meters_per_second_t startVel, units::meters_per_second_t endVel, units::meter_t distance);
    
  
};
