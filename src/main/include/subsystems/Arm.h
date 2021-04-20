/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DoubleSolenoid.h>

#include <ctre/Phoenix.h>

#include <units/length.h>
#include <units/angle.h>

#include "Constants.h"

using namespace ArmConstants;

class Arm : public frc2::SubsystemBase {
 public:
  Arm();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void ManualControl(double Output);
  void SetPosition(double Position);
  void SetAngle(units::degree_t Angle, double);
  void SetAngle(units::angle::armEncoderUnits_t Angle);
  void SetLock(Lock_Position lock_position);
  double GetPosition();
  double GetVelocity();
  double GetMotorOutput();
  double GetError();
  double GetTargetPosition();
  double GetArmAngle();
  double GetArmAngleMotor();
  bool GetArrived();
  units::degree_t GetTargetPositionUnits();
  units::degree_t GetArmAngleUnits();
  units::degree_t GetArmAngleMotorUnits();

  //Method used to do some calculations
  units::degree_t CalculateAngleFromDistance(units::meter_t distance);
  

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  
  WPI_TalonSRX* m_armMotor = new WPI_TalonSRX{kArmMotorID};
  WPI_TalonSRX* m_armEncoder = new WPI_TalonSRX{11};
  frc::DoubleSolenoid m_lockSolenoid{kSolonoidPortAid, kSolonoidPortBid};
  
  
};
