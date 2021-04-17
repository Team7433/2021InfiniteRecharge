// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/Phoenix.h>

#include "Constants.h"

using namespace ClimberConstants;

class Climber : public frc2::SubsystemBase {
 public:
  Climber();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  ClimberLock_Position GetLockPosition() { if (m_climbSolenoid.Get() == frc::DoubleSolenoid::Value::kForward) {return ClimberLock_Position::Lock;} else {return ClimberLock_Position::Unlock;} }
  void SetLockPosition(ClimberLock_Position lockPosition) { if(lockPosition == ClimberLock_Position::Lock) {m_climbSolenoid.Set(frc::DoubleSolenoid::Value::kForward);} else {m_climbSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);} }
  void SetTarget(double targetEncoderCount, double maxAccel, double maxVel, double kF = 0.0);
 
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::DoubleSolenoid m_climbSolenoid{kClimberSolenoidPortAID, kClimberSolenoidPortBID};

  TalonFX* m_masterMotor = new TalonFX{kMasterMotorID};
  TalonFX* m_slaveMotor = new TalonFX{kSlaveMotorID};
};
