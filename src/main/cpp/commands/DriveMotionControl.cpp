// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveMotionControl.h"

DriveMotionControl::DriveMotionControl(DriveTrain *driveTrain, Gyro *gyro,  
                                              units::meter_t targetDistance,
                                              units::meters_per_second_t startVelocity,
                                              units::meters_per_second_t endVelocity,
                                              units::meters_per_second_t maxVelocity,
                                              units::meters_per_second_squared_t maxAcceleration,
                                              units::degree_t targetAngle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({ driveTrain, gyro });

  m_driveTrain = driveTrain;
  m_gyro = gyro;

  m_targetDistance = targetDistance;
  m_startVelocity = startVelocity;
  m_endVelocity = endVelocity;
  m_maxVelocity = maxVelocity;
  m_maxAcceleration = maxAcceleration;

  m_targetAngle = targetAngle;

}

// Called when the command is initially scheduled.
void DriveMotionControl::Initialize() {
  
  m_leftStartingEncoder = m_driveTrain->getLeftEncoder();
  m_rightStartingEncoder = m_driveTrain->getRightEncoder();

}

// Called repeatedly when this Command is scheduled to run
void DriveMotionControl::Execute() {


  
}

// Called once the command ends or is interrupted.
void DriveMotionControl::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveMotionControl::IsFinished() {
  return false;
}
