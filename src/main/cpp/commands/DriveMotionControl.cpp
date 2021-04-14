// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveMotionControl.h"
#include <frc/smartdashboard/SmartDashboard.h>


#define angleAdj
  

DriveMotionControl::DriveMotionControl(DriveTrain *driveTrain, Gyro *gyro,  
                                              units::meter_t targetDistance,
                                              units::meters_per_second_t startVelocity,
                                              units::meters_per_second_t endVelocity,
                                              units::meters_per_second_t maxVelocity,
                                              units::meters_per_second_squared_t maxAcceleration,
                                              std::function<units::degree_t()> angleFunc) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({ driveTrain, gyro });

  m_driveTrain = driveTrain;
  m_gyro = gyro;

  m_targetDistance = targetDistance;
  m_startVelocity = startVelocity;
  m_endVelocity = endVelocity;
  m_maxVelocity = maxVelocity;
  m_maxAcceleration = units::math::fabs(maxAcceleration);

  m_AngleFunc = angleFunc;
  frc::SmartDashboard::PutNumber("DriveMC/AngleFudgeValue", 0.3);

}

// Called when the command is initially scheduled.
void DriveMotionControl::Initialize() {
  std::cout << "DriveMotionControl Init\n";
  m_driveTrain->SetSlot(1);
  m_leftStartingEncoder = m_driveTrain->getLeftDistance();
  m_rightStartingEncoder = m_driveTrain->getRightDistance();

  m_currentVelocity = m_startVelocity;
  m_currentDistance = 0.0_m;
  m_targetAngle = m_AngleFunc();
  
  if ( ( ( units::math::pow<2>(m_endVelocity) - units::math::pow<2>(m_startVelocity) ) / ( 2 * m_targetDistance ) ) > m_maxAcceleration ) {
    Cancel();
  }

}

// Called repeatedly when this Command is scheduled to run
void DriveMotionControl::Execute() {

  m_currentDistance = ( ( m_driveTrain->getLeftDistance() - m_leftStartingEncoder ) + ( m_driveTrain->getRightDistance() - m_rightStartingEncoder ) ) / 2;

  units::meters_per_second_t newVelocity = 0.0_mps;

  if (units::math::fabs(m_targetDistance) - units::math::fabs(m_currentDistance) < 1_mm) {
    m_driveTrain->setVelocity(m_endVelocity, m_endVelocity);
    return;
  }

  if (m_maxVelocity > 0.0_mps) {
    newVelocity = units::math::min(
      m_maxVelocity,
      units::math::min(
        m_currentVelocity + units::meters_per_second_t( m_maxAcceleration * 20_ms),
        units::math::sqrt(units::math::pow<2>(m_endVelocity) + ( 2 * m_maxAcceleration * ( m_targetDistance - m_currentDistance ) ) )
      )
    );
  } else {
    newVelocity = units::math::max(
      m_maxVelocity, //This is negative
      units::math::max(
        m_currentVelocity + units::meters_per_second_t( -m_maxAcceleration * 20_ms),
        -units::math::sqrt(units::math::pow<2>(m_endVelocity) + ( 2 * m_maxAcceleration * ( units::math::fabs(m_targetDistance) - units::math::fabs(m_currentDistance) ) ) )
      )
    );
  }

  #ifdef angleAdj
  units::radian_t difference = m_targetAngle - m_gyro->GetYaw();
  double fudgeValue = frc::SmartDashboard::GetNumber("DriveMC/AngleFudgeValue", 0.1);
  frc::SmartDashboard::PutNumber("DriveMC/Difference", difference.convert<units::degree>().to<double>());
  units::meters_per_second_t leftVelocity = newVelocity + units::meters_per_second_t( ( (DriveTrainConstants::kWheelBaseWidth / 2) / 20_ms ) * difference.to<double>() * fudgeValue );
  units::meters_per_second_t rightVelocity = newVelocity - units::meters_per_second_t( ( (DriveTrainConstants::kWheelBaseWidth / 2) / 20_ms ) * difference.to<double>() * fudgeValue );
  m_driveTrain->setVelocity(leftVelocity, rightVelocity);
  #else
  std::cout << "VelocityVel: " << units::velocity::to_string(newVelocity) << "\n";
  m_driveTrain->setVelocity(newVelocity, newVelocity);
  
  #endif
  m_currentVelocity = newVelocity;
}

// Called once the command ends or is interrupted.
void DriveMotionControl::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveMotionControl::IsFinished() {
  return units::math::fabs(m_targetDistance) - units::math::fabs(m_currentDistance) < 1_cm;
}


units::meters_per_second_squared_t DriveMotionControl::getAccelaration(units::meters_per_second_t startVel, units::meters_per_second_t endVel, units::meter_t distance) {
  return ( ( units::math::pow<2>(endVel) - units::math::pow<2>(startVel) ) / ( 2 * distance ) );
}


DriveMotionControl::DriveMotionControl(DriveTrain *driveTrain, Gyro *gyro,  
                                              units::meter_t targetDistance,
                                              units::meters_per_second_t startVelocity,
                                              units::meters_per_second_t endVelocity,
                                              units::meters_per_second_t maxVelocity,
                                              units::meters_per_second_squared_t maxAcceleration,
                                              units::degree_t targetAngle) : DriveMotionControl(driveTrain, gyro, targetDistance, startVelocity, endVelocity, maxVelocity, maxAcceleration, [targetAngle] { return targetAngle; } ) {}