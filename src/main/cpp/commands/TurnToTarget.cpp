/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnToTarget.h"
#include <cmath>
#include "units/math.h"
#include "units/time.h"


TurnToTarget::TurnToTarget(Vision* vision, Gyro* gyro, DriveTrain* drivetrain) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({vision, drivetrain, gyro});
  
  m_vision = vision;
  m_driveTrain = drivetrain;
  m_gyro = gyro;

}

TurnToTarget::TurnToTarget(Gyro* gyro, DriveTrain* drivetrain, units::degree_t overideAngle, bool end) : TurnToTarget(gyro, drivetrain, [overideAngle] {return overideAngle;}, end) {}


TurnToTarget::TurnToTarget(Gyro* gyro, DriveTrain* drivetrain, std::function<units::degree_t()> overideAngle, bool end) {

  AddRequirements({drivetrain, gyro});
  m_driveTrain = drivetrain;
  m_gyro = gyro;
  m_overideAngle = [overideAngle] {return overideAngle();};
  m_overide = true;
  m_end = end;

}
                                           
// Called when the command is initially scheduled.
void TurnToTarget::Initialize() {
  //Checks if there is a target if not ends command
  if (m_overide == false) {
    if (m_vision->getPowerPortDetected() == false) {
      std::cout << "Power Port not detected" << std::endl;
      m_done = true;
    }
    //using vision to set a gyro target
    m_gyroTarget = m_gyro->GetYaw() + m_vision->getPowerPortHorizontalAngle() - units::math::atan(160_mm / m_vision->getPortDistance());
    m_lastGyroAngle = m_gyro->GetYaw();
    m_startError = m_gyroTarget - m_gyro->GetYaw();
  } else {
    m_gyroTarget = m_overideAngle();
    m_startError = m_gyroTarget - m_gyro->GetYaw();
  }

  frc::SmartDashboard::PutString("TTT/Gyro target", units::angle::to_string(m_gyroTarget));
  std::cout << "Target Angle: " << units::angle::to_string(m_gyroTarget) << std::endl;


}

// Called repeatedly when this Command is scheduled to run
void TurnToTarget::Execute() { 
  

  //Sets the error
  // m_error = (m_gyroTarget - m_gyro->GetYaw());
  m_error = m_gyro->GetClosestError(m_gyroTarget);
  
  // if (fabs(m_gyro->GetYaw() - m_lastGyroAngle) > 0.5) {

  //   m_ks = 0.0;

  // }

  if (units::math::fabs(m_error) < units::math::fabs(m_startError*0.80)) {

    m_ks = 0.0;

  } else {

  if (m_error > 0_deg) {
    m_ks = 0.065;
  } else {
    m_ks = -0.065;
  }

  }

  if (units::math::fabs(m_error) < m_izone) {

    m_accumulator += m_ki*m_error.to<double>();

  }

  double outputPower = (m_error.to<double>()*m_kp) + m_ks + m_accumulator;
  frc::SmartDashboard::PutNumber("TTT/output power: ", outputPower);
  frc::SmartDashboard::PutNumber("TTT/AnglePerSecond", (m_gyro->GetYaw() - m_lastGyroAngle).to<double>());
  frc::SmartDashboard::PutNumber("TTT/KS", m_ks);
  frc::SmartDashboard::PutNumber("TTT/Accumululator", m_accumulator);
  frc::SmartDashboard::PutNumber("TTT/gyro error", m_error.to<double>());


  m_driveTrain->ArcadeDrive(0, outputPower, false);

  m_lastGyroAngle = m_gyro->GetYaw();

}

// Called once the command ends or is interrupted.
void TurnToTarget::End(bool interrupted) {
  m_driveTrain->ArcadeDrive(0, 0, false);
  m_accumulator = 0.0;
  m_counter = 0;
  m_done = false;
  m_timer.Stop();
  m_timer.Reset();

  
                                                   


}

// Returns true when the command should end.
bool TurnToTarget::IsFinished() { 
  // if(m_vision->getPowerPortDetected() == false) {
  //   return true;
  // }
  // return false;

  // if (fabs(m_error) < 1.0) {

  //   m_timer.Start();

  // } else {
  //   m_timer.Stop();
  //   m_timer.Reset();

  // }

  // return (m_timer.Get() > 0.5_s);
  if (m_end) {
    if (units::math::fabs(m_error) < 0.5_deg) {
      m_counter++;
    } else {
      m_counter = 0;
    }
    if (m_counter >= 3 ) {return true;}
  }
    return false;
  // return (units::math::fabs(m_error) < 0.5_deg);


  }
