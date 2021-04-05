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

  frc::SmartDashboard::PutNumber("TTT/Drive limelight Kp", 0.0);
}

TurnToTarget::TurnToTarget(Gyro* gyro, DriveTrain* drivetrain, double overideAngle) {

  AddRequirements({drivetrain, gyro});
  m_driveTrain = drivetrain;
  m_gyro = gyro;
  m_overideAngle = overideAngle;
  m_overide = true;


}

// Called when the command is initially scheduled.
void TurnToTarget::Initialize() {
  //Checks if there is a target if not ends command
  if (m_overide == false) {
    if (m_vision->getPowerPortDetected() == false) {
      m_done = true;
    }
    //using vision to set a gyro target
    m_gyroTarget = (m_gyro->GetYaw() + m_vision->getPowerPortHorizontalAngle()) - atan(160 / m_vision->getPortDistance()) * (180/kPi);
    frc::SmartDashboard::PutNumber("Gyro target limelight", m_gyroTarget);
    m_lastGyroAngle = m_gyro->GetYaw();
    m_startError = m_gyroTarget - m_gyro->GetYaw();
  } else {
    m_gyroTarget = m_overideAngle;
    m_startError = m_gyroTarget - m_gyro->GetYaw();
  }
  //using vision to set a gyro target

  m_gyroTarget = m_gyro->GetYaw() + m_vision->getPowerPortHorizontalAngle() - units::math::atan(160_mm / m_vision->getPortDistance());
  frc::SmartDashboard::PutString("TTT/Gyro target limelight", units::angle::to_string(m_gyroTarget));


}

// Called repeatedly when this Command is scheduled to run
void TurnToTarget::Execute() { 
  

  //Sets the error
  m_error = ( m_gyroTarget - m_gyro->GetYaw() ).to<double>();
  frc::SmartDashboard::PutNumber("TTT/gyro error", m_error);
 

  //Sets the output to arcade drive.(error x kp)
  frc::SmartDashboard::PutNumber("TTT/output power: ", (m_error * m_kp + 0.1));
  

  // if (fabs(m_gyro->GetYaw() - m_lastGyroAngle) > 0.5) {

  //   m_ks = 0.0;

  // }

  if (fabs(m_error) < fabs(m_startError*0.80)) {

    m_ks = 0.0;

  } else {

  if (m_error > 0) {
    m_ks = 0.065;
  } else {
    m_ks = -0.065;
  }

  }

  if (fabs(m_error) < m_izone) {

    
    m_accumulator += m_ki*m_error;

  }

  double outputPower = (m_error*m_kp) + m_ks + m_accumulator;
  frc::SmartDashboard::PutNumber("TurnToLimelight output power: ", outputPower);
  frc::SmartDashboard::PutNumber("TTT/AnglePerSecond", (m_gyro->GetYaw() - m_lastGyroAngle));
  frc::SmartDashboard::PutNumber("TTT/KS", m_ks);
  frc::SmartDashboard::PutNumber("TTT/Accumululator", m_accumulator);

  m_driveTrain->ArcadeDrive(0, outputPower, false);

  m_lastGyroAngle = m_gyro->GetYaw();

}

// Called once the command ends or is interrupted.
void TurnToTarget::End(bool interrupted) {
  m_driveTrain->ArcadeDrive(0, 0, false);
  m_accumulator = 0.0;
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

  return (fabs(m_error) < 0.5);


  }
