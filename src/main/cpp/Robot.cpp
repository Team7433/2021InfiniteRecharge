/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
  // frc::CameraServer::GetInstance()->StartAutomaticCapture();
  m_container.CoastMode(); // Set DriveTrain to Coast Mode
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  m_container.zeroOutputDisabled();
  m_container.CoastMode(); // Set Drivetrain to Coast Mode

}

void Robot::DisabledPeriodic() {

  if (m_container.GetArmAngle() < 54_deg) {
    m_container.SetLimelightLED(VisionConstants::LEDState::currentPipeline);
    if(m_container.GetVisionSubsystem().getPowerPortDetected() == false) {m_container.ControlLight(140, 0, 128);} 
      else {
      if(0.5 > m_container.GetTargetError().to<double>() && m_container.GetTargetError().to<double>() > -0.5 ) {m_container.ControlLight(0, 255, 0);}
      else if (m_container.GetTargetError().to<double>() > 0.5) {m_container.ControlLight(0, 0, 255); }
      else if (m_container.GetTargetError().to<double>() < -0.5) {m_container.ControlLight(255, 0, 0);}
      }
  } else {m_container.SetLimelightLED(VisionConstants::LEDState::forceOff); m_container.ControlLight(0, 0, 0);}
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();
  m_container.BrakeMode(); // Set Drivetrain to Brake Mode
  m_container.SetLimelightLED(VisionConstants::LEDState::currentPipeline);

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  m_container.SetLimelightLED(VisionConstants::LEDState::currentPipeline);
  m_container.BrakeMode(); // Set Drivetrain to Brake Mode
  
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
  m_container.ResetStartOfTeleop();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  // frc::SmartDashboard::PutNumber("MatchTime", m_timer.GetMatchTime());
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
