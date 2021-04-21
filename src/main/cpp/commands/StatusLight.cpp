// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/StatusLight.h"

StatusLight::StatusLight(RGBStrip* rgbStrip, std::function<units::degree_t()> targetAngle, std::function<units::degree_t()> currentAngle, std::function<bool()> targetDeteced, std::function<double()> targetVelocity) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(rgbStrip);

  m_RGBStrip = rgbStrip;
  m_targetAngle = targetAngle;
  m_currentAngle = currentAngle;
  m_targetDetected = targetDeteced;
  m_targetVelocity = targetVelocity;

}

// Called when the command is initially scheduled.
void StatusLight::Initialize() {

  frc::SmartDashboard::PutBoolean("strip/RainbowMode", false);

}

// Called repeatedly when this Command is scheduled to run
void StatusLight::Execute() {



  if(!frc::SmartDashboard::GetBoolean("strip/RainbowMode", false)) {
    if (units::math::fabs(m_currentAngle() - m_targetAngle()) < 1_deg) {
      if (m_targetDetected() && m_currentAngle() && m_targetVelocity() < 500) {
        m_RGBStrip->SetRGBStrip(0, 0, 255);
      } else if (!m_targetDetected() && m_currentAngle() && m_targetVelocity() < 500) {
        m_RGBStrip->SetRGBStrip(140, 0, 128);
      } else { m_RGBStrip->SetRGBStrip(0, 255, 0); }

    } else {
      m_RGBStrip->SetRGBStrip(255, 0, 0);
    }
  } else {

    srand (time(NULL));
    double randomR = (rand() % 9 + 1);
    srand (time(NULL)-20);
    double randomG = (rand() % 9 + 1);
    srand (time(NULL)-40);
    double randomB = (rand() % 9 + 1);
    
    randomR = randomR/1000;
    randomG = randomG/1000;
    // randomR = randomR/1000;
    
    if (rand() %3 == 1 && m_countR > rand() %10000 + 10000) {

        m_countR = 0;
        if (m_signR == -1) {
            m_signR = 1;
        } else {m_signR = -1;}

    } else {m_countR++;}
    if(m_RGBStrip->GetR() + (randomR*m_signR) > 255) {
        m_signR = -1;
    } else if (m_RGBStrip->GetR() + (randomR*m_signR) < 0) {
        m_signR = +1;
        }

    m_RGBStrip->SetRGBStrip(m_RGBStrip->GetR() + (randomR*m_signR), 0, 0);


  }

}

// Called once the command ends or is interrupted.
void StatusLight::End(bool interrupted) {}

// Returns true when the command should end.
bool StatusLight::IsFinished() {
  return false;
}
