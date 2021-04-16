/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Feeder.h"
#include <frc/smartdashboard/SmartDashboard.h>

Feeder::Feeder() {
    m_feederMotor->ConfigFactoryDefault();
    m_feederMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDslotID, ktimeoutMs);
    m_feederMotor->SetInverted(true);
    m_feederMotor->SetSensorPhase(true);
    m_feederMotor->Config_kF(kPIDslotID, kfeederF, ktimeoutMs);
    configPID(kfeederP, kfeederI, kfeederD, kfeederIzone, kfeederMaxAccumulator);
    m_feederMotor->ConfigMotionAcceleration(4096, ktimeoutMs);
    m_feederMotor->ConfigMotionCruiseVelocity((4096/10*40), ktimeoutMs);
}

// This method will be called once per scheduler run
void Feeder::Periodic() {

    // frc::SmartDashboard::PutNumber("Feeder/currentVelocity", m_feederMotor->GetSelectedSensorVelocity());
    // frc::SmartDashboard::PutNumber("Feeder/Error", m_feederMotor->GetClosedLoopError());

}

void Feeder::SetFeeder(double value) {
    m_feederMotor->Set(ControlMode::PercentOutput, value);
}

void Feeder::SetFeederVelocity(double velocity) {
    m_feederMotor->Set(ControlMode::Velocity, velocity);
}

void Feeder::RunRevolutions(double revolution, double maxAccel, double maxVelo) {
    m_feederMotor->ConfigMotionAcceleration((4096/10)*maxAccel, ktimeoutMs);
    m_feederMotor->ConfigMotionCruiseVelocity((4096/10)*maxVelo, ktimeoutMs);
    m_feederMotor->Set(ControlMode::MotionMagic, m_feederMotor->GetSelectedSensorPosition() + (revolution*4096));

}

