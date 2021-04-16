/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Feeder.h"

Feeder::Feeder() {
    m_feederMotor->ConfigFactoryDefault();

    m_feederMotor->SetInverted(true);
    configPID(kfeederP, kfeederI, kfeederD, kfeederIzone, kfeederMaxAccumulator);
}

// This method will be called once per scheduler run
void Feeder::Periodic() {

    std::cout << m_feederMotor->GetSelectedSensorVelocity() << std::endl;

}

void Feeder::SetFeeder(double value) {
    m_feederMotor->Set(ControlMode::PercentOutput, value);
}

void Feeder::SetFeederVelocity(double velocity) {

    m_feederMotor->Set(ControlMode::Velocity, velocity);

}

