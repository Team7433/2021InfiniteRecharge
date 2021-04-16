/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter() {
    m_shooterA->ConfigFactoryDefault();
    m_shooterB->ConfigFactoryDefault();

    m_shooterA->SetInverted(true);
    m_shooterB->Follow(*m_shooterA);
    m_shooterB->SetInverted(kbMotorInvert);
    m_shooterA->Config_kF(0, 0.05, ktimeoutMs);

    configPID(kshooterP, kshooterI, kshooterD, kshooterIZone, kshooterMaxAccumulator);
    

    m_shooterA->ConfigVoltageCompSaturation(11.5);
    m_shooterA->EnableVoltageCompensation(true);
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
    // frc::SmartDashboard::PutNumber("I Accumulator", GetIAccumulator());
    // frc::SmartDashboard::PutNumber("Percentage Output", GetPercentageOutput());
    // frc::SmartDashboard::PutNumber("Target velocity", GetVelocityLoopTarget());
    // frc::SmartDashboard::PutNumber("Shooter/Velocity", GetVelocity());
    // frc::SmartDashboard::PutNumber("")


}

void Shooter::ManualControl(double Output) {
    m_shooterA->Set(ControlMode::PercentOutput, Output);
}

void Shooter::SetVelocity(double Velocity) {

    m_shooterA->Set(ControlMode::Velocity, int(Velocity));
    // std::cout << "SetVelcoity: " << Velocity << "\n";

}

void Shooter::configPID(double P, double I, double D, double Izone, double MaxAccumulator) {

    m_shooterA->Config_kP(0, P, ktimeoutMs);
    m_shooterA->Config_kI(0, I, ktimeoutMs);
    m_shooterA->Config_kD(0, D, ktimeoutMs);
    m_shooterA->Config_IntegralZone(0, Izone, ktimeoutMs);
    m_shooterA->ConfigMaxIntegralAccumulator(0, MaxAccumulator, ktimeoutMs);

}

double Shooter::GetVelocity() {
    return m_shooterA->GetSelectedSensorVelocity();
}

double Shooter::GetVelocityLoopError() {
    return m_shooterA->GetClosedLoopError();
}

double Shooter::GetIAccumulator() {
    return m_shooterA->GetIntegralAccumulator();
}

double Shooter::GetPercentageOutput() {
     
    return m_shooterA->GetMotorOutputPercent();
}

void Shooter::configKF(double kF) {
    m_shooterA->Config_kF(0, kF, ktimeoutMs);
}

double Shooter::GetVelocityLoopTarget() {
    return m_shooterA->GetClosedLoopTarget();

}
