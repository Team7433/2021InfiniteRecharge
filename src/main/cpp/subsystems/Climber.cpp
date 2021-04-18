// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

#include "frc/smartdashboard/SmartDashboard.h"

Climber::Climber() {

    m_masterMotor->Config_kP(KslotID, Kp, ktimeoutMs);
    m_masterMotor->Config_kI(KslotID, Ki, ktimeoutMs);
    m_masterMotor->Config_kD(KslotID, Kd, ktimeoutMs);
    m_masterMotor->Config_kF(KslotID, Kf, ktimeoutMs);

    m_masterMotor->ConfigMotionAcceleration(0.0, ktimeoutMs);
    m_masterMotor->ConfigMotionCruiseVelocity(0.0, ktimeoutMs);

    m_slaveMotor->SetNeutralMode(NeutralMode::Coast);
    m_masterMotor->SetNeutralMode(NeutralMode::Brake);

    m_masterMotor->ConfigMotionAcceleration(1*((11.431*2048)/10), ktimeoutMs);
    m_masterMotor->ConfigMotionCruiseVelocity(2*((11.431*2048)/10), ktimeoutMs);

    // m_slaveMotor->Follow(*m_masterMotor);


}



// This method will be called once per scheduler run
void Climber::Periodic() {

    frc::SmartDashboard::PutNumber("Climber/PercetangeOutput", m_masterMotor->GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Climber/Error", m_masterMotor->GetClosedLoopError());

    if (GetLockPosition() == ClimberConstants::ClimberLock_Position::Lock) {

        SetOutput([] {return 0.0;});

    }

}


void Climber::SetTarget(double targetEncoderCount, double maxAccel, double maxVel) {

    if (GetLockPosition() == ClimberLock_Position::Unlock) {
        m_masterMotor->ConfigMotionAcceleration(maxAccel, ktimeoutMs);
        m_masterMotor->ConfigMotionCruiseVelocity(maxVel, ktimeoutMs);
        m_masterMotor->Set(ControlMode::MotionMagic, targetEncoderCount);

    } else {

        std::cout << "Climber Locked unable to enable motors\n";

    }
}

void Climber::SetOutput(std::function<double()> output) {
    if (GetLockPosition() == ClimberLock_Position::Unlock) {
        m_masterMotor->Set(ControlMode::PercentOutput, output());
    } else { std::cout << "Climber Locked unable to enable motors\n"; }
}

void Climber::RunRevolutions(double revolutions, double maxAccel, double maxVel) {

    if (GetLockPosition() == ClimberLock_Position::Unlock) {

        m_masterMotor->ConfigMotionAcceleration(maxAccel*((11.431*2048)/10), ktimeoutMs);
        m_masterMotor->ConfigMotionCruiseVelocity(maxVel*((11.431*2048)/10), ktimeoutMs);
        m_masterMotor->Set(ControlMode::MotionMagic, m_masterMotor->GetSelectedSensorPosition()+(revolutions*(2048*11.431)));

    } else {

        std::cout << "Climber Locked unable to enable motors\n";

    }


}

void Climber::RunDynamicRevoltions(std::function<double()> revolutionTarget) {

    if (GetLockPosition() == ClimberConstants::ClimberLock_Position::Unlock) {

        m_masterMotor->Set(ControlMode::MotionMagic, (revolutionTarget()*(2048*11.431)));


    } else {

        std::cout << "Climber Locked unable to enable motors\n";

    }

}
