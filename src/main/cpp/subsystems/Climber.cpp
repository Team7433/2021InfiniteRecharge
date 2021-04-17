// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber() {

    m_masterMotor->Config_kP(KslotID, Kp, ktimeoutMs);
    m_masterMotor->Config_kI(KslotID, Ki, ktimeoutMs);
    m_masterMotor->Config_kD(KslotID, Kd, ktimeoutMs);
    m_masterMotor->Config_kF(KslotID, Kf, ktimeoutMs);

    m_masterMotor->ConfigMotionAcceleration(0.0, ktimeoutMs);
    m_masterMotor->ConfigMotionCruiseVelocity(0.0, ktimeoutMs);

    m_slaveMotor->Follow(*m_masterMotor);


}



// This method will be called once per scheduler run
void Climber::Periodic() {}


void Climber::SetTarget(double targetEncoderCount, double maxAccel, double maxVel, double setkF) {

    if (GetLockPosition() == ClimberLock_Position::Unlock) {
        m_masterMotor->Config_kF(KslotID, setkF, ktimeoutMs);
        m_masterMotor->ConfigMotionAcceleration(maxAccel, ktimeoutMs);
        m_masterMotor->ConfigMotionCruiseVelocity(maxVel, ktimeoutMs);
        m_masterMotor->Set(ControlMode::MotionMagic, targetEncoderCount);

    } else {

        std::cout << "Climber Locked unable to enable motors\n";

    }
}