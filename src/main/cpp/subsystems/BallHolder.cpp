/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/BallHolder.h"
#include <frc/smartdashboard/SmartDashboard.h>

BallHolder::BallHolder() {
    m_indexerMotor->ConfigFactoryDefault();
    m_magazineMotor->ConfigFactoryDefault();

    m_indexerMotor->SetInverted(true);
    m_magazineMotor->SetInverted(false);
}

// This method will be called once per scheduler run
void BallHolder::Periodic() {
    frc::SmartDashboard::PutBoolean("BeltInSensor", GetSensorBeltIn());
    frc::SmartDashboard::PutBoolean("BeltFullSensor", GetSensorBeltFull());
    frc::SmartDashboard::PutBoolean("indexerOutSensor", GetSensorIndexerOut());
    frc::SmartDashboard::PutBoolean("indexerInSensor", GetSensorIndexerIn());
    frc::SmartDashboard::PutBoolean("BeltMiddleSensor", GetSensorBeltMiddle());
}

void BallHolder::SetMagazine(double value) {
    m_magazineMotor->Set(ControlMode::PercentOutput, value);
}

void BallHolder::SetIndexer(double value) {
    m_indexerMotor->Set(ControlMode::PercentOutput, value);
}

bool BallHolder::GetSensorBeltIn() {
    return !m_sensor_beltIn.Get();
}

bool BallHolder::GetSensorBeltFull() {
    return !m_sensor_beltFull.Get();
}

bool BallHolder::GetSensorIndexerOut() {
    return !m_sensor_IndexerOut.Get();
}

bool BallHolder::GetSensorIndexerIn() {
    return !m_sensor_IndexerIn.Get();
}

bool BallHolder::GetSensorBeltMiddle() {
    return !m_sensor_BeltMiddle.Get();
}