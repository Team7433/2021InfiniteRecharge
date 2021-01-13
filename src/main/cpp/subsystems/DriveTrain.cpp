/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"

DriveTrain::DriveTrain() {

    m_leftDrive2->Follow(*m_leftDrive1);
    m_rightDrive2->Follow(*m_rightDrive1);

    m_rightDrive1->SetInverted(true);
    m_rightDrive2->SetInverted(true);

    m_rightDrive1->SetNeutralMode(NeutralMode::Brake);
    m_rightDrive2->SetNeutralMode(NeutralMode::Brake);
    m_leftDrive1->SetNeutralMode(NeutralMode::Brake);
    m_leftDrive2->SetNeutralMode(NeutralMode::Brake);
    

    m_robotDrive.SetRightSideInverted(false);

}

// This method will be called once per scheduler run
void DriveTrain::Periodic() {}

void DriveTrain::ArcadeDrive(double forward, double rotation, bool squaredInputs) {

    m_robotDrive.ArcadeDrive(forward, rotation, squaredInputs);

}

void DriveTrain::CurvatureDrive(double forward, double rotation, bool quickTurn){
    m_robotDrive.CurvatureDrive(forward, rotation, quickTurn);
}