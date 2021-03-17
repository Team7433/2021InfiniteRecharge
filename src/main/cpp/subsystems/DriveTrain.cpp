/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"

DriveTrain::DriveTrain() {

    m_leftDriveSlave->Follow(*m_leftDriveMaster);
    m_rightDriveSlave->Follow(*m_rightDriveMaster);

    m_rightDriveMaster->SetInverted(true);
    m_rightDriveSlave->SetInverted(true);

    m_rightDriveMaster->SetNeutralMode(NeutralMode::Brake);
    m_rightDriveSlave->SetNeutralMode(NeutralMode::Brake);
    m_leftDriveMaster->SetNeutralMode(NeutralMode::Brake);
    m_leftDriveSlave->SetNeutralMode(NeutralMode::Brake);
    
    m_leftDriveMaster->ConfigVoltageCompSaturation(12);
    m_leftDriveMaster->EnableVoltageCompensation(true);
  
    m_rightDriveMaster->ConfigVoltageCompSaturation(12);
    m_rightDriveMaster->EnableVoltageCompensation(true);

    m_robotDrive.SetRightSideInverted(false);

    //Set P
    m_leftDriveMaster->Config_kP(kPIDSlotIdx, kP_Profiling, kTimeoutMs);//1
    m_rightDriveMaster->Config_kP(kPIDSlotIdx, kP_Profiling, kTimeoutMs);

    //Set I
    m_leftDriveMaster->Config_kI(kPIDSlotIdx, kI_Profiling, kTimeoutMs);
    m_rightDriveMaster->Config_kI(kPIDSlotIdx, kI_Profiling, kTimeoutMs);

    //Set D
    m_leftDriveMaster->Config_kD(kPIDSlotIdx, kD_Profiling, kTimeoutMs);//20
    m_rightDriveMaster->Config_kD(kPIDSlotIdx, kD_Profiling, kTimeoutMs);

    //Set F
    m_leftDriveMaster->Config_kF(kPIDSlotIdx, kF_Profiling, kTimeoutMs);
    m_rightDriveMaster->Config_kF(kPIDSlotIdx, kF_Profiling, kTimeoutMs);

    m_leftDriveMaster->ChangeMotionControlFramePeriod(5);
    m_rightDriveMaster->ChangeMotionControlFramePeriod(5);

    m_robotDrive.SetSafetyEnabled(false);

    m_leftDriveMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic,10,kTimeoutMs);
    m_rightDriveMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic,10,kTimeoutMs);

    m_profiler->SetMetersToUnits((1/kMetersPerUnit));
    m_profiler->SetmpsToUnit100ms(kUnits100msPerMeterSecond);

}

// This method will be called once per scheduler run
void DriveTrain::Periodic() {}

void DriveTrain::ArcadeDrive(double forward, double rotation, bool squaredInputs) {

    m_robotDrive.ArcadeDrive(forward, rotation, squaredInputs);

}

void DriveTrain::CurvatureDrive(double forward, double rotation, bool quickTurn){
    m_robotDrive.CurvatureDrive(forward, rotation, quickTurn);
}