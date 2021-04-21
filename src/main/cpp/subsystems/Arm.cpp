/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Arm.h"
#include <units/length.h>
#include <units/angle.h>
#include <units/math.h>

Arm::Arm() {

    m_armMotor->ConfigFactoryDefault();


    m_armMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, ktimeoutMs);
    m_armEncoder->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, ktimeoutMs);


    m_armMotor->SetNeutralMode(NeutralMode::Brake);

    m_armMotor->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyClosed, ktimeoutMs);
    m_armMotor->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, ktimeoutMs);
    
    m_armMotor->SetSensorPhase(true);
    m_armEncoder->SetSensorPhase(true);

    m_armMotor->ConfigNeutralDeadband(0);
    

    // configures motors PID
    m_armMotor->Config_kF(0, Kf, ktimeoutMs);
    m_armMotor->Config_kP(0, Kp, ktimeoutMs);
    m_armMotor->Config_kI(0, Ki, ktimeoutMs);
    m_armMotor->Config_kD(0, Kd, ktimeoutMs);
    m_armMotor->ConfigMotionSCurveStrength(4, ktimeoutMs);

    m_armMotor->ConfigMotionAcceleration(KmotionAcceleration, ktimeoutMs);
    m_armMotor->ConfigMotionCruiseVelocity(KmotionCruiseVelocity, ktimeoutMs);  

    m_armMotor->ConfigClearPositionOnLimitR(true, ktimeoutMs);
    frc::SmartDashboard::PutBoolean("Arm/LockButton", false);

}

// This method will be called once per scheduler run
void Arm::Periodic() {
    frc::SmartDashboard::PutNumber("Arm/Velocity", GetVelocity());
    frc::SmartDashboard::PutNumber("Arm/Position", GetPosition());
    // frc::SmartDashboard::PutNumber("Arm/TargetEncoderCount", GetTargetPositionUnits().to<double>());
    frc::SmartDashboard::PutNumber("Arm/Accumulator", m_armMotor->GetIntegralAccumulator());
    // frc::SmartDashboard::PutNumber("Arm/EncoderError", m_armMotor->GetClosedLoopError());
    frc::SmartDashboard::PutNumber("Arm/Output", GetMotorOutput());
    frc::SmartDashboard::PutString("Arm/Angle", units::angle::to_string(GetArmAngleUnits()));
    frc::SmartDashboard::PutString("Arm/AngleMotor", units::angle::to_string(GetArmAngleMotorUnits()));
    frc::SmartDashboard::PutNumber("Arm/EncoderDifference", (GetArmAngleUnits() - GetArmAngleMotorUnits()).to<double>() );

}

void Arm::ManualControl(double Output) {
    m_armMotor->Set(ControlMode::PercentOutput, Output);
}

void Arm::SetPosition(double Position) {
    m_armMotor->Set(ControlMode::MotionMagic, Position);
}

void Arm::SetAngle(units::angle::armEncoderUnits_t Angle) {
    m_armMotor->Set(ControlMode::MotionMagic, Angle.to<double>());
}

void Arm::SetAngle(units::degree_t Angle, double feedForward) {
    //some complex math equations ++--++--
        m_armMotor->Set(ControlMode::MotionMagic, (53.8) * (Angle.to<double>() - armAngleOffset), DemandType::DemandType_ArbitraryFeedForward, feedForward);

}

void Arm::SetLock(Lock_Position lock_position) {

    if (lock_position == Lock_Position::Lock) {
        m_lockSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        m_lockSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
}

double Arm::GetPosition() {
    return m_armMotor->GetSelectedSensorPosition();
}

double Arm::GetVelocity() {
    return m_armMotor->GetSelectedSensorVelocity();
}
double Arm::GetMotorOutput() {
    return m_armMotor->GetMotorOutputPercent();
}
double Arm::GetError() {
    return m_armMotor->GetClosedLoopError();
}


double Arm::GetTargetPosition() {
    return m_armMotor->GetClosedLoopTarget();
}

units::degree_t Arm::GetTargetPositionUnits() {
    return units::armEncoderUnits_t(m_armMotor->GetClosedLoopTarget());
}

double Arm::GetArmAngle() {

    return ((m_armEncoder->GetSelectedSensorPosition() / 4096.0) * 360 ) + 196;
}

double Arm::GetArmAngleMotor() {
   
    return (m_armMotor->GetSelectedSensorPosition() / 53.8) +  armAngleOffset;

}

units::degree_t Arm::GetArmAngleUnits() {

    return units::degree_t(((m_armEncoder->GetSelectedSensorPosition() / 4096.0) * 360 ) + 196);

}

units::degree_t Arm::GetArmAngleMotorUnits() {
   
    return units::armEncoderUnits_t(m_armMotor->GetSelectedSensorPosition());

}

units::degree_t Arm::CalculateAngleFromDistance(units::meter_t distance) {

    double distanceMM = distance.to<double>();
    
    return units::degree_t(15.5504 + (130.439 / (distanceMM + 2.46224)));

}

bool Arm::GetArrived() {


    return units::math::fabs(GetArmAngleMotorUnits() - GetTargetPositionUnits()) < 0.5_deg;

}

