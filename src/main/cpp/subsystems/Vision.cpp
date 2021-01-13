/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Vision.h"


Vision::Vision(Arm* arm) {
    m_arm = arm;
    

}

// This method will be called once per scheduler run
void Vision::Periodic() {
    frc::SmartDashboard::PutNumber("Lidar Distance", getLidarDistance());
    frc::SmartDashboard::PutBoolean("TargetDetected", getPowerPortDetected());
    frc::SmartDashboard::PutNumber("Target Distance Bumper", getPortDistanceBumper());
    frc::SmartDashboard::PutNumber("Target Distance", getPortDistance());
}

bool Vision::getPowerPortDetected() {
    if (nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0.0) == 1) {
        return true;
    }
    return false;
}

double Vision::getPowerPortHorizontalAngle() {
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
}

double Vision::getPowerPortVerticalAngle() {
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
}
double Vision::getCamMode() {
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("camMode", 0);
}
void Vision::changeCamMode(int mode) {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", mode);
}

double Vision::getPortDistance() {
    if (getPowerPortDetected() == true) {
        AngleOfArm = m_arm->GetArmAngle();
        
        HC = klengthOfArm * sin((AngleOfArm+kangleOffsetCamera)*kPi/180) + kheightOfRobot;
        frc::SmartDashboard::PutNumber("HC", HC);
        HeightOfTarget_datam = kheightOfTarget - HC;


        PHI = AngleOfArm + kangleOfCamera;
        frc::SmartDashboard::PutNumber("PHI Value", PHI);

        return ((HeightOfTarget_datam)/tan((PHI + getPowerPortVerticalAngle())*kPi/180));
    } else {
        return (0);
    }
}

double Vision::getPortDistanceBumper() {

    if (getPowerPortDetected() == true) {
        AngleOfArm = m_arm->GetArmAngle();

        // return getPortDistance() + (klengthOfArm * cos(AngleOfArm+kangleOffsetCamera)*kPi/180);
        return getPortDistance() + klengthOfArm * cos((AngleOfArm+kangleOffsetCamera)*kPi/180) - klengthPivotToBumper;
        ;
        

    } else {
        return 0;
    }


}

double Vision::getLidarDistance() {
    m_lidar->SetMaxPeriod(1.0);
    m_lidar->SetSemiPeriodMode(true);
    m_lidar->Reset();
    double cm = ((m_lidar->GetPeriod() * 1000000.0 / 10.0)/ 100);
    return cm;
}