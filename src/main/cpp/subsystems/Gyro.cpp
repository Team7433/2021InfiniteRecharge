/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Gyro.h"
#include <frc/DriverStation.h>

Gyro::Gyro() {
    try {
       /* Communicate w/navX-MXP via the MXP SPI Bus.                                       */
       /* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
       /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
        m_gyro = new AHRS(frc::SPI::Port::kMXP);

    } catch (std::exception ex ) {

        std::string err_string = "Error Instantiating navX-MXP: ";
        err_string += ex.what();
        frc::DriverStation::ReportError(err_string.c_str());
    }



    Reset();

}

// This method will be called once per scheduler run
void Gyro::Periodic() {
    frc::SmartDashboard::PutNumber("Gyro/Yaw", GetYaw().to<double>());
    frc::SmartDashboard::PutNumber("Gyro/Roll", GetRoll().to<double>());
    frc::SmartDashboard::PutNumber("Gyro/Pitch", GetPitch().to<double>());
}

units::degree_t Gyro::GetYaw() {
    return units::degree_t( m_gyro->GetYaw() );
}

void Gyro::Reset() {
    return m_gyro->Reset();
}

units::degree_t Gyro::GetPitch() {
    return units::degree_t( m_gyro->GetPitch() );
}

units::degree_t Gyro::GetRoll() {
    return units::degree_t( m_gyro->GetRoll() );
}
