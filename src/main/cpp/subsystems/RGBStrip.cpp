// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/RGBStrip.h"


RGBStrip::RGBStrip() = default;

// This method will be called once per scheduler run
void RGBStrip::Periodic() {}

void RGBStrip::SetRGBStrip(double R, double G, double B) {


    m_strip->SetLEDOutput(R/255.0, CANifier::LEDChannelB);
    m_strip->SetLEDOutput(G/255.0, CANifier::LEDChannelA);
    m_strip->SetLEDOutput(B/255.0, CANifier::LEDChannelC);
    m_R = R;
    m_G = G;
    m_B = B;

}

void RGBStrip::Rainbow() {
    srand (time(NULL));
    double randomR = 1 - rand() % 3;
    double randomG = 1 - rand() % 3;
    double randomB = 1 - rand() % 3;

    m_R = m_R - randomR;
    m_G = m_G - randomG;
    m_B = m_B - randomB;

    m_strip->SetLEDOutput(m_R/255.0, CANifier::LEDChannelB);
    m_strip->SetLEDOutput(m_G/255.0, CANifier::LEDChannelA);
    m_strip->SetLEDOutput(m_B/255.0, CANifier::LEDChannelC);

}
