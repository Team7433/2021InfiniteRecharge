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
    double randomR = (rand() % 9 + 1);
    srand (time(NULL)-20);
    double randomG = (rand() % 9 + 1);
    srand (time(NULL)-40);
    double randomB = (rand() % 9 + 1);
    
    randomR = randomR/1000;
    randomR = randomR/1000;
    randomR = randomR/1000;
    
    if (rand() %3 == 1 && m_countR > rand() %10000 + 10000) {

        m_countR = 0;
        if (m_signR == -1) {
            m_signR = 1;
        } else {m_signR = -1;}

    } else {m_countR++;}
    if(m_R + (randomR*m_signR) > 255) {
        m_signR = -1;
    } else if (m_R + (randomR*m_signR) < 0) {
        m_signR = +1;
        }
    m_R = m_R + (randomR*m_signR);


//     if (m_R + randomR > 255) {
//         if (m_R - randomR < 0) {
//             m_R = m_R + randomR;
//         } else {m_R = m_R - randomR;}
//     } else {m_R = m_R + randomR;}

//     if (m_G + randomG > 255) {
//         if (m_G - randomR < 0) {
//             m_G = m_G + randomG;
//         } else {m_G = m_G - randomG;}
//     } else {m_G = m_G + randomG;}

//     if (m_B + randomB > 255) {
//         if (m_B - randomB < 0) {
//             m_B = m_B + randomB;
//         } else {m_B = m_B - randomB;}
//     } else {m_B = m_B + randomB;}


    m_strip->SetLEDOutput(m_R/255.0, CANifier::LEDChannelB);
    // m_strip->SetLEDOutput(m_G/255.0, CANifier::LEDChannelA);
    // m_strip->SetLEDOutput(m_B/255.0, CANifier::LEDChannelC);

}
