// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/ConfigImport.h"
#include <iostream>
#include <fstream>

ConfigImport::ConfigImport() = default;

ConfigImport& ConfigImport::GetInstance() {
  static ConfigImport instance;
  return instance;
}

bool ConfigImport::LoadConfig(std::string fileLocation) {
    m_configFileName = fileLocation;

    std::cout << "Config Loading File: " << fileLocation << std::endl;
    std::string line;
    std::ifstream configFile(fileLocation);
    if (configFile.is_open()) {
        int i = 0;
        while (getline(configFile, line)) {
            

            std::vector<double> lineParamaters = {};

            std::string shortenedLine = line;  

            if (line.find('!') != -1) {
                continue;
            }

            int comma_next = shortenedLine.find(',');
            int separationLength = comma_next;

            bool lineDone = false;
            //Go throught the line until their are no more comma's
            while (!lineDone) {
                //Using the point in the sting the comma is at get a sub string of the current data point in string form
                std::string dataPoint_str = shortenedLine.substr(0, separationLength);
                
                if (dataPoint_str.length() == 0) {
                    break;
                }

                // std::cout << "Importing: " << dataPoint_str << ", length: " << dataPoint_str.length() << std::endl;
                //Convert the sting to a double
                double dataPoint;
                try {
                    dataPoint = std::stod(dataPoint_str);
                } catch(const std::exception& e) {
                    std::cerr << e.what() << '\n';
                    m_errorLines.push_back(i);
                    break;
                }
                
                lineParamaters.push_back(dataPoint);

                //Shorten the string removing the inputed datapoint and its trailing comma
                shortenedLine = shortenedLine.substr(separationLength+1, shortenedLine.length()-separationLength-1);

                if (comma_next == -1) {
                    lineDone = true;
                    break;
                }

                comma_next = shortenedLine.find(',');
                separationLength = comma_next;
                if (comma_next == -1) {
                    separationLength = shortenedLine.length() - 1;
                }
                // std::cout << "Comma: " << comma_next << ", SeparationLength: " << separationLength << ", length: " << shortenedLine.length() << ", string: " << shortenedLine << std::endl;

            }

            m_paramaters.push_back(lineParamaters); 
            i++;

        } 


    } else {
        return true;
    }

    struct stat result;
    if(stat(fileLocation.c_str(), &result)==0)
    {
        auto mod_time = result.st_mtime;
        // auto t = std::time(nullptr);
        std::string tz = "TZ=Australia/Sydney";
        putenv(tz.data());
        auto tm = *std::localtime(&mod_time);
        std::stringstream timeStream;
        timeStream << std::put_time(&tm, "%c %Z");
        m_modifiedTime = timeStream.str();
    }

    std::cout << "LastModified: " << m_modifiedTime << "\n";
    // std::cout << m_errorLines << std::endl;

    // for (std::vector<double> line : m_paramaters) {
    //     for (double paramater : line) {
    //         std::cout << paramater << ",";
    //     }
    //     std::cout << std::endl;
    // }


    return false; // No Errors
}