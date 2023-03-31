// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/voltage.h>
#include <units/length.h>
#include <units/angle.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/controller/SimpleMotorFeedForward.h>
#include <ctre/Phoenix.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants
{
    //One weight
    constexpr units::volt_t kS = 0.14848_V;
    constexpr auto kV = 1.8528 * 1_V / 1_mps;
    constexpr auto kA = 0.45158 * 1_V / 1_mps_sq;
    constexpr units::meter_t kTrackWidth = 0.48251_m;
    constexpr auto kVAngular = 2.1592 * 1_V/1_mps;
    constexpr auto kAAngular = 0.53732 * 1_V / 1_mps_sq;
    constexpr auto kRightDirection = TalonFXInvertType::CounterClockwise;
    constexpr auto kLeftDirection = TalonFXInvertType::Clockwise;
    /*constexpr units::volt_t kS = 0.14643_V;
    constexpr auto kV = 1.8676 * 1_V / 1_mps;
    constexpr auto kA = 0.46219 * 1_V / 1_mps_sq;
    constexpr units::meter_t kTrackWidth = 0.44618_m;
    constexpr auto kVAngular = 2.0807 * 1_V/1_mps;
    constexpr auto kAAngular = 0.39601 * 1_V / 1_mps_sq;*/

}

namespace testRobot
{  
    constexpr units::volt_t kS = 0.62045_V;
    constexpr auto kV = 2.5115 * 1_V / 1_mps;
    constexpr auto kA = 0.31661 * 1_V / 1_mps_sq;
    constexpr units::meter_t kTrackWidth = 0.69_m;
    constexpr auto kVAngular = 2.7453 * 1_V/1_mps;
    constexpr auto kAAngular = 0.054563 * 1_V / 1_mps_sq;
    constexpr auto kLeftDirection = TalonFXInvertType::Clockwise;
    constexpr auto kRightDirection = TalonFXInvertType::CounterClockwise;
}
    
namespace vision
{
    constexpr units::meter_t cameraHeight = 53_in;
    constexpr units::meter_t targetHeight = 28.5_in;
    constexpr units::degree_t cameraPitch = -3_deg;
    constexpr units::meter_t retroHeight = 44_in;
    constexpr units::meter_t aprilTagArray[9] = {0_in, 18.22_in, 18.22_in, 18.22_in, 27.38_in, 27.38_in, 18.22_in, 18.22_in, 18.22_in}; 
}
