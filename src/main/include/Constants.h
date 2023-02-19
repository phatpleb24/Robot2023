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
    constexpr units::volt_t kS = 0.17242_V;
    constexpr auto kV = 1.8421 * 1_V / 1_mps;
    constexpr auto kA = 0.36004 * 1_V / 1_mps_sq;
    constexpr units::meter_t kTrackWidth = 0.48251_m;
    constexpr auto kVAngular = 2.1647 * 1_V/1_mps;
    constexpr auto kAAngular = 0.30817 * 1_V / 1_mps_sq;
    constexpr auto kRightDirection = TalonFXInvertType::Clockwise;
    constexpr auto kLeftDirection = TalonFXInvertType::CounterClockwise;

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
    constexpr units::meter_t cameraHeight = 1.875_in;
    constexpr units::meter_t targetHeight = 29_in;
    constexpr units::degree_t cameraPitch = 2.5_deg;
}
