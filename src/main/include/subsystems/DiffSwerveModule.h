// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/Spark.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <ctre/Phoenix.h>
#include <wpi/math>

#include "Constants.h"

class DiffSwerveModule {
  using radians_per_second_squared_t =
      units::compound_unit<units::radians,
                           units::inverse<units::squared<units::second>>>;

 public:
  DiffSwerveModule(int driveMotorChannel, int turningMotorChannel,
               //const int driveEncoderPorts[2], const int turningEncoderPorts[2],
               int steeringEncoderID);

  frc::SwerveModuleState GetState();

  void SetDesiredState(const frc::SwerveModuleState& state);

  void ResetEncoders();

  float GetDriveMotorSpeed(){
      return ((m_driveMotor.GetSelectedSensorVelocity() - m_turningMotor.GetSelectedSensorVelocity()) / 2.0) 
          * (10.0 / 2048) /*Revs per second*/ * ((10 / 88.0) * (54 / 14.0) * (1 / 3.0)) /*Gear Ratios*/ * (4.5 * 0.0254 * wpi::math::pi) /*Axle Revs per Second*/;
  }

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
      units::radians_per_second_t(wpi::math::pi);  // radians per second
  static constexpr units::unit_t<radians_per_second_squared_t>
      kModuleMaxAngularAcceleration =
          units::unit_t<radians_per_second_squared_t>(
              wpi::math::pi * 2.0);  // radians per second squared

  TalonFX m_driveMotor;
  TalonFX m_turningMotor;

  //frc::Encoder m_driveEncoder;
  //frc::Encoder m_turningEncoder;

  CANCoder m_steeringEncoder;

  bool m_reverseDriveEncoder;
  bool m_reverseTurningEncoder;

  frc2::PIDController m_drivePIDController{
      ModuleConstants::kPModuleDriveController, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      ModuleConstants::kPModuleTurningController,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
};
