// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DiffSwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>

#include "Constants.h"

DiffSwerveModule::DiffSwerveModule(int driveMotorChannel, int turningMotorChannel,
                           int steeringEncoderID)
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      m_steeringEncoder(steeringEncoderID) {
  m_turningPIDController.EnableContinuousInput(units::radian_t(-wpi::math::pi),
                                               units::radian_t(wpi::math::pi));
}

frc::SwerveModuleState DiffSwerveModule::GetState() {
  return {units::meters_per_second_t{GetDriveMotorSpeed()},
          frc::Rotation2d(units::radian_t(m_steeringEncoder.GetVelocity() * (wpi::math::pi / 180.0)))};
}

void DiffSwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(m_steeringEncoder.GetPosition() * (wpi::math::pi / 180.0)));

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      GetDriveMotorSpeed(), state.speed.to<double>());

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t(m_steeringEncoder.GetPosition() * (wpi::math::pi / 180.0)), state.angle.Radians());

  // Set the motor outputs.
  m_driveMotor.Set(ControlMode::PercentOutput, (driveOutput - turnOutput) * 0.1);
  m_turningMotor.Set(ControlMode::PercentOutput, (-driveOutput - turnOutput) * 0.1);
}

void DiffSwerveModule::ResetEncoders() {
}
