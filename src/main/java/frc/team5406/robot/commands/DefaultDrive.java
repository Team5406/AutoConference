// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultDrive extends CommandBase {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;

  public DefaultDrive(DriveSubsystem _drive, DoubleSupplier _forward, DoubleSupplier _rotation) {
    m_drive = _drive;
    m_forward = _forward;
    m_rotation = _rotation;
    addRequirements(m_drive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_forward.getAsDouble(), m_rotation.getAsDouble());
  }

}
