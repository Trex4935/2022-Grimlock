// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.Drivetrain;

public class ca_moveForwardInches extends CommandBase {
  /** Creates a new ca_moveforward. */
  private Drivetrain drive;
  private double inches;
  private double speed;

  public ca_moveForwardInches(Drivetrain dt, double in, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = dt;
    inches = in;
    speed = speed;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive_straight_gyro(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopAllDriveMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (drive.getEncoderDistance() > drive.inchesToTicks(inches));
  }
}