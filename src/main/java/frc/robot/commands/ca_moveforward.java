// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.Drivetrain;

public class ca_moveforward extends CommandBase {
  /** Creates a new ca_moveforward. */
  private Drivetrain drive;
  private double inches;
  private boolean isFinished = false;

  public ca_moveforward(Drivetrain dt, double in) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = dt;
    inches = in;
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
    if (drive.getEncoderDistance() > drive.inchesToTicks(inches)) {
      drive.stopAllDriveMotors();
      isFinished = true;
    } else {
      drive.drive_straight_gyro(.25);
      isFinished = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}