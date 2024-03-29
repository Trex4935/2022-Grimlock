// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystem.Drivetrain;

public class c_driveWithController extends CommandBase {
  private final Drivetrain drive;
  private final XboxController controller;
  private final XboxController coDriver;

  public c_driveWithController(Drivetrain dt, XboxController con, XboxController cod) {
    drive = dt;
    controller = con;
    coDriver = cod;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.driveWithController(controller, coDriver, Constants.driveSpeedLimit);
    // drive.driveMiddleWithController(controller, Constants.driveSpeedLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("SHOULD NOT BE HERE");
    drive.stopAllDriveMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
