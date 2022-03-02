// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Shooter;

public class c_detectShootingReady extends CommandBase {
  Shooter shooter;
  Intake intake;

  /** Creates a new c_shootWithVision. */
  public c_detectShootingReady(Intake it, Shooter sh) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = it;
    shooter = sh;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run shooter motor at correct speed
    shooter.runShooter(intake.redBlueDecision(intake.readSensor()));
    Constants.readyToShoot = true;

    // get shoot speed
    // get on target
    // run mag -- to shoot ball

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooterMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
