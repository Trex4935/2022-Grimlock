// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.Limelight;
import frc.robot.subsystem.Shooter;

public class c_shootWithVision extends CommandBase {
  Shooter shooter;
  Limelight limelight;

  /** Creates a new c_shootWithVision. */
  public c_shootWithVision(Shooter sh, Limelight ll) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = sh;
    limelight = ll;
    addRequirements(shooter, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shootBallWithVision(limelight.getDistance());
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