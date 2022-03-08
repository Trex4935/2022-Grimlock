// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_archive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Shooter;

public class c_redBlueDecision extends CommandBase {
  /** Creates a new c_redBlueDecision. */
  private final Intake intake;
  private final Shooter shooter;

  public c_redBlueDecision(Intake it, Shooter sh) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = it;
    shooter = sh;
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooter.runShooterPID(1000);
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
