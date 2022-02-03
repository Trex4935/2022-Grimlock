// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Turret;

public class c_redBlueDecision extends CommandBase {
  /** Creates a new c_redBlueDecision. */
  private final Intake intake;
  private final Turret turret;
  public c_redBlueDecision(Intake it, Turret trt) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = it;
    turret = trt;
    addRequirements(intake, turret);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.runTurretShooter(intake.redBlueDecision(intake.readSensor()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopShooterMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
