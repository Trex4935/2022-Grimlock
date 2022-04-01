// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.Pneumatics;

public class c_intakeForward extends CommandBase {
  private final Pneumatics pneumatics;

  /** Creates a new c_rotateAndUpClimb. */
  public c_intakeForward(Pneumatics pn) {
    // Use addRequirements() here to declare subsystem dependencies.
    pneumatics = pn;
    addRequirements(pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pneumatics.intakeForward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
