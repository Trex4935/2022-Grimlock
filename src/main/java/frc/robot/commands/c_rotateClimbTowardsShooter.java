// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.LeftClimber;

public class c_rotateClimbTowardsShooter extends CommandBase {
  /** Creates a new runIntakeMotors. */
  private final LeftClimber climber;

  public c_rotateClimbTowardsShooter(LeftClimber cl) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = cl;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // climber.rotateArmsTowardsShooter();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // climber.stopArmRotation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
