// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.Climber;

public class c_robotClimbsDownMotionMagic extends CommandBase {
  /** Creates a new runIntakeMotors. */
  private final Climber climber;

  public c_robotClimbsDownMotionMagic(Climber cl) {
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
    climber.climbDownMotionMagic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimbMotor();
    climber.setEncoderToZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getMotorBottomLimit();
  }
}
