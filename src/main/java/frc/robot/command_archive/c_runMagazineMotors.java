// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_archive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.Intake;

public class c_runMagazineMotors extends CommandBase {
  private final Intake intake;

  /** Creates a new runMagazineMotors. */
  public c_runMagazineMotors(Intake it) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = it;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.runMagazineMotors();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.readProxColorSensor();
    intake.magazineMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
