// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.extensions.Limelight;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Turret;

public class c_detectShootingReady extends CommandBase {
  Shooter shooter;
  Intake intake;
  Turret turret;

  /** Creates a new c_shootWithVision. */
  public c_detectShootingReady(Intake it, Shooter sh, Turret trt) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = it;
    addRequirements(intake);
    shooter = sh;
    addRequirements(shooter);
    turret = trt;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Need a distance check
    // Need an on target check
    // Only if all three are true do we shoot
    if (shooter.runShooterPID(intake.readSensor(), Limelight.getDistance(), DriverStation.getAlliance())) {
      intake.runMagazineMotors(true);
    } else {
      intake.runMagazineMotors(false);
    }

    // Aim the turret
    turret.turnOnPIDAutoAim();

    // Run singulation
    intake.singulateBall();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooter.stopShooterMotor();
    intake.magazineMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
