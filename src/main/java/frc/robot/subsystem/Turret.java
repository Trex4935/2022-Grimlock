// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  WPI_TalonFX turretShooter;
  PWMSparkMax turretRotation = new PWMSparkMax(Constants.turretRotationPWMID);
  

  Limelight limelight;

  /** Creates a new turret. */
  public Turret() {

    // Init motors
    turretShooter = new WPI_TalonFX(Constants.turretShooterCanID);
    // turretRotation = new WPI_TalonFX(Constants.turretRotationCanID);

    limelight = new Limelight();

  }

  // Uses limelight output to move rotation motor directly
  public void turnOnSimpleAutoAim() {
    turretRotation.set(limelight.getLimelightX() / 270);
  }

  // Moves the rotation motor based on controller input
  public void aimWithController(XboxController controller) {

    double LT = controller.getRawAxis(Constants.leftTrigger);
    double RT = controller.getRawAxis(Constants.rightTrigger) * -1;
    turretRotation.set((LT + RT) / 25);
  }

  // Stop the rotation moto
  public void stopRotationMotor() {
    turretRotation.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}