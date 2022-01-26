// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class turret extends SubsystemBase {
  WPI_TalonFX turretShooter;
  WPI_TalonFX turretRotation;

  /** Creates a new turret. */
  public turret() {

    turretShooter = new WPI_TalonFX(Constants.turretShooterCanID);
    turretRotation = new WPI_TalonFX(Constants.turretRotationCanID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
