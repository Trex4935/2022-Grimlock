// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  WPI_TalonFX turretShooter;
  CANSparkMax turretRotation;
  public CANSparkMax m_leftMotor;

  Limelight limelight;

  /** Creates a new turret. */
  public Turret() {

    turretShooter = new WPI_TalonFX(Constants.turretShooterCanID);
    // turretRotation = new WPI_TalonFX(Constants.turretRotationCanID);

    limelight = new Limelight();
    turretRotation = new CANSparkMax(Constants.turretRotationCanID, MotorType.kBrushless);

  }

  public void turnOnSimpleAutoAim() {

    turretRotation.set(limelight.getLimelightX() / 270);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
