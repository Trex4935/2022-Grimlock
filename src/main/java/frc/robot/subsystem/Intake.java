// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  WPI_TalonFX intakeMotor;
  WPI_TalonFX magazineMotor1;
  WPI_TalonFX magazineMotor2;

  public Intake() {

  intakeMotor = new WPI_TalonFX(Constants.intakeMotorCanID);
  magazineMotor1 = new WPI_TalonFX(Constants.magazineMotor1CanID);
  magazineMotor2 = new WPI_TalonFX(Constants.magazineMotor2CanID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
