// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  WPI_TalonFX climbMotor;
  WPI_TalonSRX rotationMotor;
  WPI_TalonSRX pinMotor;

  /** Creates a new Climber. */
  public Climber() {

    climbMotor = new WPI_TalonFX(Constants.climbMotorCanID);
    rotationMotor = new WPI_TalonSRX(Constants.rotationmotorCanID);
    pinMotor = new WPI_TalonSRX(Constants.pinMotorCanID);
  }

  public void stopAllClimbMotors(){
    climbMotor.stopMotor();
    rotationMotor.stopMotor();
    pinMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
