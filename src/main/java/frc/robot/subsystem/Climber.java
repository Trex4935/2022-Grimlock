// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

// Imports
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  // Declare Motors
  WPI_TalonFX climbMotor;
  WPI_TalonSRX rotationMotor;
  WPI_TalonSRX pinMotor;

  // Declare Sensors

  // Construct a climber object
  public Climber() {

    // Populate the variables with motor objects with the correct IDs
    climbMotor = new WPI_TalonFX(Constants.climbMotorCanID);
    rotationMotor = new WPI_TalonSRX(Constants.rotationMotorCanID);
    pinMotor = new WPI_TalonSRX(Constants.pinMotorCanID);

  }

  // Stop all of the climb motors
  public void stopAllClimbMotors() {
    climbMotor.stopMotor();
    rotationMotor.stopMotor();
    pinMotor.stopMotor();
  }

  public void pinRotate() throws InterruptedException {
    pinMotor.set(Constants.pinMotorSpeed);
    wait(1000);
    pinMotor.stopMotor();
  }

  public void rotateClimbLeft() {
    climbMotor.setInverted(false);
    climbMotor.set(Constants.climbRotateSpeed);
  }

  public void rotateClimbRight() {
    climbMotor.setInverted(true);
    climbMotor.set(Constants.climbRotateSpeed);
  }

  public void stopClimbRotate() {
    climbMotor.stopMotor();
  }

  public void motorClimbUp() {
    climbMotor.setInverted(false);
    climbMotor.set(Constants.climbMotorSpeed);
  }

  public void motorClimbDown() {
    climbMotor.setInverted(true);
    climbMotor.set(Constants.climbMotorSpeed);
  }

  public void stopClimbMotor() {
    climbMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
