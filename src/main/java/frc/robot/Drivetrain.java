// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase {

  // Declaring motors
  WPI_TalonFX leftFront;
  WPI_TalonFX leftBack;
  WPI_TalonFX rightFront;
  WPI_TalonFX rightBack;
  WPI_TalonFX middleLeft;
  WPI_TalonFX middleRight;
  
  // Declaring motor groups
  MotorControllerGroup rightSide;
  MotorControllerGroup leftSide;
  MotorControllerGroup middleSide;

  // Drives
  DifferentialDrive drive;

  public Drivetrain() {

    // Creating Motor Objects
leftFront = new WPI_TalonFX(Constants.leftFrontCanID);
leftBack = new WPI_TalonFX(Constants.leftBackCanID);
rightFront = new WPI_TalonFX(Constants.rightFrontCanID);
rightBack = new WPI_TalonFX(Constants.rightBackCanID);
middleLeft = new WPI_TalonFX(Constants.middleLeftCanID);
middleRight = new WPI_TalonFX(Constants.middleRightCanID);

    // Creating Motor Groups
rightSide = new MotorControllerGroup(rightFront, rightBack);
leftSide = new MotorControllerGroup(leftFront, leftBack);
middleSide = new MotorControllerGroup(middleLeft, middleRight);

    // Creating Drive Movement
drive = new DifferentialDrive(leftSide, rightSide);
  }

  // Move mid motor
public void driveMiddleWithController(XboxController controller){
  middleLeft.set(controller.getRawAxis(Constants.leftHorizontal));
  middleRight.set(controller.getRawAxis(Constants.leftHorizontal));
}

public void middleStop() {
  middleLeft.set(Constants.motorStop);
  middleRight.set(Constants.motorStop);
}

  @Override
  public void periodic() {
  }

public void driveWithController(XboxController controller) {
  drive.arcadeDrive(controller.getRawAxis(Constants.rightStick), controller.getRawAxis(Constants.leftStick)* -1);
}
}
