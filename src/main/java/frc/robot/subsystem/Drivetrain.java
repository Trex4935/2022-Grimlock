// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

// Imports
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  // Declaring motors
  WPI_TalonFX leftFront;
  WPI_TalonFX leftBack;
  WPI_TalonFX rightFront;
  WPI_TalonFX rightBack;

  // Look at the front of the robot and then rotate the robot 90 degrees clockwise
  // to determine left and right
  WPI_TalonFX middleLeft;
  WPI_TalonFX middleRight;

  // Declaring motor groups
  MotorControllerGroup rightMotorGroup;
  MotorControllerGroup leftMotorGroup;
  MotorControllerGroup centerMotorGroup;

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
    rightMotorGroup = new MotorControllerGroup(rightFront, rightBack);
    leftMotorGroup = new MotorControllerGroup(leftFront, leftBack);
    centerMotorGroup = new MotorControllerGroup(middleLeft, middleRight);

    // Invert motors
    leftMotorGroup.setInverted(false);
    rightMotorGroup.setInverted(false);
    middleLeft.setInverted(true);
    middleRight.setInverted(false);

    // Ramp speeds
    leftFront.configOpenloopRamp(Constants.RampLimiter);
    leftBack.configOpenloopRamp(Constants.RampLimiter);
    rightFront.configOpenloopRamp(Constants.RampLimiter);
    rightBack.configOpenloopRamp(Constants.RampLimiter);
    middleLeft.configOpenloopRamp(1);
    middleRight.configOpenloopRamp(1);

    // Creating Drive Movement
    drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  }

  // Move mid motor
  public void driveMiddleWithController(XboxController controller, double speedLimiter) {
    middleLeft.set(controller.getRawAxis(Constants.leftHorizontal) * speedLimiter);
    middleRight.set(controller.getRawAxis(Constants.leftHorizontal) * speedLimiter);
  }

  public void middleStop() {
    middleLeft.stopMotor();
    middleRight.stopMotor();
  }

  @Override
  public void periodic() {
  }

  public void driveWithController(XboxController controller, double speedLimiter) {
    drive.arcadeDrive(controller.getRawAxis(Constants.rightHorizontal) * speedLimiter,
        controller.getRawAxis(Constants.leftVertical) * speedLimiter * -1);
  }

}
