// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

// Imports
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.multiplexedColorSensor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {

  // Declaring motors
  WPI_TalonFX leftFront;
  WPI_TalonFX leftBack;
  WPI_TalonFX rightFront;
  WPI_TalonFX rightBack;

  // Look at the front of the robot and then rotate the robot 90 degrees clockwise
  // to determine middle left and right
  WPI_TalonFX middleLeft;
  WPI_TalonFX middleRight;

  // Declaring motor groups
  MotorControllerGroup rightMotorGroup;
  MotorControllerGroup leftMotorGroup;
  MotorControllerGroup centerMotorGroup;

  // Gyro
  public static AHRS ahrs;

  // Drives
  DifferentialDrive drive;

  // Sensors
  private multiplexedColorSensor lineSensorLeft;
  private multiplexedColorSensor lineSensorRight;

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

    // set all motors to factory default to avoid possible config issues
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();
    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();
    middleLeft.configFactoryDefault();
    middleRight.configFactoryDefault();

    // Invert motors as needed
    leftMotorGroup.setInverted(false);
    rightMotorGroup.setInverted(false);
    middleLeft.setInverted(true);
    middleRight.setInverted(false);

    // Ramp speeds, how fast the motors take to get to full speed
    leftFront.configOpenloopRamp(Constants.outsideRampLimiter);
    leftBack.configOpenloopRamp(Constants.outsideRampLimiter);
    rightFront.configOpenloopRamp(Constants.outsideRampLimiter);
    rightBack.configOpenloopRamp(Constants.outsideRampLimiter);
    middleLeft.configOpenloopRamp(Constants.middleRampLimiter);
    middleRight.configOpenloopRamp(Constants.middleRampLimiter);

    // Set brake mode
    leftFront.setNeutralMode(Constants.outsideBrakeMode);
    leftBack.setNeutralMode(Constants.outsideBrakeMode);
    rightFront.setNeutralMode(Constants.outsideBrakeMode);
    rightBack.setNeutralMode(Constants.outsideBrakeMode);
    middleLeft.setNeutralMode(Constants.middleBreakMode);
    middleRight.setNeutralMode(Constants.middleBreakMode);

    // Creating Drive Object
    drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

    // Initializing gyro
    ahrs = new AHRS(SPI.Port.kMXP);

    // Sensors
    lineSensorLeft = new multiplexedColorSensor(I2C.Port.kOnboard, 3);
    lineSensorRight = new multiplexedColorSensor(I2C.Port.kOnboard, 4);
  }

  // Reset the gyro
  public void resetGyro() {
    ahrs.reset();
  }

  // get the gyro angle
  public double getGyroAngle() {
    return ahrs.getAngle();
  }

  // check if gyro is calibrating
  public boolean checkCalibrationStatus() {
    return ahrs.isCalibrating();
  }

  // Move center motors
  public void driveMiddleWithController(XboxController controller, double speedLimiter) {
    middleLeft.set(controller.getRawAxis(Constants.leftHorizontal) * speedLimiter);
    middleRight.set(controller.getRawAxis(Constants.leftHorizontal) * speedLimiter);
  }

  // Controls for the outside wheels using built in arcadeDrive
  public void driveWithController(XboxController controller, double speedLimiter) {
    drive.arcadeDrive(controller.getRawAxis(Constants.rightHorizontal) * speedLimiter,
        controller.getRawAxis(Constants.leftVertical) * speedLimiter * -1);
  }

  public void moveToLineLeft() {
    if (Constants.sensorIRBlackValue >= lineSensorLeft.getIR()) {
      leftFront.stopMotor();
      leftBack.stopMotor();
    } else {
      leftFront.set(.25);
      leftBack.set(.25);
    }
  }

  public void moveToLineRight() {
    if (Constants.sensorIRBlackValue >= lineSensorRight.getIR()) {
      rightFront.stopMotor();
      rightBack.stopMotor();
    } else {
      rightFront.set(.25);
      rightBack.set(.25);
    }
  }

  // Stop the center motors
  public void stopMiddleDriveMotors() {
    middleLeft.stopMotor();
    middleRight.stopMotor();
  }

  // stop the outside four drive motors
  public void stopOutsideDriveMotors() {
    leftFront.stopMotor();
    leftBack.stopMotor();
    rightFront.stopMotor();
    rightBack.stopMotor();
  }

  // stop all drive motors
  public void stopAllDriveMotors() {
    stopMiddleDriveMotors();
    stopOutsideDriveMotors();
  }

  @Override
  public void periodic() {
  }

}
