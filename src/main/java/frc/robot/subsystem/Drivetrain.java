// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.SmartDebug;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.extensions.Falcon;

public class Drivetrain extends SubsystemBase {

  // Declaring motors
  WPI_TalonFX leftFront;
  WPI_TalonFX leftBack;
  WPI_TalonFX rightFront;
  WPI_TalonFX rightBack;

  // Declaring motor groups
  MotorControllerGroup rightMotorGroup;
  MotorControllerGroup leftMotorGroup;

  // Gyro
  public static AHRS ahrs;

  // Drives
  DifferentialDrive drive;

  public Drivetrain() {

    // Creating Motor Objects
    leftFront = Falcon.createDefaultFalcon(Constants.leftFrontCanID);
    leftBack = Falcon.createDefaultFalcon(Constants.leftBackCanID);
    rightFront = Falcon.createDefaultFalcon(Constants.rightFrontCanID);
    rightBack = Falcon.createDefaultFalcon(Constants.rightBackCanID);

    // Creating Motor Groups
    rightMotorGroup = new MotorControllerGroup(rightFront, rightBack);
    leftMotorGroup = new MotorControllerGroup(leftFront, leftBack);

    // Creating Drive Object
    drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

    // Initializing gyro
    ahrs = new AHRS(SPI.Port.kMXP);

  }

  // Reset the gyro
  public void resetGyro() {
    ahrs.reset();
  }

  // get the gyro angle
  public double getGyroAngle() {
    SmartDebug.putDouble("Gyro Angle", ahrs.getAngle());
    return ahrs.getAngle();
  }

  // check if gyro is calibrating
  public boolean checkCalibrationStatus() {
    return ahrs.isCalibrating();
  }

  // Controls for the outside wheels using built in arcadeDrive
  public void driveWithController(XboxController controller, XboxController coDriver, double speedLimiter) {

    // setup the arcade drive
    drive.arcadeDrive(getRotationAxis(controller), getLogitudinalAxis(controller));

    // Make sure we aren't in an overtemp condition
    driveOverTempProtection();

  }

  // check the temp of the drive falcons and take action if needed
  private void driveOverTempProtection() {

    // read the temperature
    double rbTemp = rightBack.getTemperature();
    double rfTemp = rightFront.getTemperature();
    double lbTemp = leftBack.getTemperature();
    double lfTemp = leftFront.getTemperature();

    // push the temp to the dashboard - debug
    SmartDebug.putDouble("RightBack MotorTemp", rbTemp);
    SmartDebug.putDouble("RightFront MotorTemp", rfTemp);
    SmartDebug.putDouble("LeftBack MotorTemp", lbTemp);
    SmartDebug.putDouble("LeftFront MotorTemp", lfTemp);

    // if any of the drives are in overheat then we need to slow everything down to
    // the HOT speed
    if (rbTemp > Constants.driveToHot || rfTemp > Constants.driveToHot || lbTemp > Constants.driveToHot
        || lfTemp > Constants.driveToHot) {
      Constants.driveSpeedLimit = Constants.driveSpeedLimitHot;
      Constants.rotationSpeedLimit = Constants.rotationSpeedLimitHot;
      SmartDebug.putDouble("Drive Speed", Constants.driveSpeedLimit);
    }
    // in all other cases keep the default speed
    else {
      Constants.driveSpeedLimit = Constants.driveSpeedlimitDefault;
      Constants.rotationSpeedLimit = Constants.rotationSpeedLimitDefault;
      SmartDebug.putDouble("Drive Speed", Constants.driveSpeedLimit);
    }

  }

  // read and return the speed value for the rotational axis
  private double getRotationAxis(XboxController controller) {

    // Axis value * rotation speed limiter
    return controller.getRawAxis(Constants.rightHorizontal) * Constants.rotationSpeedLimit;

  }

  // read and return the speed value for the logitudinal axis (forward and
  // backwards)
  private double getLogitudinalAxis(XboxController controller) {

    // Axis value * translational speed limiter
    return controller.getRawAxis(Constants.leftVertical) * Constants.driveSpeedLimit;
  }

  // stop the outside four drive motors
  public void stopDriveMotors() {
    leftFront.stopMotor();
    leftBack.stopMotor();
    rightFront.stopMotor();
    rightBack.stopMotor();
  }

  // stop all drive motors
  public void stopAllDriveMotors() {
    // stopMiddleDriveMotors();
    stopDriveMotors();
  }

  public double getEncoderDistance() {
    double frontEncoder = (Math.abs(leftFront.getSelectedSensorPosition())
        + Math.abs(rightFront.getSelectedSensorPosition())) / 2;
    SmartDebug.putDouble("Front Encoders", frontEncoder);
    return frontEncoder;
  }

  public void resetEncoderPosition() {
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
  }

  public double inchesToTicks(double inches) {
    // 6 inch diameter wheel
    // 2048 ticks per motor rotation
    // 9.52:1 gear reduction

    // 18.8 inches of travel per rotation of the wheel
    // 9.52 motor rotations = 18.8 inches of travel
    // 19497 ticks per wheel rotation
    // 19497 ticks per 18.8 inches
    // 1037 ticks per inch of travel
    return inches * 1037;
  }

  public void drive_straight_gyro(double power) {
    // System.out.println("i exist lol");
    double error = -ahrs.getAngle(); // Our target angle is zero
    double turn_power = Constants.kPDt * error; // Kp
    drive.arcadeDrive(turn_power, power, false);

  }

  public void drive_angle_gyro(double power, double angle) {
    // System.out.println("i exist lol");
    double error = angle - ahrs.getAngle(); // Our target angle is zero
    double turn_power = Constants.kPDt * error; // Kp
    drive.arcadeDrive(turn_power, power, false);
  }

  public void drive_turn_gyro(double cst_turn_power, double angle) {
    // System.out.println("i exist lol");
    double error = angle - ahrs.getAngle(); // Our target angle is variable
    double turn_power = Constants.kPDt * error; // Kp
    if (Math.abs(error) > 2) {
      drive.arcadeDrive(cst_turn_power, 0, false); // if error, large, we turn at a constant speed.
    } else {
      drive.arcadeDrive(turn_power, 0, false); // if error, small, we fine tune with P. Good alternative if no I.
    }
  }

  @Override
  public void periodic() {
  }

}
