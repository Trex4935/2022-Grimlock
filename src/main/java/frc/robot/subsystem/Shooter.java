// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.BallColor;
import frc.robot.extensions.Helper;
import frc.robot.extensions.Limelight;
import frc.robot.extensions.SmartDebug;
import frc.robot.extensions.Falcon;

public class Shooter extends SubsystemBase {

  // Declare shooter motor
  WPI_TalonFX shooterMotor;

  // ** Creates a new Shooter. */
  public Shooter() {
    // Setup the shooter motor
    shooterMotor = Falcon.createDefaultFalcon(Constants.shooterMotorCanID);
    shooterMotor = Falcon.configurePID(shooterMotor, 0.25, 0, 0, 0.043);
    shooterMotor.setInverted(true);

  }

  // Read from the dashboard if we are shooting high goal or low goal
  private boolean getHighLowShooting() {
    // System.out.println(defaultTable.getEntry("Shoot Low"));
    return NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shoot Low").getBoolean(true);
  }

  // Read from the dashboard an adjustment values for the shooter RPM +-500 RPM
  private double getShooterAdjust() {
    return NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter Adjust").getDouble(0.0);
  }

  // converts the ticks to RPM values
  public double tickstoRPM(double ticks) {
    return (ticks * Constants.ticks2RPM);
  }

  // converts the RPM to ticks values
  public double rpmtoTicks(double rpm) {
    return (rpm / Constants.ticks2RPM);
  }

  // runs an adjusted version of a value set in constants with PID
  public void runShooterPID(BallColor color, double distance) {

    double targetTicks;
    boolean shootLow = getHighLowShooting();

    // Update the dashboard for drive feedback
    SmartDashboard.putString("Ball Color", color.toString());
    // SmartDashboard.putString("Alliance", allianceColor.toString());
    SmartDashboard.putBoolean("Target Seen", Limelight.getLimelightA());
    SmartDebug.putDouble("Shooter Motor Temp", shooterMotor.getTemperature());
    SmartDebug.putBoolean("Shootin Low", shootLow);

    // Get how fast we need the shooter moving
    targetTicks = getShootingTicks(color, distance, shootLow);

    // Set the motor speed to our calculated value
    shooterMotor.set(ControlMode.Velocity, targetTicks);

    // Return feedback if the shooter is at speed
    shooterAtSpeed(targetTicks, color);

    // Return feedback if the robot is within a acceptable shooting range
    atDistance(distance);
  }

  // Return ticks based on if we are low or high ... color ball or none
  private double getShootingTicks(BallColor color, double distance, boolean shootLow) {
    // If shooting low just use low speed
    if (shootLow) {
      SmartDebug.putDouble("Target RPM", Constants.shooterLowSpeed);
      return rpmtoTicks(Constants.shooterLowSpeed);
    }
    // Color is none then just keep spinning at the idle speed
    else if (color == BallColor.NONE) {
      SmartDebug.putDouble("Target RPM", Constants.shooterTargetSpeed);
      return rpmtoTicks(Constants.shooterTargetSpeed);
    }
    // Calculate our new target speed
    else {
      double speed = allianceSpeed(color, distance);
      SmartDebug.putDouble("Target RPM", tickstoRPM(speed));
      return speed;
    }
  }

  private void shooterAtSpeed(double targetTicks, BallColor color) {

    SmartDashboard.putNumber("Target_Ticks", targetTicks);
    SmartDashboard.putNumber("Shooter_Ticks", shooterMotor.getSelectedSensorVelocity());

    if (color == BallColor.NONE) {
      SmartDashboard.putBoolean("Shooter At Speed", false);
    } else {
      // Detect if we are within acceptable speed range
      if (Helper.RangeCompare(targetTicks + Constants.shooterRange, targetTicks - Constants.shooterRange,
          shooterMotor.getSelectedSensorVelocity())) {
        SmartDashboard.putBoolean("Shooter At Speed", true);

      } else {
        SmartDashboard.putBoolean("Shooter At Speed", false);
      }
    }
  }

  private void atDistance(double distance) {

    // Detect if we are within acceptable distance range
    // Return true or false for usage with the magazine bypass
    if (Helper.RangeCompare(Constants.maximumShootDistance + 6, Constants.minimumShootDistance - 6, distance)) {

      SmartDashboard.putBoolean("Robot at Distance", true);
    } else {

      SmartDashboard.putBoolean("Robot at Distance", false);
    }
  }

  // determine the shooter speed based on distance ball color and alliance
  private double allianceSpeed(BallColor color, double distance) {
    // Red & Red == speed by distance
    if (color == BallColor.RED && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      Constants.shooterTargetSpeed = getSpeedSetPointRPM(distance);
      return rpmtoTicks(Constants.shooterTargetSpeed);
    }
    // Blue & Blue == speed by distance
    else if (color == BallColor.BLUE && DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      Constants.shooterTargetSpeed = getSpeedSetPointRPM(distance);
      return rpmtoTicks(Constants.shooterTargetSpeed);
    }
    // all other cases low speed shot!
    else {
      return rpmtoTicks(Constants.shooterLowSpeed);
    }
  }

  // Stop shooter motor
  public void stopShooterMotor() {
    shooterMotor.stopMotor();
  }

  // Based on the best fit formula return the target RPM
  public double getSpeedSetPointRPM(double distance) {

    // 7ft = 2600 rpm
    // 8ft == 2600 rpm
    // 13ft = 3000 rpm
    // 15.5 = 3500 rpm
    // RPM = y = 0.1076x2 - 20.291x + 3549.8
    return (0.1076 * (distance * distance)) - (20.291 * distance) + 3549.8 + getShooterAdjust();
  }

  public void shooting() {
    shooterMotor.set(Constants.shooterSpeed);

  }

  public void shooterShootLow() {
    shooterMotor.set(Constants.shooterLowSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
