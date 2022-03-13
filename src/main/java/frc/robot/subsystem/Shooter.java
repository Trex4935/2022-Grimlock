// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.BallColor;
import frc.robot.extensions.Helper;
import frc.robot.extensions.Limelight;

public class Shooter extends SubsystemBase {

  // Declare shooter motor
  WPI_TalonFX shooterMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    // Setup the shooter motor
    shooterMotor = new WPI_TalonFX(Constants.shooterMotorCanID);
    shooterMotor.setInverted(true);

    // Setup shooter configuration
    TalonFXConfiguration config = new TalonFXConfiguration();
    initMotorController(config);
  }

  private void initMotorController(TalonFXConfiguration config) {

    // Config default settings for the motor
    shooterMotor.configFactoryDefault();

    // Set motor brake mode
    shooterMotor.setNeutralMode(NeutralMode.Brake);

    // Set motor limits
    //// normal output forward and reverse = 0% ... i.e. stopped
    shooterMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    shooterMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);

    //// Max output forward and reverse = 100%
    shooterMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    shooterMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    // PID configs
    // setting up the pid
    shooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);

    // Set kP(proportional); kI(Integral); kD(differential); kF(FeedForward)
    shooterMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocity_Shooter.getkP(), Constants.kTimeoutMs);
    shooterMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocity_Shooter.getkI(), Constants.kTimeoutMs);
    shooterMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocity_Shooter.getkD(), Constants.kTimeoutMs);
    shooterMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocity_Shooter.getkF(), Constants.kTimeoutMs);

    shooterMotor.config_IntegralZone(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);
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
  public boolean runShooterPID(BallColor color, double distance, DriverStation.Alliance allianceColor) {
    // switch statement to decide what to do depending on ball color
    // currently placeholder values
    // System.out.println(color.toString());
    double targetTicks;
    SmartDashboard.putString("Color", color.toString());
    SmartDashboard.putString("Alliance", allianceColor.toString());
    SmartDashboard.putBoolean("Target Seen", Limelight.getLimelightV());
    // Take in Ball Color and process magazine activity and shooter speed
    // Code needs to be here due to handling of the NONE state
    switch (color) {
      case NONE:
        targetTicks = rpmtoTicks(Constants.shooterIdleSpeed);

        // System.out.println("NONE");
        shooterMotor.set(ControlMode.Velocity, targetTicks);

        // check if we are at speed and update the dashboard
        return (Limelight.getLimelightV() && shooterAtSpeed(targetTicks, color) && atDistance(distance));

      case RED:
        // System.out.println("RED");

        // Determined the # of ticks based on ball color and distance
        targetTicks = allianceSpeed(BallColor.RED, distance);
        // System.out.println(targetTicks);

        // Set the motor speed
        shooterMotor.set(ControlMode.Velocity, targetTicks);

        // Detect if we are within acceptable speed range
        // Return true or false for usage with the magazine bypass
        if (allianceColor == DriverStation.Alliance.Red) {
          return (Limelight.getLimelightV() && shooterAtSpeed(targetTicks, color) && atDistance(distance));
        } else {
          return shooterAtSpeed(targetTicks, color);
        }
      case BLUE:
        // System.out.println("BLUE");

        // Determined the # of ticks based on ball color and distance
        targetTicks = allianceSpeed(BallColor.BLUE, distance);
        // System.out.println("----------------");
        // System.out.println(targetTicks);

        // Set the motor speed
        shooterMotor.set(ControlMode.Velocity, targetTicks);
        // System.out.println(shooterMotor.getSelectedSensorVelocity());

        // Detect if we are within acceptable speed range
        // Return true or false for usage with the magazine bypass
        if (allianceColor == DriverStation.Alliance.Blue) {
          return (Limelight.getLimelightV() && shooterAtSpeed(targetTicks, color) && atDistance(distance));
        } else {
          return shooterAtSpeed(targetTicks, color);
        }
      default:
        targetTicks = rpmtoTicks(Constants.shooterIdleSpeed);

        // System.out.println("NONE");
        shooterMotor.set(ControlMode.Velocity, targetTicks);

        // check if we are at speed and update the dashboard
        return (Limelight.getLimelightV() && shooterAtSpeed(targetTicks, color) && atDistance(distance));
    }
  }

  private boolean shooterAtSpeed(double targetTicks, BallColor color) {

    SmartDashboard.putNumber("Target Ticks", targetTicks);
    SmartDashboard.putNumber("Tp100", shooterMotor.getSelectedSensorVelocity());

    if (color == BallColor.NONE) {
      SmartDashboard.putBoolean("Shooter At Speed", false);
      return false;
    } else {
      // Detect if we are within acceptable speed range
      // Return true or false for usage with the magazine bypass
      if (Helper.RangeCompare(targetTicks + Constants.shooterRange, targetTicks - Constants.shooterRange,
          shooterMotor.getSelectedSensorVelocity())) {
        SmartDashboard.putBoolean("Shooter At Speed", true);
        return true;

      } else {
        SmartDashboard.putBoolean("Shooter At Speed", false);
        return false;
      }
    }
  }

  private boolean atDistance(double distance) {

    // Detect if we are within acceptable distance range
    // Return true or false for usage with the magazine bypass
    if (Helper.RangeCompare(Constants.maximumShootDistance + 6, Constants.minimumShootDistance - 6, distance)) {

      SmartDashboard.putBoolean("Robot at Distance", true);
      return true;
    } else {

      SmartDashboard.putBoolean("Robot at Distance", false);
      return false;
    }
  }

  // determine the shooter speed based on distance ball color and alliance
  private double allianceSpeed(BallColor color, double distance) {
    // Red & Red == speed by distance
    if (color == BallColor.RED && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      Constants.shooterIdleSpeed = getSpeedSetPoint(distance);
      return rpmtoTicks(getSpeedSetPoint(distance));
    }
    // Blue & Blue == speed by distance
    else if (color == BallColor.BLUE && DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      Constants.shooterIdleSpeed = getSpeedSetPoint(distance);
      return rpmtoTicks(getSpeedSetPoint(distance));
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

  public double getSpeedSetPoint(double distance) {
    // RPM = 3.7037 * distance + 2722.2
    return 3.7037 * distance + 2722.2;
  }

  public void shooting() {
    shooterMotor.set(Constants.shooterSpeed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
