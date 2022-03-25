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
    SmartDashboard.putBoolean("Target Seen", Limelight.getLimelightA());
    SmartDashboard.putNumber("Shooter", shooterMotor.getTemperature());
    System.out.println(Constants.forceShoot);
    // Take in Ball Color and process magazine activity and shooter speed
    // Code needs to be here due to handling of the NONE state
    switch (color) {
      case NONE:

        // Determine the target ticks based on color distance and high/low
        targetTicks = getShootingTicks(color, distance);

        shooterMotor.set(ControlMode.Velocity, targetTicks);

        // check if we are at speed and update the dashboard
        return shooterConditionsMet(color, targetTicks, distance);

      case RED:
        // Determine the target ticks based on color distance and high/low
        targetTicks = getShootingTicks(color, distance);

        // Set the motor speed
        shooterMotor.set(ControlMode.Velocity, targetTicks);

        // Detect if we are within acceptable speed range
        // Return true or false for usage with the magazine bypass
        return shooterConditionsMet(color, targetTicks, distance);

      case BLUE:
        // Determine the target ticks based on color distance and high/low
        targetTicks = getShootingTicks(color, distance);

        // Set the motor speed
        shooterMotor.set(ControlMode.Velocity, targetTicks);

        // Detect if we are within acceptable speed range
        // Return true or false for usage with the magazine bypass
        return shooterConditionsMet(color, targetTicks, distance);

      default:
        // Determine the target ticks based on color distance and high/low
        targetTicks = getShootingTicks(color, distance);

        // Set the motor speed
        shooterMotor.set(ControlMode.Velocity, targetTicks);

        // check if we are at speed and update the dashboard
        return shooterConditionsMet(color, targetTicks, distance);

    }
  }

  // validate that we are ready to shoot and override if needed
  private boolean shooterConditionsMet(BallColor color, double targetTicks, double distance) {

    // Get the color of the alliance we are on
    DriverStation.Alliance allianceColor = DriverStation.getAlliance();

    // If we are shooting low then we ONLY shoot if forceShoot is true!
    if (Constants.shootingLow) {
      return Constants.forceShoot;
    }
    // If we are shooting high &
    // If the ball color matches the alliance color then test if we are ready to
    // shoot
    else if (color == BallColor.RED && allianceColor == DriverStation.Alliance.Red
        || color == BallColor.BLUE && allianceColor == DriverStation.Alliance.Blue) {
      return (Limelight.getLimelightA() && shooterAtSpeed(targetTicks, color) && atDistance(distance)
          || Constants.forceShoot);
    }
    // all other cases low speed shot as soon as the shooter is ready
    else {
      return shooterAtSpeed(targetTicks, color);
    }
  }

  // Return ticks based on if we are low or high ... color ball or none
  private double getShootingTicks(BallColor color, double distance) {
    // If shooting low just use low speed
    if (Constants.shootingLow) {
      return rpmtoTicks(Constants.shooterLowSpeed);
    }

    // if none use idle speed
    // any thing else => calculate speed
    else {
      switch (color) {
        case NONE:
          return Constants.shooterIdleSpeed;
        default:
          return allianceSpeed(color, distance);
      }

    }
  }

  private boolean shooterAtSpeed(double targetTicks, BallColor color) {

    SmartDashboard.putNumber("Target_Ticks", targetTicks);
    SmartDashboard.putNumber("Shooter_Ticks", shooterMotor.getSelectedSensorVelocity());

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
    // if we are shooting low color doesn't matter just return the low ball color
    if (Constants.shootingLow) {
      return rpmtoTicks(Constants.shooterLowSpeed);
    } else {
      // Red & Red == speed by distance
      if (color == BallColor.RED && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        Constants.shooterIdleSpeed = getSpeedSetPoint(distance);
        return rpmtoTicks(Constants.shooterIdleSpeed);
      }
      // Blue & Blue == speed by distance
      else if (color == BallColor.BLUE && DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        Constants.shooterIdleSpeed = getSpeedSetPoint(distance);
        return rpmtoTicks(Constants.shooterIdleSpeed);
      }
      // all other cases low speed shot!
      else {
        return rpmtoTicks(Constants.shooterLowSpeed);
      }
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

  public void shooterShootLow() {
    shooterMotor.set(Constants.shooterLowSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
