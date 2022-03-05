// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.BallColor;
import frc.robot.extensions.Helper;

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
  public boolean runShooterPID(BallColor color, double distance) {

    // switch statement to decide what to do depending on ball color
    // currently placeholder values
    System.out.println(color.toString());
    double targetTicks;
    double shooterSpeed;

    // Calculate the right shooter speed, per distance
    shooterSpeed = getSpeedSetPoint(distance);

    // Take in Ball Color and process magazine activity and shooter speed
    // Code needs to be here due to handling of the NONE state
    switch (color) {
      case NONE:
        // System.out.println("NONE");
        shooterMotor.set(ControlMode.Velocity, rpmtoTicks(shooterSpeed));
        return false;
      case RED:
        // System.out.println("RED");

        targetTicks = rpmtoTicks(1000);
        shooterMotor.set(ControlMode.Velocity, targetTicks);

        // Detect if we are within acceptable speed range
        // Return true or false for usage with the magazine bypass
        if (Helper.RangeCompare(targetTicks + Constants.shooterRange, targetTicks - Constants.shooterRange,
            shooterMotor.getSelectedSensorVelocity())) {
          return true;
        } else {
          return false;
        }

      case BLUE:
        // System.out.println("BLUE");

        targetTicks = rpmtoTicks(3200);
        shooterMotor.set(ControlMode.Velocity, targetTicks);

        // Detect if we are within acceptable speed range
        // Return true or false for usage with the magazine bypass
        if (Helper.RangeCompare(targetTicks + Constants.shooterRange, targetTicks - Constants.shooterRange,
            shooterMotor.getSelectedSensorVelocity())) {
          return true;
        } else {
          return false;
        }

      default:
        // System.out.println("defaultdefault");
        shooterMotor.set(ControlMode.Velocity, rpmtoTicks(shooterSpeed));
        return false;
    }
  }

  // Determine motor speed based on distance and linear equation for speed vs
  // distance
  public void shootBallWithVision(double distance) {
    // Linear equation relating motor speed to distance
    double motorSpeed = Constants.shooterA * distance + Constants.shooterB;
    shooterMotor.set(TalonFXControlMode.Velocity, motorSpeed);
  }

  // Stop shooter motor
  public void stopShooterMotor() {
    shooterMotor.stopMotor();
  }

  public double getSpeedSetPoint(double distance) {
    double motorSpeed = Constants.shooterA * distance + Constants.shooterB;
    return motorSpeed;
  }

  public double getSpeed() {
    return 1;// TODO
  }

  public double getPosition() {
    return 1;// TODO
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
