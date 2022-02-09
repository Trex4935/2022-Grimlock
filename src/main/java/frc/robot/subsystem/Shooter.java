// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  WPI_TalonFX shooterMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new WPI_TalonFX(Constants.shooterMotorCanID);
    shooterMotor.setInverted(true);

    // Set motor limits
    //// normal output forward and reverse = 0% ... i.e. stopped
    shooterMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    shooterMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);

    //// Max output forward and reverse = 100%
    shooterMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    shooterMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    // PID configs
    shooterMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocity_Shooter.getkP(), Constants.kTimeoutMs);
    shooterMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocity_Shooter.getkI(), Constants.kTimeoutMs);
    shooterMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocity_Shooter.getkD(), Constants.kTimeoutMs);
    shooterMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocity_Shooter.getkF(), Constants.kTimeoutMs);
  }

  // runs the turret shooter with a given speed
  public void runShooter(double turretShooterSpeed) {
    shooterMotor.set(turretShooterSpeed);

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
