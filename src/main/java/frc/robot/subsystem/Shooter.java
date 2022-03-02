// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  WPI_TalonFX shooterMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new WPI_TalonFX(Constants.shooterMotorCanID);
    shooterMotor.setInverted(true);

    //
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
    config.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers,
                                                       // in sec
    config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
    shooterMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder

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

    shooterMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocity_Shooter.getkP(), Constants.kTimeoutMs);
    shooterMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocity_Shooter.getkI(), Constants.kTimeoutMs);
    shooterMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocity_Shooter.getkD(), Constants.kTimeoutMs);
    shooterMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocity_Shooter.getkF(), Constants.kTimeoutMs);
  }

  // runs the turret shooter with a given speed
  public boolean runShooter(double turretShooterSpeed) {
    shooterMotor.set(turretShooterSpeed);
    if ((shooterMotor.getSelectedSensorVelocity() >= (turretShooterSpeed - 0.1))
        && (shooterMotor.getSelectedSensorVelocity() <= (turretShooterSpeed + 0.1))) {
      return Constants.shooterToSpeed = true;
    } else {
      return Constants.shooterToSpeed = false;
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
