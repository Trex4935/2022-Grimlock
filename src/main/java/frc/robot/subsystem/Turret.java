// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class Turret extends PIDSubsystem {

  WPI_TalonFX turretShooter;
  PWMSparkMax turretRotation;

  Limelight limelight;

  // magnet sensors
  private static DigitalInput leftMagLimit;
  private static DigitalInput middleMag;
  private static DigitalInput rightMagLimit;

  /** Creates a new turret. */
  public Turret() {
    super(
        // The PIDController used by the subsystem
        new PIDController(1, 0, 0));
    setSetpoint(0);
    getController().setTolerance(0.05);

    // Init motors
    turretShooter = new WPI_TalonFX(Constants.turretShooterCanID);
    // turretRotation = new WPI_TalonFX(Constants.turretRotationCanID);
    turretRotation = new PWMSparkMax(Constants.turretRotationPWMID);

    limelight = new Limelight();

  }

  // Uses limelight output to move rotation motor directly
  // Uses magnets to detect if it is and prevent it from rotating too far left or
  // right
  public void turnOnSimpleAutoAim() {
    if (leftMagLimit.get() == true && (limelight.getLimelightX() / 270) <= 0) {
      turretRotation.set(0);
    } else if (rightMagLimit.get() == true && (limelight.getLimelightX() / 270) >= 0) {
      turretRotation.set(0);
    } else {
      turretRotation.set(limelight.getLimelightX() / 270);
    }
  }

  // Returns the rotation motor to the middle by turning it to the right until it
  // reaches the middle.
  // If it rotates all the way to the far right instead the motor will reverse
  // directions until it reaches the middle.
  // When the rotation motor reaches the middle it will stop moving.
  public boolean returnToMiddle() {
    if (rightMagLimit.get() == false) {
      if (middleMag.get() == false) {
        return false;
      } else {
        turretRotation.set(0);
        return true;
      }
    } else {
      Constants.returnToMiddleSpeed = (Math.abs(Constants.returnToMiddleSpeed) * -1);
      turretRotation.set(Constants.returnToMiddleSpeed);
      return false;
    }
  }

  // runs the turret shooter with a given speed
  public void runTurretShooter(double turretShooterSpeed) {
    turretShooter.set(turretShooterSpeed);
  }

  // Moves the rotation motor based on controller input
  public void aimWithController(XboxController controller) {

    double LT = controller.getRawAxis(Constants.leftTrigger);
    double RT = controller.getRawAxis(Constants.rightTrigger) * -1;
    turretRotation.set((LT + RT) / 25);
  }

  // Stop the rotation motor
  public void stopRotationMotor() {
    turretRotation.stopMotor();
  }

  public double getDistance() {
    double angle2 = limelight.getLimelightY();
    double d = (Constants.h2 - Constants.h1) / Math.tan(Constants.angle1 + angle2);
    return d;
  }

  public void shootBallWithVision() {
    double motorSpeed = Constants.shooterA * getDistance() + Constants.shooterB;
    turretShooter.set(TalonFXControlMode.Velocity, motorSpeed);
  }

  // Stop shooter motor
  public void stopShooterMotor() {
    turretShooter.stopMotor();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    turretRotation.set(output);
  }

  @Override
  public double getMeasurement() {
    double angle = limelight.getLimelightX() / 270;
    // Return the process variable measurement here
    return angle;
  }
}

// hello