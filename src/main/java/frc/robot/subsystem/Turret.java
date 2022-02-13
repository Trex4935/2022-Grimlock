// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.extensions.Limelight;
import edu.wpi.first.wpilibj.DigitalInput;

public class Turret extends PIDSubsystem {

  // Motors
  PWMSparkMax turretRotation;

  // magnet sensors
  private static DigitalInput leftMagLimit;
  private static DigitalInput middleMag;
  private static DigitalInput rightMagLimit;

  /** Creates a new turret. */
  public Turret() {
    // The PIDController used by the subsystem
    super(new PIDController(1, 0, 0));
    setSetpoint(0);
    getController().setTolerance(0.05);

    // Init motor
    turretRotation = new PWMSparkMax(Constants.turretRotationPWMID);

    leftMagLimit = new DigitalInput(Constants.leftMagLimitID);
    middleMag = new DigitalInput(Constants.middleMagID);
    rightMagLimit = new DigitalInput(Constants.rightMagLimitID);

  }

  // Uses limelight output to move rotation motor directly
  // Uses magnets to detect if it is and prevent it from rotating too far left or
  // right
  public void turnOnSimpleAutoAim() {
    if (leftMagLimit.get() == true && (Limelight.getLimelightX() / 270) <= 0) {
      turretRotation.set(0);
    } else if (rightMagLimit.get() == true && (Limelight.getLimelightX() / 270) >= 0) {
      turretRotation.set(0);
    } else {
      turretRotation.set(Limelight.getLimelightX() / 270);
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

  // Moves the rotation motor based on controller input
  public void aimWithController(XboxController controller) {

    double triggerValue = (controller.getRawAxis(Constants.leftTrigger) * -1)
        + controller.getRawAxis(Constants.rightTrigger);

    turretRotation.set((triggerValue) / 10);

    if (leftMagLimit.get() == false && (triggerValue) >= 0) {
      turretRotation.stopMotor();
    } else if (rightMagLimit.get() == false && (triggerValue) <= 0) {
      turretRotation.stopMotor();
    } else {
    }
  }

  // Stop the rotation motor
  public void stopRotationMotor() {
    turretRotation.stopMotor();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use output to take action on the turret motor (i.e. make it move)
    turretRotation.set(output);
  }

  @Override
  public double getMeasurement() {
    // gather and return the value that will be used to calculate the PID (i.e.
    // sensor output)
    double angle = Limelight.getLimelightX() / 270;
    return angle;
  }
}

// hello