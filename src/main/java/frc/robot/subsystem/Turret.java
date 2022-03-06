// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.FlippedDIO;
import frc.robot.extensions.Helper;
import frc.robot.extensions.Limelight;

public class Turret extends SubsystemBase {

  // PID
  PIDController turretPID = new PIDController(0.01, 0, 0);

  // Motors
  PWMSparkMax turretRotation;

  // magnet sensors
  private static FlippedDIO leftMagLimit;
  private static FlippedDIO middleMag;
  private static FlippedDIO rightMagLimit;

  /** Creates a new turret. */
  public Turret() {
    /*
     * // The PIDController used by the subsystem
     * super(new PIDController(.01, 0, 0));
     * setSetpoint(0);
     * getController().setTolerance(0.05);
     */

    // Init motor
    turretRotation = new PWMSparkMax(Constants.turretRotationPWMID);
    turretRotation.setInverted(true);

    leftMagLimit = new FlippedDIO(Constants.leftMagLimitID);
    middleMag = new FlippedDIO(Constants.middleMagID);
    rightMagLimit = new FlippedDIO(Constants.rightMagLimitID);

  }

  // If the turret PID goes over 20% (.2 or -.2), bring it back to 20%
  public double turretThreshold() {
    double motorOutput = turretPID.calculate(Limelight.getLimelightX(), 0);
    if (Helper.RangeCompare(.2, -.2, motorOutput)) {
      return motorOutput;
    } else {
      if (motorOutput < 0) {
        return -0.2;
      } else {
        return 0.2;
      }
    }
  }

  // Using PID values, the turret autolocks on a target and turns based off of
  // where the target is
  public void turnOnPIDAutoAim() {
    turretRotation.set(turretThreshold());
  }

  // The limelight's on target > returns true or false
  public boolean limelightTarget() {
    return Helper.RangeCompare(2, -2, Limelight.getLimelightX() / 270);
  }

  public boolean returnToMiddle() {

    // System.out.println(Constants.returnToMiddleSpeed);

    // Returns the rotation motor to the middle by turning it to the right until it
    // reaches the middle.
    // If it rotates all the way to the far right instead the motor will reverse
    // directions until it reaches the middle.
    // When the rotation motor reaches the middle it will stop moving.
    if (rightMagLimit.get() == true) {
      Constants.returnToMiddleSpeed = Constants.returnToMiddleSpeedLeft;
      turretRotation.set(Constants.returnToMiddleSpeed);
      return true;
    } else if (middleMag.get() == true) {
      Constants.returnToMiddleSpeed = Math.abs(Constants.returnToMiddleSpeed);
      turretRotation.set(0);
      return false;
    } else {
      turretRotation.set(Constants.returnToMiddleSpeed);
      return false;
    }
  }

  // Moves the rotation motor based on controller input
  public void aimWithController(XboxController controller) {

    // Pull in values from left and right trigger and normalize them
    double triggerValue = (controller.getRawAxis(Constants.leftTrigger) * -1)
        + controller.getRawAxis(Constants.rightTrigger);

    // ensure we stop at the right limit switches
    if (leftMagLimit.get() == false && (triggerValue) <= 0) {
      turretRotation.stopMotor();
    } else if (rightMagLimit.get() == false && (triggerValue) >= 0) {
      turretRotation.stopMotor();
    } else {
      // Divide input by 10 to get a max of 0.1
      turretRotation.set((triggerValue) * 0.2);

    }

  }

  // Stop the rotation motor
  public void stopRotationMotor() {
    turretRotation.stopMotor();
  }

}

// hello