// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.FlippedDIO;
import frc.robot.extensions.Helper;
import frc.robot.extensions.Limelight;
import frc.robot.extensions.SmartDebug;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class Turret extends SubsystemBase {

  // PID
  // PIDController turretPID = new PIDController(0.03, 0.0, 0);
  SparkMaxPIDController turretPID;

  // Motors
  CANSparkMax turretRotation;
  private RelativeEncoder encoder;

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

    // turretPID.setTolerance(0.05);
    // turretPID.setIntegratorRange(-30, 30);

    // Init motor
    // turretRotation = new CANSparkMax(Constants.turretRotationPWMID);\

    // CAN SPARK MAX!! and pid controller and encoder
    turretRotation = new CANSparkMax(0, MotorType.kBrushless);
    encoder = turretRotation.getEncoder(NEOEncoder.Type.kQuadrature, 4096);

    turretRotation.restoreFactoryDefaults();

    turretPID = turretRotation.getPIDController();
    turretPID.setFeedbackDevice(encoder);

    turretPID.setP(0.03);
    turretPID.setI(0.0);
    turretPID.setD(0);

    // turretRotation.setInverted(true);

    leftMagLimit = new FlippedDIO(Constants.leftMagLimitID);
    middleMag = new FlippedDIO(Constants.middleMagID);
    rightMagLimit = new FlippedDIO(Constants.rightMagLimitID);

  }

  // Using PID values, the turret autolocks on a target and turns based off of
  // where the target is
  public void turnOnPIDAutoAim(XboxController coDrivController) {

    // System.out.println(coDrivController.getRawAxis(Constants.leftTrigger));

    // If the controller is trying to turn the turret use that otherwise use the PID
    // if (coDrivController.getRawAxis(Constants.leftTrigger) > 0.1
    // || coDrivController.getRawAxis(Constants.rightTrigger) > 0.1) {
    if (coDrivController.getRawAxis(Constants.leftTrigger) > 0.1
        || coDrivController.getRawAxis(Constants.rightTrigger) > 0.1) {
      // System.out.println("True");
      aimWithController(coDrivController);
    }
    // use the PID to move the turret
    else {
      // Get the speed that we are going to run the motor
      double tt = turretPID.calculate(Limelight.getLimelightX(), 0);
      SmartDebug.putDouble("Turret Motor Output", tt);

      // Handle the limit switches to make sure we don't over rotate
      if (leftMagLimit.get() == true && tt >= 0) {
        turretRotation.stopMotor();
      } else if (rightMagLimit.get() == true && tt <= 0) {
        turretRotation.stopMotor();
      } else {
        turretRotation.set(tt);
      }
    }
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
  public void aimWithController(XboxController coDriverController) {

    // Pull in values from left and right trigger and normalize them
    double triggerValue = (coDriverController.getRawAxis(Constants.rightTrigger) * -1)
        + (coDriverController.getRawAxis(Constants.leftTrigger));

    // System.out.println(triggerValue);

    // ensure we stop at the right limit switches
    if (leftMagLimit.get() == true && (triggerValue) >= 0) {
      turretRotation.stopMotor();
    } else if (rightMagLimit.get() == true && (triggerValue) <= 0) {
      turretRotation.stopMotor();
    } else {
      // Divide input by 10 to get a max of 0.1
      turretRotation.set((triggerValue) * 0.7);

    }

  }

  // Stop the rotation motor
  public void stopRotationMotor() {
    turretRotation.stopMotor();
  }

  public double ticksToRotate() {

    // small gear = 16 teeth
    // 16:1
    // 110 teeth
    // 360 degrees circle
    // 768 ticks to turn big gear

    // 48 * 16 = 768 (ticks to turn big gear once),
    // 360/110 = 3.27 degrees per big gear turn,
    // 16 * 3.27 = one small gear turning resulting in degrees,
    // 52.32/768 = 0.068 = degrees for one tick,
    // divide degrees by 0.068 = 1 tick for amount of ticks
    // then run encoder to amount of ticks calculated
    // ???????

    // 1.09

    double ticks = 1;
    return (ticks * Limelight.getLimelightX());
  }

  public void turretTurn() {
    ticksToRotate();
  }
}

// hello