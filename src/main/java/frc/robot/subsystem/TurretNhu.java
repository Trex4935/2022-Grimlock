// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.IFollower;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxRelativeEncoder;
import frc.robot.extensions.Helper;
import frc.robot.extensions.Limelight;
import frc.robot.extensions.FlippedDIO;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;

public class TurretNhu extends SubsystemBase {
  /** Creates a new turrentNHU. */

  CANSparkMax turretRotation;
  private RelativeEncoder encoder;

  private static FlippedDIO leftMagLimit;
  private static FlippedDIO middleMag;
  private static FlippedDIO rightMagLimit;

  public TurretNhu() {
    turretRotation = new CANSparkMax(0, MotorType.kBrushless);
    encoder = turretRotation.getEncoder();

    leftMagLimit = new FlippedDIO(Constants.leftMagLimitID);
    middleMag = new FlippedDIO(Constants.middleMagID);
    rightMagLimit = new FlippedDIO(Constants.rightMagLimitID);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double tt = ticksToRotate();
    double en = encoder.getPosition();

    if (tt > 0) {
      turretRotation.set(0.1);
    } else if (tt < 0) {
      ;
      turretRotation.set(-0.1);
    } else {
      turretRotation.stopMotor();
    }
    {
    }
  }
}

/*
 * 
 * Direction of movement based on tt + or -
 * Move turrent motor in that direction at 10%
 * if tt in range X do nothing
 * 
 * 
 */