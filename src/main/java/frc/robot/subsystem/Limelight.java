// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

  public NetworkTableEntry tx;
  public NetworkTableEntry ty;
  public NetworkTable table;

  /** Creates a new Limelight. */
  public Limelight() {

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");

  }

  // Pull TX from the limelight
  public double getLimelightX() {

    double x = tx.getDouble(0.0);
    return x;
  }

  // Pull TY from the limelight
  public double getLimelightY() {

    double y = ty.getDouble(0.0);
    return y;
  }

  // Distance to the target
  public double getDistance() {
    double angle2 = getLimelightY();
    double distance = (Constants.h2 - Constants.h1) / Math.tan(Constants.angle1 + angle2);
    return distance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}