// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  public double getLimelightX() {

    double x = tx.getDouble(0.0);
    return x;
  }

  public double getLimelightY() {

    double y = ty.getDouble(0.0);
    return y;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
