// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class TurretPID extends PIDSubsystem {

  CANSparkMax turretRotation;

  Limelight limelight;

  public TurretPID() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

        limelight = new Limelight();
      turretRotation = new CANSparkMax(Constants.turretRotationCanID, MotorType.kBrushless);
  
    }

    // Uses limelight output to move rotation motor directly
  public void turnOnSimpleAutoAim() {
    turretRotation.set(limelight.getLimelightX() / 270);
  }

  // Moves the rotation motor based on controller input
  public void aimWithController(XboxController controller) {

    double LT = controller.getRawAxis(Constants.leftTrigger);
    double RT = controller.getRawAxis(Constants.rightTrigger) * -1;
    turretRotation.set((LT + RT) / 25);
  }

  // Stop the rotation moto
  public void stopRotationMotor() {
    turretRotation.stopMotor();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
