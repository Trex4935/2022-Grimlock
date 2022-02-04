// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class Turret extends SubsystemBase {

  WPI_TalonFX turretShooter;
  PWMSparkMax turretRotation = new PWMSparkMax(Constants.turretRotationPWMID);
  

  Limelight limelight;

  //magnet sensors
  private static DigitalInput leftMagLimit;
  private static DigitalInput middleMag;
  private static DigitalInput rightMagLimit;

  /** Creates a new turret. */
  public Turret() {

    // Init motors
    turretShooter = new WPI_TalonFX(Constants.turretShooterCanID);
    // turretRotation = new WPI_TalonFX(Constants.turretRotationCanID);

    limelight = new Limelight();

  }

  // Uses limelight output to move rotation motor directly
  // Uses magnets to detect if it is and prevent it from rotating too far left or right
  public void turnOnSimpleAutoAim() {
    if (leftMagLimit.get() == true && (limelight.getLimelightX() / 270) <= 0) {
      turretRotation.set(0);
    }
    else if (rightMagLimit.get() == true && (limelight.getLimelightX() / 270) >= 0) {
      turretRotation.set(0);
    }
    else {
      turretRotation.set(limelight.getLimelightX() / 270);
    }
  }

  //runs the turret shooter with a given speed
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
  // Stop shooter motor
  public void stopShooterMotor() {
    turretShooter.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}




//hello