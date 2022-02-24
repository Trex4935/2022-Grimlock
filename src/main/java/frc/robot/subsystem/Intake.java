// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.c_shootBall;
import frc.robot.extensions.BallColor;
import frc.robot.extensions.FlippedDIO;
import frc.robot.extensions.Limelight;
import frc.robot.extensions.multiplexedColorSensor;

public class Intake extends SubsystemBase {

  WPI_TalonFX intakeMotor;
  WPI_TalonFX magazineMotor;
  WPI_TalonFX shooterMotor;

  // intake color sensor
  private multiplexedColorSensor sensor2;

  // magazine smacna
  private static FlippedDIO magazineSensor1DIO;
  private static FlippedDIO magazineSensor2DIO;
  private static FlippedDIO magazineSensor3DIO;

  // initializing the color sensor table
  public NetworkTable color_table;
  public NetworkTableEntry received_color;

  //// Add 2 smakna
  //// Add color sensor
  //// remove motor in the mag (there are only two total)

  // Constructor
  public Intake() {

    intakeMotor = new WPI_TalonFX(Constants.intakeMotorCanID);
    intakeMotor.setInverted(false);
    magazineMotor = new WPI_TalonFX(Constants.magazineMotor1CanID);
    magazineMotor.setInverted(true);

    magazineSensor1DIO = new FlippedDIO(Constants.magazineSensor1DIO);
    magazineSensor2DIO = new FlippedDIO(Constants.magazineSensor2DIO);
    magazineSensor3DIO = new FlippedDIO(Constants.magazineSensor3DIO);

    sensor2 = new multiplexedColorSensor(I2C.Port.kOnboard, 4);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable color_table = inst.getTable("Intake");
    received_color = color_table.getEntry("color");

    // shooter motor required for magazine logic
    shooterMotor = new WPI_TalonFX(Constants.shooterMotorCanID);

  }

  // eun intake motor
  public void runIntakeMotor() {

    intakeMotor.set(Constants.intakeMotorSpeed);
    magazineMotor.set(Constants.magazineMotorSpeed);

  }

  // Sets readyToShoot to true or false depending on if the limelight and shooter
  // speed is within a certain range
  // !! CURRENTLY NOT BEING USED OR RAN ANYWHERE !!
  public void detectShootingReady() {
    if ((Limelight.getLimelightX() <= Constants.limelightRange)
        && (Limelight.getLimelightX() >= -Constants.limelightRange)
        && (shooterMotor.getSelectedSensorVelocity() <= (redBlueDecision(readSensor()) + Constants.shooterSpeedRange))
        && (shooterMotor
            .getSelectedSensorVelocity() >= -(redBlueDecision(readSensor()) + Constants.shooterSpeedRange))) {

      Constants.readyToShoot = true;

    }

    else {

      Constants.readyToShoot = false;

    }

  }

  // run magazine motor
  public void runMagazineMotors() {

    magazineMotor.set(Constants.magazineMotorSpeed);

  }

  // stop intake motor
  public void intakeMotorStop() {
    intakeMotor.stopMotor();
  }

  // stop magazine motor
  public void magazineMotorStop() {
    magazineMotor.stopMotor();

  }

  public BallColor readSensor() {
    // System.out.println(sensor2.getRed() + ";" + sensor2.getBlue());
    if (sensor2.getRed() > Constants.sensorRequiredValue && sensor2.getBlue() < Constants.sensorRequiredValue) {
      // System.out.println("Red");
      return BallColor.RED;
    } else if (sensor2.getBlue() > Constants.sensorRequiredValue && sensor2.getRed() < Constants.sensorRequiredValue) {
      // System.out.println("Blue");
      return BallColor.BLUE;
    } else {
      // System.out.println("X");
      return BallColor.NONE;
    }
  }

  public double dash_Color() {

    double x = received_color.getDouble(0.0);
    return x;
  }

  // determine what to do with ball based on color
  public double redBlueDecision(BallColor color) {

    // switch statement to decide what to do depending on ball color
    // currently placeholder values
    switch (color) {
      case NONE:
        return 0.5;
      case RED:
        return 0.7;
      case BLUE:
        return 0.3;
      default:
        return 0.5;
    }

  }

  // Checks prox. color sens. for its value and if in range, returns true
  public boolean readProxColorSensor() {
    double prox_value = sensor2.getProximity();
    if (prox_value > Constants.proxSensorMin && prox_value < Constants.proxSensorMax) {
      return true;
    }
    return false;
  }

  // Get the value of the magazine sensor
  public boolean getMagazineSensor() {
    boolean a = magazineSensor1DIO.get();
    return (!a);
  }

  // When the magazine sensor sees a ball run the HB
  public boolean singulateBall() {
    if (getMagazineSensor() && readProxColorSensor() == true) {

      intakeMotorStop();
      magazineMotorStop();

      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}