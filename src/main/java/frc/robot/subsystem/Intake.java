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
import frc.robot.Extensions.BallColor;
import frc.robot.Extensions.multiplexedColorSensor;

public class Intake extends SubsystemBase {

  WPI_TalonFX intakeMotor;
  WPI_TalonFX magazineMotor1;
  WPI_TalonFX magazineMotor2;

  // intake color sensor
  private multiplexedColorSensor sensor2;

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
    magazineMotor1 = new WPI_TalonFX(Constants.magazineMotor1CanID);
    magazineMotor1.setInverted(false);
    magazineMotor2 = new WPI_TalonFX(Constants.magazineMotor2CanID);
    magazineMotor2.setInverted(false);

    sensor2 = new multiplexedColorSensor(I2C.Port.kOnboard, 2);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable color_table = inst.getTable("Intake");
    received_color = color_table.getEntry("color");

  }

  // eun intake motor
  public void runIntakeMotor() {

    intakeMotor.set(Constants.intakeMotorSpeed);

  }

  // run magazine motor
  public void runMagazineMotors() {

    magazineMotor1.set(Constants.magazineMotorSpeed);
    magazineMotor2.set(Constants.magazineMotorSpeed);

  }

  // stop intake motor
  public void intakeMotorStop() {
    intakeMotor.stopMotor();
  }

  // stop magazine motor
  public void magazineMotorStop() {
    magazineMotor1.stopMotor();
    magazineMotor2.stopMotor();
  }

  public BallColor readSensor() {
    if (sensor2.getRed() > Constants.sensorRequiredValue && sensor2.getBlue() < Constants.sensorRequiredValue) {
      System.out.println("Red");
      return BallColor.RED;
    } else if (sensor2.getBlue() > Constants.sensorRequiredValue && sensor2.getRed() < Constants.sensorRequiredValue) {
      System.out.println("Blue");
      return BallColor.BLUE;
    } else {
      System.out.println("X");
      return BallColor.NONE;
    }
  }

  public double dash_Color() {

    double x = received_color.getDouble(0.0);
    return x;
  }

  //determine what to do with ball based on color
  public double redBlueDecision(BallColor color) {
    
    //switch statement to decide what to do depending on ball color
    switch (color) {
      case NONE:
        return 0.6;
      case RED:
        return 0.8;
      case BLUE:
        return 0.4;
      default:
        return 0.6;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}