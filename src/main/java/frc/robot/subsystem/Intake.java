// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Extensions.multiplexedColorSensor;

public class Intake extends SubsystemBase {

  WPI_TalonFX intakeMotor;
  WPI_TalonFX magazineMotor1;
  WPI_TalonFX magazineMotor2;

  // intake color sensor
  private multiplexedColorSensor sensor2;

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

  public int readSensor() {
    if (sensor2.getRed() > Constants.sensorRequiredValue && sensor2.getBlue() < Constants.sensorRequiredValue) {
      System.out.println("Red");
      return 1;
    } else if (sensor2.getBlue() > Constants.sensorRequiredValue && sensor2.getRed() < Constants.sensorRequiredValue) {
      System.out.println("Blue");
      return 2;
    } else {
      System.out.println("X");
      return 3;
    }
  }

  public void redBlueDecision() {
    runIntakeMotor();
    runMagazineMotors();
    if (readSensor() == 1) {
      intakeMotorStop();
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}