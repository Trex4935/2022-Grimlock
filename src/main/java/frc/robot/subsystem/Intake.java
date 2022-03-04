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
import frc.robot.extensions.BallColor;
import frc.robot.extensions.FlippedDIO;
import frc.robot.extensions.Limelight;
import frc.robot.extensions.multiplexedColorSensor;

public class Intake extends SubsystemBase {

  WPI_TalonFX intakeMotor;
  WPI_TalonFX magazineMotor;

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

  }

  // eun intake motor
  public void runIntakeMotor() {
    intakeMotor.set(Constants.intakeMotorSpeed);
  }

  // run magazine motor
  public void runMagazineMotors(boolean bypassProx) {

    // bypass proximity sensor and run magazine regardless of there being a ball or
    // not if shooter is at correct speed and on target
    if (bypassProx) {
      magazineMotor.set(Constants.magazineMotorSpeed);
    } else {
      // if proximity sensor is not bypassed then stop magazine if a ball is detected
      if (readProxColorSensor()) {
        magazineMotor.stopMotor();
      } else {
        magazineMotor.set(Constants.magazineMotorSpeed);
      }
    }
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
    // System.out.println(sensor2.getRed() + ";" + sensor2.getBlue());
    if (sensor2.getRed() > Constants.sensorRequiredValue) {
      // System.out.println("Red");
      return BallColor.RED;
    } else if (sensor2.getBlue() > Constants.sensorRequiredValue) {
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
        // System.out.println("NONE");
        return 1000;
      case RED:
        // System.out.println("RED");
        return 2000;
      case BLUE:
        // System.out.println("BLUE");
        return 3000;
      default:
        // System.out.println("defaultdefault");
        return 1000;
    }

  }

  // Checks prox. color sens. for its value and if in range, returns true
  public boolean readProxColorSensor() {
    double prox_value = sensor2.getProximity();
    // System.out.println(sensor2.getProximity());
    // System.out.println(readSensor());
    if (prox_value > Constants.proxSensorMin) {

      // System.out.println("bababa");

      return true;
    }
    // System.out.println("h");

    return false;
  }

  // Get the value of smakna 1
  public boolean getMagazineSensor1DIO() {
    return magazineSensor1DIO.get();
  }

  // Get the value of smakna 2
  public boolean getMagazineSensor2DIO() {
    return magazineSensor2DIO.get();
  }

  // Get the value of smakna 3
  public boolean getMagazineSensor3DIO() {
    return magazineSensor3DIO.get();
  }

  // When the magazine sensor sees a ball run the HB
  public void singulateBall() {
    if ((getMagazineSensor1DIO() || getMagazineSensor2DIO() || getMagazineSensor3DIO())
        && readProxColorSensor() == true) {
      intakeMotorStop();
    }
    runIntakeMotor();
    readProxColorSensor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}