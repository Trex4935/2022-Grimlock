// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.BallColor;
import frc.robot.extensions.FlippedDIO;
//import frc.robot.extensions.multiplexedColorSensor;
import frc.robot.extensions.SmartDebug;

public class Intake extends SubsystemBase {

  WPI_TalonFX intakeMotor;
  WPI_TalonFX magazineMotor;
  WPI_TalonFX intakeRetractionMotor;

  // intake color sensor
  private ColorSensorV3 sensor2;

  // magazine smacna
  private static FlippedDIO leftTrapSmaknaDIO;
  private static FlippedDIO rightTrapSmaknaDIO;
  private static FlippedDIO insideMagSmaknaDIO;

  // initializing the color sensor table
  public NetworkTable color_table;
  public NetworkTableEntry received_color;

  //// Add 2 smakna
  //// Add color sensor
  //// remove motor in the mag (there are only two total)

  // Constructor
  public Intake() {

    // run the intake rollers at the front
    intakeMotor = new WPI_TalonFX(Constants.intakeMotorCanID);
    intakeMotor.setInverted(true);

    // motor to run the magazine belts
    magazineMotor = new WPI_TalonFX(Constants.magazineMotorCanID);
    magazineMotor.setInverted(true);

    // intake retraction motor
    intakeRetractionMotor = new WPI_TalonFX(Constants.intakeRetractionMotorID);
    intakeRetractionMotor.setInverted(false);
    intakeRetractionMotor.setNeutralMode(NeutralMode.Brake);

    // sensors for the trap portion of the intake
    leftTrapSmaknaDIO = new FlippedDIO(Constants.leftTrapSmakna);
    rightTrapSmaknaDIO = new FlippedDIO(Constants.rightTrapSmakna);
    insideMagSmaknaDIO = new FlippedDIO(Constants.insideMagSmakna);

    // color sensor at the top of the magazine
    sensor2 = new ColorSensorV3(I2C.Port.kMXP);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable color_table = inst.getTable("Intake");
    received_color = color_table.getEntry("color");

  }

  // eun intake motor
  public void runIntakeMotor() {
    SmartDebug.putBoolean("Intake On", true);
    intakeMotor.set(Constants.intakeMotorSpeed);
  }

  // run magazine motor
  public void runMagazineMotors(boolean bypassProx) {

    // bypass proximity sensor and run magazine regardless of there being a ball or
    // not if shooter is at correct speed and on target
    if (bypassProx) {
      magazineMotor.set(Constants.magazineMotorSpeed);
      SmartDashboard.putBoolean("Mag Active", true);

    } else {
      // System.out.println("No ByPass");
      // if proximity sensor is not bypassed then stop magazine if a ball is detected
      if (readProxColorSensor()) {
        // System.out.println("See Ball");
        magazineMotor.stopMotor();
        SmartDashboard.putBoolean("Mag Active", false);
      } else {
        magazineMotor.set(Constants.magazineMotorSpeed);
        SmartDashboard.putBoolean("Mag Active", true);
      }
    }
  }

  // stop intake motor
  public void intakeMotorStop() {
    intakeMotor.stopMotor();
  }

  public void runIntakeRetractionMotor() {
    if (Constants.retractionState) {

      intakeRetractionMotor.set(Constants.retractionSpeed);
      // new WaitCommand(Constants.retractionRunTime);

    } else {

      intakeRetractionMotor.set(-Constants.retractionSpeed);
      // new WaitCommand(Constants.retractionRunTime);
    }
  }

  public void flipIntakeRetrationMotorState() {
    Constants.retractionState = !Constants.retractionState;
  }

  // stop magazine motor
  public void magazineMotorStop() {
    magazineMotor.stopMotor();

  }

  public BallColor readSensor() {
    // System.out.println(sensor2.getRed() + ";" + sensor2.getBlue() + ";" +
    // sensor2.getGreen());

    // If we detect a ball with the prox sensor determine the color
    if (readProxColorSensor()) {

      // subtrace the blue channel from the red channel so we know which one we have
      // more of
      // if positive == red
      // if negative == blue
      double colorCompare = sensor2.getRed() - sensor2.getBlue();

      // determine color based on +/- of value
      if (colorCompare <= 0) {
        return BallColor.BLUE;
      } else {
        return BallColor.RED;
      }

    }
    // since we don't see a ball == NONE
    else {
      return BallColor.NONE;
    }
  }

  // ????
  public double dash_Color() {

    double x = received_color.getDouble(0.0);
    return x;
  }

  // Checks prox. color sens. for its value and if in range, returns true
  public boolean readProxColorSensor() {
    double prox_value = sensor2.getProximity();
    // System.out.println(sensor2.getProximity());
    // System.out.println(readSensor());
    if (prox_value > Constants.proxSensorMin) {
      // System.out.println("BALL");
      return true;
    }
    SmartDashboard.putNumber("Ball in top of MAG", prox_value);
    // System.out.println("NO BALL");
    return false;
  }

  // Get the value of smakna 1
  public boolean getMagazineSensor1DIO() {
    return leftTrapSmaknaDIO.get();
  }

  // Get the value of smakna 2
  public boolean getMagazineSensor2DIO() {
    return rightTrapSmaknaDIO.get();
  }

  // Get the value of smakna 3
  public boolean getMagazineSensor3DIO() {
    return insideMagSmaknaDIO.get();
  }

  // When the magazine sensor sees a ball run the HB
  public void singulateBall() {
    // if any of the trap sensors see a ball & we have a ball at the top stop intake
    if ((getMagazineSensor1DIO() || getMagazineSensor2DIO() || getMagazineSensor3DIO())
        && readProxColorSensor()) {
      intakeMotorStop();
    }
    // else keep the intake running
    else {
      runIntakeMotor();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // stop the intake retraction motor
  public void stopIntakeRetrationMotor() {
    intakeRetractionMotor.stopMotor();
  }

}