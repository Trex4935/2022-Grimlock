// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.ColorSensorV3;

import frc.robot.Constants;
import frc.robot.extensions.FlippedDIO;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensors extends SubsystemBase {

  // Created smakna variables.
  static FlippedDIO leftTrapSmaknaDIO;
  static FlippedDIO rightTrapSmaknaDIO;
  static FlippedDIO insideMagSmaknaDIO;

  public static boolean valueLeftSmakna;
  public static boolean valueRightSmakna;
  public static boolean valueInsideSmakna;

  // Created color sensor variables
  static ColorSensorV3 colorSensor;

  public static int blueColorSensor;
  public static int redColorSensor;
  public static int proxColorSensor;

  // Create magnet sensor variables
  static FlippedDIO leftMagLimit;
  static FlippedDIO middleMag;
  static FlippedDIO rightMagLimit;

  public static boolean valueLeftMagLimit;
  public static boolean valueMiddleMag;
  public static boolean valueRightMagLimit;

  /** Creates a new Sensors. */
  public Sensors() {
    // This method will be called once per scheduler run

    // Creates smackna sensor object at start.
    leftTrapSmaknaDIO = new FlippedDIO(Constants.leftTrapSmakna);
    rightTrapSmaknaDIO = new FlippedDIO(Constants.rightTrapSmakna);
    insideMagSmaknaDIO = new FlippedDIO(Constants.insideMagSmakna);

    // Creates color sensor object at start.
    colorSensor = new ColorSensorV3(I2C.Port.kMXP);

    // Creates color sensor object at start.
    leftMagLimit = new FlippedDIO(Constants.leftMagLimitID);
    middleMag = new FlippedDIO(Constants.middleMagID);
    rightMagLimit = new FlippedDIO(Constants.rightMagLimitID);
  }

  @Override
  public void periodic() {
    // Constantly gets smakna sensor values. (Boolean)
    valueLeftSmakna = leftTrapSmaknaDIO.get();
    valueRightSmakna = rightTrapSmaknaDIO.get();
    valueInsideSmakna = insideMagSmaknaDIO.get();

    // Constantly gets color sensor values. (Int)
    blueColorSensor = colorSensor.getBlue();
    redColorSensor = colorSensor.getRed();
    proxColorSensor = colorSensor.getProximity();

    // Constantly gets magnet sensor values. (Boolean)
    valueLeftMagLimit = leftMagLimit.get();
    valueMiddleMag = middleMag.get();
    valueRightMagLimit = rightMagLimit.get();
  }
}
