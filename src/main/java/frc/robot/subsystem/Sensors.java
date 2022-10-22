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

  // Sensors.sensor2

  // Created color sensor variable.
  static ColorSensorV3 sensor2;

  /** Creates a new Sensors. */
  public Sensors() {
    // This method will be called once per scheduler run

    // Gets smackna values every run.
    leftTrapSmaknaDIO = new FlippedDIO(Constants.leftTrapSmakna);
    rightTrapSmaknaDIO = new FlippedDIO(Constants.rightTrapSmakna);
    insideMagSmaknaDIO = new FlippedDIO(Constants.insideMagSmakna);

    // Gets color value every run.
    sensor2 = new ColorSensorV3(I2C.Port.kMXP);
  }

  @Override
  public void periodic() {
    
  }
}
