// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Shooter. */
  Solenoid intakePneumatic;
  Compressor compressor;

  public Pneumatics() {
    intakePneumatic = new Solenoid(0, PneumaticsModuleType.CTREPCM, 0);
    compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeON() {
    // turns it on
    intakePneumatic.set(true);
  }

  public void intakeOFF() {
    intakePneumatic.set(false);
  }
}