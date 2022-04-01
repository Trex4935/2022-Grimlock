// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Shooter. */
  DoubleSolenoid intakeLifter;
  Compressor compressor;

  public Pneumatics() {
    intakeLifter = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
    compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeOff() {
    intakeLifter.set(Value.kOff);
  }

  public void intakeForward() {
    intakeLifter.set(Value.kForward);
  }

  public void intakeReverse() {
    intakeLifter.set(Value.kReverse);
  }

}