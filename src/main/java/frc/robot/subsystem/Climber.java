// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

// Imports
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  // Declare Motors
  WPI_TalonFX climbMotor;
  WPI_TalonSRX rotationMotor;
  WPI_TalonSRX pinMotor;

  // Declare Sensors

  // Construct a climber object
  public Climber() {

    // Populate the variables with motor objects with the correct IDs
    climbMotor = new WPI_TalonFX(Constants.climbMotorCanID);
    rotationMotor = new WPI_TalonSRX(Constants.rotationMotorCanID);
    pinMotor = new WPI_TalonSRX(Constants.pinMotorCanID);

  }

  // Stop all of the climb motors
  public void stopAllClimbMotors() {
    climbMotor.stopMotor();
    rotationMotor.stopMotor();
    pinMotor.stopMotor();
  }

  // Moves the pin motor for a second and then stops it. (might be wonky)
  public void pinRotate() throws InterruptedException {
    pinMotor.set(Constants.pinMotorSpeed);
    wait(1000);
    pinMotor.stopMotor();
  }

  // The rotating climber motor goes left (test for correct direction then change
  // inverse if its the wrong way?)
  // then prints what POV direction was pressed
  public void rotateClimbLeft() {
    rotationMotor.setInverted(false);
    rotationMotor.set(Constants.climbRotateSpeed);
    // System.out.println("left");
  }

  // The rotating climber motor goes right (test for correct direction then change
  // inverse if its the wrong way?)
  // then prints what POV direction was pressed
  public void rotateClimbRight() {
    rotationMotor.setInverted(true);
    rotationMotor.set(Constants.climbRotateSpeed);
    // System.out.println("right");
  }

  // Stops the rotating climber motor
  public void stopClimbRotate() {
    rotationMotor.stopMotor();
  }

  // The default climber motor goes up (test for correct direction then change
  // inverse if its the wrong way?)
  // then prints what POV direction was pressed
  public void motorClimbUp() {
    climbMotor.setInverted(false);
    climbMotor.set(Constants.climbMotorSpeed);
    //System.out.println("up");
  }

  // The default climber motor goes down (test for correct direction then change
  // inverse if its the wrong way?)
  // then prints what POV direction was pressed
  public void motorClimbDown() {
    climbMotor.setInverted(true);
    climbMotor.set(Constants.climbMotorSpeed);
    //System.out.println("down");
  }

  // Stops the default climber motor
  public void stopClimbMotor() {
    climbMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
