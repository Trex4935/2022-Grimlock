// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// MAG LIMIT SWITCHES
//   left side of robot - 6 + 7
//   right side of robot - 8 + 9
package frc.robot.subsystem;

// Imports
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.FlippedDIO;

public class Climber extends SubsystemBase {

  // Declare Motors
  WPI_TalonFX climbMotor;
  WPI_TalonSRX rotationMotor;
  WPI_TalonSRX pinMotor;
  WPI_TalonFX elevatorWinchMotor;

  // Declare Sensors

  // Climber Magnet Limits
  private static FlippedDIO leftClimberMagLimitTop;
  private static FlippedDIO leftClimberMagLimitBottom;
  private static FlippedDIO rightClimberMagLimitTop;
  private static FlippedDIO rightClimberMagLimitBottom;

  // Construct a climber object
  public Climber() {

    // Populate the variables with motor objects with the correct IDs
    climbMotor = new WPI_TalonFX(Constants.climbMotorCanID);
    rotationMotor = new WPI_TalonSRX(Constants.rotationMotorCanID);

    // Climber Magnet Limits
    leftClimberMagLimitTop = new FlippedDIO(Constants.leftClimberMagLimitTopID);
    leftClimberMagLimitBottom = new FlippedDIO(Constants.leftClimberMagLimitBottomID);
    rightClimberMagLimitTop = new FlippedDIO(Constants.rightClimberMagLimitTopID);
    rightClimberMagLimitBottom = new FlippedDIO(Constants.rightClimberMagLimitBottomID);

    // Braking Mode
    climbMotor.setNeutralMode(Constants.elevatorBrakeMode);
  }

  // Stop all of the climb motors
  public void stopAllClimbMotors() {
    climbMotor.stopMotor();
    rotationMotor.stopMotor();
  }

  // The rotating climber motor movest the arms towards shooter
  public void rotateClimbTowardsShooter() {
    rotationMotor.setInverted(false);
    rotationMotor.set(Constants.climbRotateSpeed);
  }

  // The rotating climber motor moves the arms towards intake
  public void rotateClimbTowardsIntake() {
    rotationMotor.setInverted(true);
    rotationMotor.set(Constants.climbRotateSpeed);
  }

  // Stops the rotating climber motor
  public void stopClimbRotate() {
    rotationMotor.stopMotor();
  }

  // The default climber arms go up
  public void motorClimbUp() {
    climbMotor.setInverted(false);

    if (leftClimberMagLimitTop.get() || rightClimberMagLimitTop.get()) {
      climbMotor.stopMotor();
      // System.out.println(Constants.leftClimberMagLimitTopID);
      // System.out.println(Constants.rightClimberMagLimitTopID);
    } else {
      climbMotor.set(Constants.climbMotorSpeed);
    }
  }

  // The default climber motor goes down (test for correct direction then change
  // inverse if its the wrong way?)
  // then prints what POV direction was pressed
  public void motorClimbDown() {
    climbMotor.setInverted(true);
    if (leftClimberMagLimitBottom.get() || rightClimberMagLimitBottom.get()) {
      climbMotor.stopMotor();
      // System.out.println(Constants.leftClimberMagLimitBottomID);
      // System.out.println(Constants.rightClimberMagLimitBottomID);
    } else {
      climbMotor.set(Constants.climbMotorSpeed);
    }
    // System.out.println("down");
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
