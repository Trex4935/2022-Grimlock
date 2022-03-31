// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// MAG LIMIT SWITCHES
//   left side of robot - 6 + 7
//   right side of robot - 8 + 9
package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// Imports
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.Helper;
import frc.robot.extensions.SmartDebug;
import frc.robot.extensions.FlippedDIO;

public class Climber extends SubsystemBase {

  // Declare Motors
  WPI_TalonFX climbMotorRight;
  WPI_TalonFX climbMotorLeft;
  WPI_TalonSRX rotationMotor;
  WPI_TalonSRX pinMotor;
  WPI_TalonFX elevatorWinchMotor;

  // Declare Sensors

  // Climber Magnet Limits
  private static FlippedDIO leftClimberMagLimitTop;
  private static FlippedDIO leftClimberMagLimitBottom;
  private static FlippedDIO rightClimberMagLimitTop;
  private static FlippedDIO rightClimberMagLimitBottom;
  private static FlippedDIO rotateArmLimit;

  // Construct a climber object
  public Climber() {

    // Populate the variables with motor objects with the correct IDs
    // climbMotor = new WPI_TalonFX(Constants.climbMotorCanID);
    // climbMotor.configFactoryDefault();
    // climbMotor.setNeutralMode(NeutralMode.Brake);
    // climbMotor.configOpenloopRamp(1);
    setMotionMagic();

    rotationMotor = new WPI_TalonSRX(Constants.rotationMotorCanID);
    rotationMotor.configFactoryDefault();
    rotationMotor.setNeutralMode(NeutralMode.Coast);

    // Climber Magnet Limits
    leftClimberMagLimitTop = new FlippedDIO(Constants.leftClimberMagLimitTopID);
    leftClimberMagLimitBottom = new FlippedDIO(Constants.leftClimberMagLimitBottomID);
    rightClimberMagLimitTop = new FlippedDIO(Constants.rightClimberMagLimitTopID);
    rightClimberMagLimitBottom = new FlippedDIO(Constants.rightClimberMagLimitBottomID);
    rotateArmLimit = new FlippedDIO(Constants.extraClimberMagLimitBottomID);
  }

  public void setMotionMagic() {

    // Populate the variables with motor objects with the correct IDs
    climbMotorRight = new WPI_TalonFX(Constants.climbMotorCanID);
    climbMotorRight.configFactoryDefault();
    climbMotorRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdxClimb,
        Constants.kTimeoutMs);
    climbMotorRight.configNeutralDeadband(0.001, Constants.kTimeoutMs);
    climbMotorRight.setSensorPhase(false);
    climbMotorRight.setInverted(true);
    climbMotorRight.setNeutralMode(NeutralMode.Brake);
    climbMotorRight.configOpenloopRamp(1);

    /* Set the peak and nominal outputs */
    climbMotorRight.configNominalOutputForward(0, Constants.kTimeoutMs);
    climbMotorRight.configNominalOutputReverse(0, Constants.kTimeoutMs);
    climbMotorRight.configPeakOutputForward(1, Constants.kTimeoutMs);
    climbMotorRight.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    climbMotorRight.selectProfileSlot(Constants.kSlotIdxClimb, Constants.kPIDLoopIdx);
    climbMotorRight.config_kF(Constants.kSlotIdxClimb, Constants.climbPidGains.getkF(), Constants.kTimeoutMs);
    climbMotorRight.config_kP(Constants.kSlotIdxClimb, Constants.climbPidGains.getkP(), Constants.kTimeoutMs);
    climbMotorRight.config_kI(Constants.kSlotIdxClimb, Constants.climbPidGains.getkI(), Constants.kTimeoutMs);
    climbMotorRight.config_kD(Constants.kSlotIdxClimb, Constants.climbPidGains.getkD(), Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    climbMotorRight.configMotionCruiseVelocity(Constants.velocityMotionMagic, Constants.kTimeoutMs);
    climbMotorRight.configMotionAcceleration(Constants.accelMotionMagic, Constants.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    climbMotorRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    /* integral Zone */
    climbMotorRight.config_IntegralZone(Constants.kSlotIdxClimb, 200);

    // Auxilary motor
    climbMotorLeft = new WPI_TalonFX(Constants.climbMotorAuxCanID);
    climbMotorLeft.configFactoryDefault();
    climbMotorRight.setInverted(false);
  }

  // Stop all of the climb motors
  public void stopAllClimbMotors() {
    climbMotorRight.stopMotor();
    rotationMotor.stopMotor();
  }

  // The rotating climber motor movest the arms towards shooter
  public void rotateArmsTowardsShooter() {
    rotationMotor.setInverted(false);

    // if the tang limit switch is impacted stop the rotation motor so we don't over
    // rotate.
    if (rotateArmLimit.get()) {
      rotationMotor.stopMotor();
    } else {
      rotationMotor.set(Constants.climbRotateSpeed);
    }

  }

  // Climb to up value pos of motion magic
  public void climbUpMotionMagic() {
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
    climbMotorRight.set(TalonFXControlMode.MotionMagic, Constants.upPosition);
    climbMotorLeft.follow(climbMotorRight, FollowerType.AuxOutput1);
  }

  // Climb to down value pos of motion magic
  public void climbDownMotionMagic() {
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
    climbMotorRight.set(TalonFXControlMode.MotionMagic, Constants.downPosition);
    climbMotorLeft.follow(climbMotorRight, FollowerType.AuxOutput1);
  }

  // Set Encoders to zero.
  public void setEncoderToZeroR() {
    climbMotorRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));

  } // Set Encoders to zero.

  public void setEncoderToZeroL() {
    climbMotorLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    System.out.println(climbMotorLeft.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
  }

  // The rotating climber motor moves the arms towards intake
  public void rotateArmsTowardsIntake() {
    rotationMotor.setInverted(false);
    rotationMotor.set(Constants.climbRotateSpeed);

  }

  // Stops the rotating climber motor
  public void stopArmRotation() {
    rotationMotor.stopMotor();
  }

  public boolean getMotorTopLimit() {
    return leftClimberMagLimitTop.get() || rightClimberMagLimitTop.get();
  }

  // The climber arms go up
  public void moveClimbArmsUP(double speed) {
    if (getMotorRightBottomLimit()) {
      setEncoderToZeroR();
    }
    if (getMotorLeftBottomLimit()) {
      setEncoderToZeroL();
    }
    if (getMotorTopLimit()) {
      climbMotorRight.stopMotor();
    } else {
      climbMotorRight.set(speed);
      climbMotorLeft.set(speed);
    }
  }

  // Sees whether the bottom limit switches are tripped or not (true / false)
  public boolean getMotorBottomLimit() {
    return leftClimberMagLimitBottom.get() || rightClimberMagLimitBottom.get();
  }

  // return Left limit switch
  public boolean getMotorLeftBottomLimit() {
    return leftClimberMagLimitBottom.get();
  }

  // Return Right Limit Switch
  public boolean getMotorRightBottomLimit() {
    return rightClimberMagLimitBottom.get();
  }

  // set Right Climber to Coast Mode
  public void changeRightClimberCoast() {
    climbMotorRight.setNeutralMode(NeutralMode.Coast);
  }

  // set Left Climber to Coast Mode
  public void changeLeftClimberCoast() {
    climbMotorLeft.setNeutralMode(NeutralMode.Coast);
  }

  // set Both Climber to Coast Mode
  public void changeBothClimberCoast() {
    climbMotorRight.setNeutralMode(NeutralMode.Coast);
    climbMotorLeft.setNeutralMode(NeutralMode.Coast);
  }

  // set Right Climber to BrakeMode
  public void changeRightClimberBrake() {
    climbMotorRight.setNeutralMode(NeutralMode.Brake);
  }

  // set Left Climber to BrakeMode
  public void changeLeftClimberBrake() {
    climbMotorLeft.setNeutralMode(NeutralMode.Brake);
  }

  // set Both Climber to BrakeMode
  public void changeBothClimberBrake() {
    climbMotorRight.setNeutralMode(NeutralMode.Brake);
    climbMotorLeft.setNeutralMode(NeutralMode.Brake);
  }

  // The default climber motor goes down (test for correct direction then change
  // inverse if its the wrong way?)
  // then prints what POV direction was pressed
  public void moveClimbArmsDown() {
    if (getMotorRightBottomLimit()) {
      climbMotorRight.stopMotor();
      setEncoderToZeroR();
    }
    if (getMotorLeftBottomLimit()) {
      climbMotorLeft.stopMotor();
      setEncoderToZeroL();
    } else {
      climbMotorRight.set(-Constants.climbMotorSpeed);
      climbMotorLeft.set(-Constants.climbMotorSpeed);
    }
  }

  // Stops the default climber motor
  public void stopClimbMotor() {
    climbMotorRight.stopMotor();
    climbMotorLeft.stopMotor();
  }

  // Return boolean value if motion magic setpoint is reached
  public boolean atSetPoint(double setPoint) {
    boolean atSt = Helper.RangeCompare(10, -10,
        climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb) - setPoint);
    System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb) - setPoint);
    System.out.println(atSt);
    return atSt;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
