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
    setMotionMagicRight();
    setMotionMagicLeft();

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

  public void setMotionMagicRight() {

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
    climbMotorRight.config_kF(Constants.kSlotIdxClimb, Constants.climbPidGainsRight.getkF(), Constants.kTimeoutMs);
    climbMotorRight.config_kP(Constants.kSlotIdxClimb, Constants.climbPidGainsRight.getkP(), Constants.kTimeoutMs);
    climbMotorRight.config_kI(Constants.kSlotIdxClimb, Constants.climbPidGainsRight.getkI(), Constants.kTimeoutMs);
    climbMotorRight.config_kD(Constants.kSlotIdxClimb, Constants.climbPidGainsRight.getkD(), Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    climbMotorRight.configMotionCruiseVelocity(Constants.velocityMotionMagicRight, Constants.kTimeoutMs);
    climbMotorRight.configMotionAcceleration(Constants.accelMotionMagicRight, Constants.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    climbMotorRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    /* integral Zone */
    climbMotorRight.config_IntegralZone(Constants.kSlotIdxClimb, 200);

  }

  public void setMotionMagicLeft() {

    // Auxilary motor
    climbMotorLeft = new WPI_TalonFX(Constants.climbMotorAuxCanID);
    climbMotorLeft.configFactoryDefault();
    climbMotorLeft.setInverted(false);
    climbMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdxClimb,
        Constants.kTimeoutMs);
    climbMotorLeft.configNeutralDeadband(0.001, Constants.kTimeoutMs);
    climbMotorLeft.setSensorPhase(false);
    climbMotorLeft.setInverted(true);
    climbMotorLeft.setNeutralMode(NeutralMode.Brake);
    climbMotorLeft.configOpenloopRamp(1);

    /* Set the peak and nominal outputs */
    climbMotorLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
    climbMotorLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
    climbMotorLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
    climbMotorLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    climbMotorLeft.selectProfileSlot(Constants.kSlotIdxClimb, Constants.kPIDLoopIdx);
    climbMotorLeft.config_kF(Constants.kSlotIdxClimb, Constants.climbPidGainsLeft.getkF(), Constants.kTimeoutMs);
    climbMotorLeft.config_kP(Constants.kSlotIdxClimb, Constants.climbPidGainsLeft.getkP(), Constants.kTimeoutMs);
    climbMotorLeft.config_kI(Constants.kSlotIdxClimb, Constants.climbPidGainsLeft.getkI(), Constants.kTimeoutMs);
    climbMotorLeft.config_kD(Constants.kSlotIdxClimb, Constants.climbPidGainsLeft.getkD(), Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    climbMotorLeft.configMotionCruiseVelocity(Constants.velocityMotionMagicLeft, Constants.kTimeoutMs);
    climbMotorLeft.configMotionAcceleration(Constants.accelMotionMagicLeft, Constants.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    climbMotorLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    /* integral Zone */
    climbMotorLeft.config_IntegralZone(Constants.kSlotIdxClimb, 200);
  }

  // Stop all of the climb motors
  public void stopAllClimbMotors() {
    climbMotorRight.stopMotor();
    climbMotorLeft.stopMotor();
  }

  // Stop right of the climb motors
  public void stopRightClimbMotors() {
    climbMotorRight.stopMotor();
  }

  // Stop left of the climb motors
  public void stopLeftClimbMotors() {
    climbMotorLeft.stopMotor();
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
  public void climbDownMotionMagic() {
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
    if (getMotorRightBottomLimit()) {
      climbMotorRight.stopMotor();
      setEncoderToZeroR();
    } else {
      climbMotorRight.set(TalonFXControlMode.MotionMagic, Constants.downPositionRight);
    }
    if (getMotorLeftBottomLimit()) {
      climbMotorLeft.stopMotor();
      setEncoderToZeroL();
    } else {
      climbMotorLeft.set(TalonFXControlMode.MotionMagic, Constants.downPositionLeft);
    }
    // climbMotorLeft.follow(climbMotorRight, FollowerType.AuxOutput1);
  }

  // Climb to up value pos of motion magic
  public void climbDownMotionMagicR() {
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
    if (getMotorRightBottomLimit()) {
      climbMotorRight.stopMotor();
      setEncoderToZeroR();
    } else {
      climbMotorRight.set(TalonFXControlMode.MotionMagic, Constants.downPositionRight);
    }
  }

  // Climb to up value pos of motion magic
  public void climbDownMotionMagicL() {
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
    if (getMotorLeftBottomLimit()) {
      climbMotorLeft.stopMotor();
      setEncoderToZeroL();
    } else {
      climbMotorLeft.set(TalonFXControlMode.MotionMagic, Constants.downPositionLeft);
    }
    // climbMotorLeft.follow(climbMotorRight, FollowerType.AuxOutput1);
  }

  // Climb to down value pos of motion magic
  public void climbUpMotionMagic() {
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
    if (atSetPointRight(Constants.upPositionRight)) {
      climbMotorRight.stopMotor();
    } else {
      climbMotorRight.set(TalonFXControlMode.MotionMagic, Constants.upPositionRight);
    }
    if (atSetPointLeft(Constants.upPositionLeft)) {
      climbMotorLeft.stopMotor();
    } else {
      climbMotorLeft.set(TalonFXControlMode.MotionMagic, Constants.upPositionLeft);
    }
    // climbMotorLeft.follow(climbMotorRight, FollowerType.AuxOutput1);
  }

  // Climb to down value pos of motion magic
  public void climbUpMotionMagicR() {
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
    if (atSetPointRight(Constants.upPositionRight)) {
      climbMotorRight.stopMotor();
    } else {
      climbMotorRight.set(TalonFXControlMode.MotionMagic, Constants.upPositionRight);
    }
    // climbMotorLeft.follow(climbMotorRight, FollowerType.AuxOutput1);
  }

  public void climbUpMotionMagicL() {
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
    if (atSetPointLeft(Constants.upPositionLeft)) {
      climbMotorLeft.stopMotor();
    } else {
      climbMotorLeft.set(TalonFXControlMode.MotionMagic, Constants.upPositionLeft);
    }
    // climbMotorLeft.follow(climbMotorRight, FollowerType.AuxOutput1);
  }

  // Set Encoders to zero.
  public void setEncoderToZeroR() {
    climbMotorRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));

  } // Set Encoders to zero.

  public void setEncoderToZeroL() {
    climbMotorLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    // System.out.println(climbMotorLeft.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
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
  public void moveClimbArmsUP() {
    climbMotorRight.set(0.5);
    climbMotorLeft.set(0.5);

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
    if (leftClimberMagLimitBottom.get()) {
      climbMotorLeft.stopMotor();
    } else {
      climbMotorLeft.set(-0.5);
    }
    if (rightClimberMagLimitBottom.get()) {
      climbMotorRight.stopMotor();
    } else {
      climbMotorRight.set(-
      0.5);
    }

  }

  // Stops the default climber motor
  public void stopClimbMotor() {
    climbMotorRight.stopMotor();
    climbMotorLeft.stopMotor();
  }

  // Return boolean value if motion magic setpoint is reached
  public boolean atSetPointRight(double setPoint) {
    boolean atSt = Helper.RangeCompare(100, -100,
        climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb) - setPoint);
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb)
    // - setPoint);
    // System.out.println(atSt);
    return atSt;
  }

  // Return boolean value if motion magic setpoint is reached
  public boolean atSetPointLeft(double setPoint) {
    boolean atSt = Helper.RangeCompare(100, -100,
        climbMotorLeft.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb) - setPoint);
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb)
    // - setPoint);
    // System.out.println(atSt);
    return atSt;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
