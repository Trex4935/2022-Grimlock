// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// MAG LIMIT SWITCHES
//   left side of robot - 6 + 7
//   right side of robot - 8 + 9
package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
// Imports
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.Helper;
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
  private static FlippedDIO rotateArmLimit;

  // Construct a climber object
  public Climber() {

    // Populate the variables with motor objects with the correct IDs
    climbMotor = new WPI_TalonFX(Constants.climbMotorCanID);
    climbMotor.configFactoryDefault();
    climbMotor.setNeutralMode(NeutralMode.Brake);
    climbMotor.configOpenloopRamp(1);

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
        climbMotor = new WPI_TalonFX(Constants.climbMotorCanID);
        climbMotor.configFactoryDefault();
        climbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdxClimb, Constants.kTimeoutMs);
        climbMotor.configNeutralDeadband(0.001, Constants.kTimeoutMs);
        climbMotor.setSensorPhase(false);
        climbMotor.setInverted(true);
        climbMotor.setNeutralMode(NeutralMode.Brake);
        climbMotor.configOpenloopRamp(1);

    /* Set the peak and nominal outputs */
		climbMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		climbMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		climbMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
		climbMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		climbMotor.selectProfileSlot(Constants.kSlotIdxClimb, Constants.kPIDLoopIdx);
		climbMotor.config_kF(Constants.kSlotIdxClimb, Constants.kGains.kF, Constants.kTimeoutMs);
		climbMotor.config_kP(Constants.kSlotIdxClimb, Constants.kGains.kP, Constants.kTimeoutMs);
		climbMotor.config_kI(Constants.kSlotIdxClimb, Constants.kGains.kI, Constants.kTimeoutMs);
		climbMotor.config_kD(Constants.kSlotIdxClimb, Constants.kGains.kD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		climbMotor.configMotionCruiseVelocity(16319, Constants.kTimeoutMs);
		climbMotor.configMotionAcceleration(16319, Constants.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    climbMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    /* integral Zone */
		climbMotor.config_IntegralZone(Constants.kSlotIdxClimb, 200);
  }
  
  // Stop all of the climb motors
  public void stopAllClimbMotors() {
    climbMotor.stopMotor();
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

  public void climbUpMotionMagic() {
    if (atSetPoint(Constants.upPosition)) {
      climbMotor.stopMotor();
    } else {
      climbMotor.set(TalonFXControlMode.MotionMagic,Constants.upPosition);
    }
  }

  
  public void climbDownMotionMagic() {
    if (getMotorBottomLimit()) {
      climbMotor.stopMotor();
      climbMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    } else {
      climbMotor.set(TalonFXControlMode.MotionMagic,Constants.downPosition);
    }
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
    climbMotor.setInverted(false);

    if (getMotorTopLimit()) {
      climbMotor.stopMotor();
    } else {
      climbMotor.set(speed);
    }
  }

  // Sees whether the bottom limit switches are tripped or not (true / false)
  public boolean getMotorBottomLimit() {
    return leftClimberMagLimitBottom.get() || rightClimberMagLimitBottom.get();
  }

  // The default climber motor goes down (test for correct direction then change
  // inverse if its the wrong way?)
  // then prints what POV direction was pressed
  public void moveClimbArmsDown() {
    climbMotor.setInverted(true);
    SmartDashboard.putNumber("Climber", climbMotor.getTemperature());
    if (getMotorBottomLimit()) {
      climbMotor.stopMotor();
    } else {
      climbMotor.set(Constants.climbMotorSpeed);
    }
  }

  // Stops the default climber motor
  public void stopClimbMotor() {
    climbMotor.stopMotor();
  }

  public boolean atSetPoint( double setPoint){
    return Helper.RangeCompare(-25, 25, climbMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdxClimb) - setPoint); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
