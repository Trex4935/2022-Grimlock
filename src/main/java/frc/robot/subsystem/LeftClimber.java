// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// MAG LIMIT SWITCHES
//   left side of robot - 6 + 7
//   right side of robot - 8 + 9
package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
// Imports
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.Helper;
import frc.robot.extensions.Falcon;
import frc.robot.extensions.FlippedDIO;

public class LeftClimber extends SubsystemBase {

  // Declare Motors
  WPI_TalonFX climbMotorLeft;

  // Declare Sensors

  // Climber Magnet Limits
  private static FlippedDIO leftClimberMagLimitBottom;
  private static FlippedDIO rightClimberMagLimitBottom;

  // Construct a climber object
  public LeftClimber() {

    // Setup the falcon climb motors
    //climbMotorRight = Falcon.createDefaultFalcon(Constants.climbMotorCanID);
    // climbMotorRight.setInverted(true);
    climbMotorLeft = Falcon.createDefaultFalcon(Constants.climbMotorAuxCanID);
    // climbMotorRight.setInverted(true);

    // Set the climb motors up for MotionMagic
    // public static final PID climbPidGainsRight = new PID(0.13372549, 0.01,
    // 1.3372549, 0.047017189);
    //climbMotorRight = Falcon.configMotinoMagic(climbMotorRight, 0.133, 0.01, 1.337, 0.047, 16319, 16319);
    climbMotorLeft = Falcon.configMotinoMagic(climbMotorLeft, 0.133, 0.01, 1.337, 0.047, 16319, 16319);

    // Climber Magnet Limit
    leftClimberMagLimitBottom = new FlippedDIO(6);
    //rightClimberMagLimitBottom = new FlippedDIO(8);
  }

  // Stop all of the climb motors
  public void stopAllClimbMotors() {
   // climbMotorRight.stopMotor();
    climbMotorLeft.stopMotor();
  }

  // Stop right of the climb motors
  public void stopRightClimbMotors() {
   // climbMotorRight.stopMotor();
  }

  // Stop left of the climb motors
  public void stopLeftClimbMotors() {
    climbMotorLeft.stopMotor();
  }

  // Climb to up value pos of motion magic
  public void climbDownMotionMagic() {
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
    if (getMotorRightBottomLimit()) {
    //  climbMotorRight.stopMotor();
      setEncoderToZeroR();
    } else {
    //  climbMotorRight.set(TalonFXControlMode.MotionMagic, Constants.downPositionRight);
    }
    if (getMotorLeftBottomLimit()) {
      climbMotorLeft.stopMotor();
      setEncoderToZeroL();
    } else {
      climbMotorLeft.set(TalonFXControlMode.MotionMagic, Constants.downPositionLeft);
    }
    // climbMotorLeft.follow(climbMotorRight, FollowerType.AuxOutput1);
  }

  // Move right arm to the down position using motion magic
  public void climbDownMotionMagicR() {
    // Stop when we hit the botom limit switch
    if (getMotorRightBottomLimit()) {
    //  climbMotorRight.stopMotor();
      setEncoderToZeroR();
    }
    // Engage motion magic with a target position
    else {
    //  climbMotorRight.set(TalonFXControlMode.MotionMagic, -5000);
    }
  }

  // Move left arm to the down position using motion magic
  public void climbDownMotionMagicL() {
    // Stop when we hit the botom limit switch
    if (getMotorLeftBottomLimit()) {
      climbMotorLeft.stopMotor();
      setEncoderToZeroL();
    }
    // Engage motion magic with a target position
    else {
      climbMotorLeft.set(TalonFXControlMode.MotionMagic, -5000);
    }
  }

  // Climb to down value pos of motion magic
  public void climbUpMotionMagicR() {
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
   // if (atSetPointRight(Constants.upPositionRight)) {
    //  climbMotorRight.stopMotor();
  //  } else {
    //  climbMotorRight.set(TalonFXControlMode.MotionMagic, Constants.upPositionRight);
   // }
    // climbMotorLeft.follow(climbMotorRight, FollowerType.AuxOutput1);
  }

  public void climbUpMotionMagicL() {
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
    if (atSetPointLeft(Constants.upPositionLeft)) {
      climbMotorLeft.stopMotor();
    } else {
      climbMotorLeft.set(TalonFXControlMode.MotionMagic, Constants.upPositionLeft);
    }
  }

  public void resetEncoderL() {
    climbMotorLeft.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
  }

  public void resetEncoderR() {
  //  climbMotorRight.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
  }

  // Set Encoders to zero.
  public void setEncoderToZeroR() {
  //  climbMotorRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));

  } // Set Encoders to zero.

  public void setEncoderToZeroL() {
    climbMotorLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    // System.out.println(climbMotorLeft.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb));
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

  // set Both Climber to Coast Mode
  public void changeBothClimberCoast() {
  //  climbMotorRight.setNeutralMode(NeutralMode.Coast);
    climbMotorLeft.setNeutralMode(NeutralMode.Coast);
  }

  // set Both Climber to BrakeMode
  public void changeBothClimberBrake() {
   // climbMotorRight.setNeutralMode(NeutralMode.Brake);
    climbMotorLeft.setNeutralMode(NeutralMode.Brake);
  }

  // The climber arms go up
  public void moveClimbArmsUP() {
  //  climbMotorRight.set(0.5);
    climbMotorLeft.set(0.5);

  }

  // Left climber goes up
public void moveClimbArmLeftUp() {
  climbMotorLeft.set(0.5);  
}

  // Right climber goes up
public void moveClimbArmRightUp() {
//  climbMotorRight.set(0.5);
}

  // The default climber motor goes down (test for correct direction then change
  // inverse if its the wrong way?)
  public void moveClimbArmsDown() {
    if (leftClimberMagLimitBottom.get()) {
      climbMotorLeft.stopMotor();
    } else {
      climbMotorLeft.set(-0.5);
    }
    if (rightClimberMagLimitBottom.get()) {
    //  climbMotorRight.stopMotor();
    } else {
    //  climbMotorRight.set(-0.5);
    }

  }

  // Stops the default climber motor
  public void stopClimbMotor() {
  //  climbMotorRight.stopMotor();
    climbMotorLeft.stopMotor();
  }

  // Moves left climber down manually w/ controller
  public void moveClimbArmLeftDown(){
    climbMotorLeft.set(-0.5);
  }

  // Moves right climber down manually w/ controller
  public void moveClimbArmRightDown(){
   // climbMotorRight.set(-0.5);
  }


  // Return boolean value if motion magic setpoint is reached
  //public boolean atSetPointRight(double setPoint) {
   // boolean atSt = Helper.RangeCompare(1000, -1000,
        //climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb) - setPoint);
    // System.out.println("Right");
    // System.out.println(climbMotorRight.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb)
    // - setPoint);
    // System.out.println(atSt);
   // return atSt;
  //}

  // Return boolean value if motion magic setpoint is reached
  public boolean atSetPointLeft(double setPoint) {
    boolean atSt = Helper.RangeCompare(1000, -1000,
        climbMotorLeft.getSelectedSensorPosition(Constants.kPIDLoopIdxClimb) - setPoint);
    // System.out.println("Left");
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
