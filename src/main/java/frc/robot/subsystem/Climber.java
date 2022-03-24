// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// MAG LIMIT SWITCHES
//   left side of robot - 6 + 7
//   right side of robot - 8 + 9
package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
// Imports
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public Climber(int ID) {    
    
    // Populate the variables with motor objects with the correct IDs
    climbMotor = new WPI_TalonFX(Constants.climbMotorCanID);
    climbMotor.configFactoryDefault();

    // Configure Sensor Source for Primary PID 
    climbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
    Constants.kTimeoutMs);

    // Configure Common Options
    climbMotor.setNeutralMode(NeutralMode.Brake); // Puts motor in brake mode
    climbMotor.configNeutralDeadband(0.001, Constants.kTimeoutMs); // set deadband to super small 0.001 (0.1 %), I  assume so we are less sensible to noise.
    climbMotor.setSensorPhase(false);  // Phase -- TOREAD , phase may be important
    climbMotor.setInverted(false); // Invert Motor Direction

		/* Set the peak and nominal outputs */
		climbMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		climbMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		climbMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
		climbMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		// Motion Magic Gain, similar to PIDF
		climbMotor.selectProfileSlot(Constants.kSlotIdxClimb, Constants.kPIDLoopIdxClimb);
		climbMotor.config_kF(Constants.kSlotIdxClimb, Constants.kGains_Position_Climber.getkF(), Constants.kTimeoutMs);
		climbMotor.config_kP(Constants.kSlotIdxClimb, Constants.kGains_Position_Climber.getkP(), Constants.kTimeoutMs);
		climbMotor.config_kI(Constants.kSlotIdxClimb, Constants.kGains_Position_Climber.getkI(), Constants.kTimeoutMs);
		climbMotor.config_kD(Constants.kSlotIdxClimb, Constants.kGains_Position_Climber.getkD(), Constants.kTimeoutMs);

    //Integral Zone, to read, must be something similar to a wind-up 
    climbMotor.config_IntegralZone(Constants.kSlotIdxClimb, 30);

    // Motion Magic for Velocity and Acceleration , no need for seperate PIDF controllers 
    //-- All the magic for curve is done here
		climbMotor.configMotionCruiseVelocity(0, Constants.kTimeoutMs);
		climbMotor.configMotionAcceleration(0, Constants.kTimeoutMs);

    
		//Zero the sensor once on robot boot up , need to change, to maybe use limit switch
		climbMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    //May need to include current limits
    //May need clear positions ConfigClearPositionOnLimitR()
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
      // System.out.println(Constants.leftClimberMagLimitTopID);
      // System.out.println(Constants.rightClimberMagLimitTopID);
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

  public void getStatus() {
    // System.out.println(extraClimberMagLimitBottom.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
