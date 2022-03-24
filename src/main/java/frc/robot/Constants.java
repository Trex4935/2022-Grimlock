// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.extensions.PID;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  ///////////////////////////////////
  ////////////////////////
  public static final boolean testingControlMode = true;
  // public static final String controlMode = "Testing";
  ///////////////////////////////////////////////////////////

  // public static final DriverStation.Alliance allianceColor =
  // DriverStation.getAlliance();

  // region DriveTrain

  // Motor IDs
  public static final int leftFrontCanID = 34;
  public static final int leftBackCanID = 30;
  public static final int rightFrontCanID = 31;
  public static final int rightBackCanID = 32;
  public static final int middleLeftCanID = 33;
  public static final int middleRightCanID = 35;

  // Xbox controller input IDs
  public static final int leftVertical = 1;
  public static final int leftHorizontal = 0;
  public static final int rightHorizontal = 4;

  // DriveTrain limits
  public static final double driveSpeedlimitDefault = 0.75;
  public static final double driveSpeedLimitHot = 0.25;
  public static double driveSpeedLimit = Constants.driveSpeedlimitDefault;

  public static final double rotationSpeedLimitDefault = 0.6;
  public static final double rotationSpeedLimitHot = 0.25;
  public static double rotationSpeedLimit = Constants.rotationSpeedLimitDefault;

  public static final double outsideRampLimiter = 0.75;
  public static final double middleRampLimiter = 1.25;

  public static final NeutralMode outsideBrakeMode = NeutralMode.Brake;
  public static final NeutralMode middleBreakMode = NeutralMode.Coast;

  // Sensors
  public static final int sensorIRBlackValue = 6;

  // Overheat protection
  public static final double driveToHot = 90;

  // endregion
  // --------------------------------------------------------

  // region Intake

  // Motor Can IDs
  public static final int intakeMotorCanID = 3;
  public static final int magazineMotorCanID = 6;
  public static final int intakeRetractionMotorID = 2;

  // Motor speeds
  public static final double magazineMotorSpeed = 0.5;
  public static final double intakeMotorSpeed = 0.45;

  // Smakna sensor ids
  public static final int leftTrapSmakna = 3;
  public static final int rightTrapSmakna = 4;
  public static final int insideMagSmakna = 5;

  // color sensor required value to detect a color
  public static final int sensorRequiredValue = 1000;

  // prox sensor max and min values for ball detection

  // 55-65 without ball
  // 598 with ball at color sensor or 1800
  public static final int proxSensorMax = 2000;
  public static final int proxSensorMin = 100;

  // Configable intake deployment and retraction
  public static final double retractionRunTime = 2.0;
  public static final double retractionSpeed = 0.75;

  // endregion
  // --------------------------------------------------------

  public static double kPDt = 1 / 360; // gyro gives angle in degree. kP/360.

  // region Turret

  // Motor IDs
  public static final int turretRotationPWMID = 0;

  // maintain state of the intake (up or down)
  public static boolean retractionState = false;

  // Xbox controller trigger IDs
  public static final int leftTrigger = 2;
  public static final int rightTrigger = 3;

  // Motor speeds
  public static double returnToMiddleSpeed = 0.1;
  public static final double returnToMiddleSpeedLeft = returnToMiddleSpeed * -1;

  // Magnet Limit Switches
  public static final int leftMagLimitID = 2;
  public static final int middleMagID = 1;
  public static final int rightMagLimitID = 0;

  // endregion
  // --------------------------------------------------------

  // region Climber

  // Braking
  public static final NeutralMode elevatorBrakeMode = NeutralMode.Brake;

  // Motor IDs
  public static final int climbMotorCanID = 21;
  public static final int rotationMotorCanID = 4;

  // Motor Speeds
  public static final double climbMotorSpeed = 0.7;
  public static final double climbMotorSpeed2 = 0.6;
  public static final double climbRotateSpeed = 0.55;

  // Magnet Limit Switches
  public static final int leftClimberMagLimitTopID = 7;
  public static final int leftClimberMagLimitBottomID = 6;
  public static final int rightClimberMagLimitTopID = 9;
  public static final int rightClimberMagLimitBottomID = 8;
  public static final int extraClimberMagLimitBottomID = 10; // ROBORIO 10 = DIO0 NAVX0

  // endregion
  // --------------------------------------------------------

  // region Shooter
  public static final int shooterMotorCanID = 11;

  public static boolean forceShoot = false;

  // conversion factor for the falconFX encoder
  public static final double ticks2RPM = 600.0 / 2048.0;

  // Distance Estimation
  public static final double h2 = 104;// 8ft 8in to inches -> 104in
  public static final double angle1 = 22.5;
  public static final double h1 = 45.5; // 44.5in
  public static final double targetOffset = 24;

  // RPM = 3.7037 * distance + 2722.2
  public static double shooterA = 3.705;
  public static double shooterB = 2722.2;

  public static double shooterSpeed = 0.5;

  // Shooter PID
  public static PID kGains_Velocity_Shooter = new PID(0.25, 0, 0, 0.043); // Good start value per CTRE docs.
  public static int kTimeoutMs = 20;
  public static int kPIDLoopIdx = 0;

  // Shooter speeds
  public static double shooterIdleSpeed = 1250;
  public static final double shooterLowSpeed = 1000;
  public static final double shooterRange = 200;

  public static final int maximumShootDistance = 156;
  public static final int minimumShootDistance = 108;

  public static final boolean shootingLow = false;

  // Control is the shooting system is on or off
  public static boolean pewpew = true;

  // endregion
  // --------------------------------------------------------
}
