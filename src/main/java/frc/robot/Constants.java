// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DriverStation;
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

    ///////////////////////////////////////////////////////////
    public static final boolean testingControlMode = true;
    // public static final String controlMode = "Testing";
    ///////////////////////////////////////////////////////////

    // region General

    public static final DriverStation.Alliance allianceColor = DriverStation.getAlliance();

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
    public static double driveSpeedLimit = 0.75;

    public static double outsideRampLimiter = 0.75;
    public static double middleRampLimiter = 1.25;

    public static final NeutralMode outsideBrakeMode = NeutralMode.Brake;
    public static final NeutralMode middleBreakMode = NeutralMode.Coast;

    // Sensors
    public static final int sensorIRBlackValue = 6;

    // endregion
    // --------------------------------------------------------

    // region Intake

    // Motor Can IDs
    public static final int intakeMotorCanID = 3;
    public static final int magazineMotor1CanID = 6;
    public static final int intakeRetractionMotorID = 2;

    // Motor speeds
    public static final double magazineMotorSpeed = 0.5;
    public static final double intakeMotorSpeed = 0.8;

    // Smakna sensor ids
    public static final int magazineSensor1DIO = 3;
    public static final int magazineSensor2DIO = 4;
    public static final int magazineSensor3DIO = 5;

    // color sensor required value to detect a color
    public static final int sensorRequiredValue = 1000;

    // prox sensor max and min values for ball detection

    // 55-65 without ball
    // 598 with ball at color sensor or 1800
    public static final int proxSensorMax = 2000;
    public static final int proxSensorMin = 100;

    // Configable intake deployment and retraction
    public static final double retractionRunTime = 2.0;
    public static final double retractionSpeed = 0.5;

    // endregion
    // --------------------------------------------------------

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
    public static final double climbMotorSpeed = 0.8;
    public static final double climbRotateSpeed = 0.8;

    // Magnet Limit Switches
    public static final int leftClimberMagLimitTopID = 7;
    public static final int leftClimberMagLimitBottomID = 6;
    public static final int rightClimberMagLimitTopID = 9;
    public static final int rightClimberMagLimitBottomID = 8;

    // endregion
    // --------------------------------------------------------

    // region Shooter
    public static final int shooterMotorCanID = 11;

    // conversion factor for the falconFX encoder
    public static final double ticks2RPM = 600.0 / 2048.0;

    // Distance Estimation
    public static final double h2 = 2;
    public static final double angle1 = 1;
    public static final double h1 = 1;

    public static double shooterA = 1;
    public static double shooterB = 2;

    // RPM = 3.7037 * distance + 2722.2
      //public static double shooterA = 3.7037;
      //public static double shooterB = 2722.2;

  public static double shooterSpeed = 0.3;

    // Shooter PID
    public static PID kGains_Velocity_Shooter = new PID(0.24, 0, 0, 0.044); // Good start value per CTRE docs.
    public static int kTimeoutMs = 20;
    public static int kPIDLoopIdx = 0;

    // Shooter speeds
    public static final double shooterIdleSpeed = 3000;
    public static final double shooterLowSpeed = 2000;
    public static final double shooterRange = 100;

    // endregion
    // --------------------------------------------------------
}
