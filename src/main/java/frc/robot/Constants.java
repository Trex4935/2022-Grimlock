// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static double driveSpeedLimit = 0.5;
    public static double RampLimiter = 0.5;


    // endregion
    // --------------------------------------------------------

    // region Intake

    // Motor Can IDs
    public static final int intakeMotorCanID = 1;

    public static final int magazineMotor1CanID = 2;
    public static final int magazineMotor2CanID = 3;

    // Motor speeds
    public static final double magazineMotorSpeed = 0.5;
    public static final double intakeMotorSpeed = 0.5;

    // threshold for a ball to be X color
    
    public static final int magazineSensorDIO = 0;

    //color sensor required value to detect a color
    public static final int sensorRequiredValue = 1000;
    public static final int proxSensor1 = 0;
    public static final int proxSensor2 = 0;
    public static final int proxSensor3 = 0;
    public static final int proxSensor4 = 0;

    // endregion
    // --------------------------------------------------------

    // region Turret

    // Motor IDs
    public static final int turretRotationPWMID = 0;
    public static final int turretShooterCanID = 41;

    // Xbox controller trigger IDs
    public static final int leftTrigger = 2;
    public static final int rightTrigger = 3;

    // Motor speeds
    public static double returnToMiddleSpeed = 0.5;

    // Mode toggle
    public static boolean toggleAutoAimToMid = true;

    // Distance Estimation

    public static final double h2 = 2;
    public static final double angle1 = 1;
    public static final double h1 = 1;

    // endregion
    // --------------------------------------------------------

    // region Climber

    // Motor IDs
    public static final int climbMotorCanID = 0;
    public static final int rotationmotorCanID = 0;
    public static final int pinMotorCanID = 0;

    // endregion
    // --------------------------------------------------------

}
