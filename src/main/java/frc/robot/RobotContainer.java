// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.command_archive.c_runMagazineMotors;
import frc.robot.commands.c_aimWithController;
import frc.robot.commands.c_driveStraightAuto;
import frc.robot.commands.c_driveTurnAuto;
import frc.robot.commands.c_releaseIntake;
import frc.robot.commands.c_detectShootingReady;
import frc.robot.commands.c_driveStraightAngleAuto;
import frc.robot.commands.c_driveWithController;
import frc.robot.commands.c_motorClimbDown;
import frc.robot.commands.c_motorClimbUp;
import frc.robot.commands.c_pullUp;
import frc.robot.commands.c_returnToMiddle;
import frc.robot.commands.c_rotateAndUpClimb;
import frc.robot.commands.c_rotateClimbTowardsShooter;
import frc.robot.commands.c_runIntakeMotor;
import frc.robot.commands.c_runIntakeRetractionMotor;
import frc.robot.commands.c_runShooterPID;
import frc.robot.commands.c_rotateClimbTowardsIntake;
import frc.robot.commands.c_shootBall;
import frc.robot.commands.c_singulateBall;
import frc.robot.commands.c_turnOnSimpleAutoAim;
import frc.robot.commands.c_turnOnPIDAutoAim;
import frc.robot.commands.cg_phaseThree;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.Drivetrain;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Turret;

public class RobotContainer {

  // Declare Subsystems
  private final Drivetrain drive = new Drivetrain();
  private final Intake intake = new Intake();
  private final Turret turret = new Turret();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();

  // Controller
  private static XboxController controller = new XboxController(0);
  private static XboxController coDriverController = new XboxController(1);

  // button variables for the controller
  private JoystickButton xbox_a, xbox_x, xbox_y, xbox_b;
  private JoystickButton c_xbox_a, c_xbox_x, c_xbox_y, c_xbox_b;
  private POVButton xbox_pov_up, xbox_pov_down, xbox_pov_left, xbox_pov_right;

  c_driveStraightAngleAuto backingUpAuto;
  c_driveStraightAngleAuto backingUpAt45Auto;
  c_driveStraightAngleAuto backingUpAt90Auto;
  c_driveTurnAuto turnAuto;
  c_releaseIntake releaseIntake;
  c_detectShootingReady rdyshot;
  //c_driveStraightAuto auto;

  public RobotContainer() {

    //auto = new c_driveStraightAuto(drive);
    backingUpAuto = new c_driveStraightAngleAuto(drive,0);
    backingUpAt45Auto = new c_driveStraightAngleAuto(drive,45);
    backingUpAt90Auto = new c_driveStraightAngleAuto(drive,90);
    turnAuto = new c_driveTurnAuto(drive,45);
    rdyshot = new c_detectShootingReady(intake, shooter, turret, coDriverController);

    // load control profile based on if we are in testing or competition mode
    /////////// TESTING PROFILE ///////////
    if (Constants.testingControlMode) {
      // Setup default drive controls
      // drive.setDefaultCommand(new c_driveWithController(drive, controller));
      // turret.setDefaultCommand(new c_aimWithController(turret, controller));
      // intake.setDefaultCommand(new c_runIntakeMotor(intake));
      intake.setDefaultCommand(new c_runMagazineMotors(intake));
      // shooter.setDefaultCommand(new c_detectShootingReady(intake, shooter,
      // turret));

      // Configure the button bindings
      configureButtonBindingsTesting();

    }
    /////////// COMPETITION PROFILE ///////////
    else {
      // Setup default drive controls
      drive.setDefaultCommand(new c_driveWithController(drive, controller));
      // turret.setDefaultCommand(new c_turnOnPIDAutoAim(turretPID));
      // intakeSingulate.setDefaultCommand(new c_singulateBall(intakeSingulate));
      shooter.setDefaultCommand(new c_detectShootingReady(intake, shooter, turret, coDriverController));

      // Configure the button bindings
      configureButtonBindingsCompetition();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  // controller map for competition
  /////////// COMPETITION PROFILE ///////////

  private void configureButtonBindingsCompetition() {

    // Set the A button
    xbox_a = new JoystickButton(controller, XboxController.Button.kA.value);
    xbox_a.toggleWhenPressed(new c_rotateAndUpClimb(climber));

    // Set the B button
    xbox_b = new JoystickButton(controller, XboxController.Button.kB.value);
    xbox_b.toggleWhenPressed(new c_pullUp(climber));

    xbox_y = new JoystickButton(controller, XboxController.Button.kY.value);
    xbox_y.whenHeld(new c_runIntakeRetractionMotor(intake));

    /// CONTROLLER MAP
    //
    // A - Rotate and Extend Arms
    // B - Lift Robot off the Ground
    // X -
    // Y -
    //
    // LT -
    // RT -
    //
    // LB -
    // RB -
    //
    // LStick Vertical - Drive forward/backward
    // LStick Horizontal - H Drive Left/Right (Strafe)
    // RStick - Rotate left/right
    //
    // Start -
    // Select -
    //
    // D-Pad
    // Up -
    // Right -
    // Down -
    // Left -
    //
    /// END MAP

  }

  // controller map for testing the robot
  /////////// TESTING PROFILE ///////////
  private void configureButtonBindingsTesting() {

    ///// PRIMARY CONTROLLER /////
    xbox_a = new JoystickButton(controller, XboxController.Button.kA.value);
    xbox_a.toggleWhenPressed(new c_rotateAndUpClimb(climber));
    // xbox_a.toggleWhenPressed(new c_returnToMiddle(turret));

    xbox_b = new JoystickButton(controller, XboxController.Button.kB.value);
    xbox_b.toggleWhenPressed(new c_pullUp(climber));
    // xbox_b.toggleWhenPressed(new c_runIntakeMotor(intake));

    xbox_y = new JoystickButton(controller, XboxController.Button.kY.value);
    xbox_y.whenHeld(new c_runIntakeRetractionMotor(intake));
    // xbox_y.toggleWhenPressed(new c_shootBall(shooter));
    // xbox_y.toggleWhenPressed(new c_rotateAndUpClimb(climber));
    // xbox_y.toggleWhenPressed(new c_runShooterPID(shooter, 2000));
    // xbox_y.whenHeld(new c_rotateClimbTowardsIntake(climber));

    xbox_x = new JoystickButton(controller, XboxController.Button.kX.value);
    // xbox_x.toggleWhenPressed(new c_runShooterPID(shooter, 4000));
    xbox_x.whenHeld(new c_shootBall(shooter));
    // xbox_x.toggleWhenPressed(new c_detectShootingReady(intake, shooter));

    xbox_pov_down = new POVButton(controller, 180);
    xbox_pov_down.whileHeld(new c_motorClimbDown(climber));

    xbox_pov_up = new POVButton(controller, 0);
    xbox_pov_up.whileHeld(new c_motorClimbUp(climber));

    // xbox_pov_left = new POVButton(controller, 270);
    // LEFT ON CONTROLLER D-PAD
    // xbox_pov_left.whileHeld(new c_rotateClimbTowardsShooter(climber));

    // xbox_pov_right = new POVButton(controller, 90);
    // RIGHT ON CONTROLLER D-PAD
    // xbox_pov_right.whileHeld(new c_rotateClimbTowardsIntake(climber));

    ///// CODRIVER CONTROLLER /////
    // c_xbox_a = new
    ///// JoystickButton(coDriverController,XboxController.Button.kA.value);

    /// CONTROLLER MAP
    //
    // A - Return turret to middle
    // B - Toggle Singulation
    // X - Tooggle shooter
    // Y - Toggle Magazine
    //
    // LT - Move turret Left
    // RT - Move turret Right
    //
    // LB -
    // RB - Toggle turret autoaim
    //
    // LStick Vertical - Drive forward/backward
    // LStick Horizontal - H Drive Left/Right (Strafe)
    // RStick - Rotate left/right
    //
    // Start -
    // Select -
    //
    // D-Pad
    // Up - Climber Arms up (robot down)
    // Right - Rotate climber arms right
    // Down - Climber arms down (robot up)
    // Left - Rotate climber arms left
    //
    /// END MAP

  }

  // .withInterrupt(Magazine::getShooterSensor).andThen(reverseMagazine2.withTimeout(0.1)).andThen(shoot));

  public Command getAutonomousCommand() {
    // tst ato
    return backingUpAuto.withTimeout(1.5).andThen(rdyshot.andThen(turnAuto.andThen(backingUpAt45Auto.withTimeout(1.5).andThen(rdyshot))));

    //works auto
    //return backingUpAuto.withTimeout(1.5).andThen(turnAuto.andThen(backingUpAt45Auto.withTimeout(1.5))) ;
  
  //angle of movment based on pos of rbot for ref see rapid react layout and marking diagram page 4 
  //each angle option is orginized by red or blue side and top to bottom slot top being 1 and bottom being 3
  // this is assuming origin is set based on robot positon as i dont know if their is a senseor on the robot that tracks that

  //slot 1 blue  backingUpAt90Auto
  //return backingUpAuto.withTimeout(1.5).andThen(rdyshot.andThen(turnAuto.andThen(backingUpAt90Auto.withTimeout(1.2).andThen(rdyshot))));
  
  //slot 2 blue
  
  
  }
}
