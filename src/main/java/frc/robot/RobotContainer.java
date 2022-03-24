// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.c_detectShootingReady;
import frc.robot.commands.c_driveWithController;
import frc.robot.commands.c_flipPewPew;
import frc.robot.commands.c_forceShoot;
import frc.robot.commands.c_robotClimbsUp;
import frc.robot.commands.c_robotClimbsDown;
import frc.robot.commands.c_pullUp;
import frc.robot.commands.c_rotateAndUpClimb;
import frc.robot.commands.c_runIntakeRetractionMotor;
import frc.robot.commands.c_shootBall;
import frc.robot.commands.c_shooterShootLow;
import frc.robot.commands.cg_autoOne;
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
  private JoystickButton xbox_a, xbox_x, xbox_y, xbox_b, xbox_start, xbox_rbump, xbox_lbump;
  private JoystickButton c_xbox_a, c_xbox_x, c_xbox_y, c_xbox_b, c_xbox_start;
  private POVButton xbox_pov_up, xbox_pov_down;

  // Auto configuration
  private cg_autoOne auto;

  public RobotContainer() {

    auto = new cg_autoOne(drive);

    //////////////////////////////////////////////////////////////////////////
    // Use during competition and remove "debug code section"
    // competitionConfiguration();
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////
    // DEBUG CODE SECTION //
    if (Constants.testingControlMode) {
      testConfiguration();
    } else {
      competitionConfiguration();
    }
    //////////////////////////////////////////////////////////////////////////
  }

  private void competitionConfiguration() {
    // Setup default drive controls
    drive.setDefaultCommand(new c_driveWithController(drive, controller, coDriverController));
    shooter.setDefaultCommand(new c_detectShootingReady(intake, shooter, turret, coDriverController));

    // Configure the drive button bindings
    configureButtonBindingsCompetitionDriver();

    // Configure the codriver button bindings
    configureButtonBindingsCompetitionCoDriver();

    xbox_lbump = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
    xbox_lbump.whenHeld(new c_shooterShootLow());
  }
  
  private void configureButtonBindingsCompetitionCoDriver() {
    ///// CoDriver Controller /////
    // Set the A button
    c_xbox_a = new JoystickButton(coDriverController, XboxController.Button.kA.value);
    c_xbox_a.toggleWhenPressed(new c_rotateAndUpClimb(climber));

    // Set the B button
    c_xbox_b = new JoystickButton(coDriverController, XboxController.Button.kB.value);
    c_xbox_b.toggleWhenPressed(new c_pullUp(climber));

    // Set the Y button
    // Lower the Robot
    c_xbox_y = new JoystickButton(coDriverController, XboxController.Button.kY.value);
    c_xbox_y.toggleWhenPressed(new c_robotClimbsDown(climber).withTimeout(0.75));

    // Raise and lower the intake
    c_xbox_x = new JoystickButton(coDriverController, XboxController.Button.kX.value);
    c_xbox_x.toggleWhenPressed(new c_runIntakeRetractionMotor(intake).withTimeout(0.75));

    // Turn off the shooting subsystem
    c_xbox_start = new JoystickButton(coDriverController, XboxController.Button.kStart.value);
    c_xbox_start.whenPressed(new c_flipPewPew());
  }

  private void configureButtonBindingsCompetitionDriver() {

    ////// Primary Controller /////
    xbox_rbump = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
    xbox_rbump.whenHeld(new c_forceShoot());

  }

  // TEST Controller configuration //
  private void testConfiguration() {

    // Setup default drive controls
    drive.setDefaultCommand(new c_driveWithController(drive, controller, coDriverController));
    // turret.setDefaultCommand(new c_aimWithController(turret, controller));
    // intake.setDefaultCommand(new c_runIntakeMotor(intake));
    // intake.setDefaultCommand(new c_runMagazineMotors(intake));
    shooter.setDefaultCommand(new c_detectShootingReady(intake, shooter, turret,
        controller));

    // Configure the button bindings
    xbox_a = new JoystickButton(controller, XboxController.Button.kA.value);
    xbox_a.toggleWhenPressed(new c_rotateAndUpClimb(climber));
    // xbox_a.toggleWhenPressed(new c_returnToMiddle(turret));

    xbox_b = new JoystickButton(controller, XboxController.Button.kB.value);
    xbox_b.toggleWhenPressed(new c_pullUp(climber));
    // xbox_b.toggleWhenPressed(new c_runIntakeMotor(intake));

    xbox_y = new JoystickButton(controller, XboxController.Button.kY.value);
    // xbox_y.whenHeld(new c_runIntakeRetractionMotor(intake));
    xbox_y.toggleWhenPressed(new c_shootBall(shooter));
    // xbox_y.toggleWhenPressed(new c_rotateAndUpClimb(climber));
    // xbox_y.toggleWhenPressed(new c_runShooterPID(shooter, 2000));
    // xbox_y.whenHeld(new c_rotateClimbTowardsIntake(climber));

    xbox_x = new JoystickButton(controller, XboxController.Button.kX.value);
    // xbox_x.toggleWhenPressed(new c_runShooterPID(shooter, 4000));
    xbox_x.whenHeld(new c_shootBall(shooter));
    // xbox_x.toggleWhenPressed(new c_detectShootingReady(intake, shooter));

    xbox_pov_down = new POVButton(controller, 180);
    xbox_pov_down.whileHeld(new c_robotClimbsUp(climber));

    xbox_pov_up = new POVButton(controller, 0);
    xbox_pov_up.whileHeld(new c_robotClimbsDown(climber));

    // xbox_pov_left = new POVButton(controller, 270);
    // LEFT ON CONTROLLER D-PAD
    // xbox_pov_left.whileHeld(new c_rotateClimbTowardsShooter(climber));

    // xbox_pov_right = new POVButton(controller, 90);
    // RIGHT ON CONTROLLER D-PAD
    // xbox_pov_right.whileHeld(new c_rotateClimbTowardsIntake(climber));

    // Turn off the shooting subsystem
    xbox_start = new JoystickButton(controller, XboxController.Button.kStart.value);
    xbox_start.whenPressed(new c_flipPewPew());

  }

  // .withInterrupt(Magazine::getShooterSensor).andThen(reverseMagazine2.withTimeout(0.1)).andThen(shoot));

  public Command getAutonomousCommand() {
    // releaseIntake.andThen(
    return auto;
    // return auto.withTimeout(.1).andThen(new
    // WaitCommand(1).andThen(auto).withTimeout(0.4));
  }
}
