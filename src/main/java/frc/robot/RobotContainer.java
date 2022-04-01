// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.c_changeClimberToCoastMode;
import frc.robot.commands.c_detectShootingReady;
import frc.robot.commands.c_driveWithController;
import frc.robot.commands.c_flipshootingSubsystemOn;
import frc.robot.commands.c_forceShoot;
import frc.robot.commands.c_robotClimbsUp;
import frc.robot.commands.c_ArmsGoUpMotionMagicL;
import frc.robot.commands.c_ArmsGoUpMotionMagicR;
import frc.robot.commands.c_robotClimbsDown;
import frc.robot.commands.c_ArmsGoDownMotionMagic;
import frc.robot.commands.c_ArmsGoDownMotionMagicL;
import frc.robot.commands.c_ArmsGoDownMotionMagicR;
import frc.robot.commands.c_ArmsGoUpMotionMagic;
import frc.robot.commands.c_runIntakeRetractionMotor;
import frc.robot.commands.cg_1BallHighAuto;
import frc.robot.commands.cg_1BallLowAuto;
import frc.robot.commands.cg_2BallHighAuto;
import frc.robot.commands.cg_2BallLowAuto;
import frc.robot.extensions.rightTriggerBool;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.Drivetrain;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Pneumatics;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Turret;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {

  // Declare Subsystems
  private final Drivetrain drive = new Drivetrain();
  private final Intake intake = new Intake();
  private final Turret turret = new Turret();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final Pneumatics pneum = new Pneumatics();

  // Controller
  private static XboxController controller = new XboxController(0);
  private static XboxController coDriverController = new XboxController(1);

  // button variables for the controller
  private JoystickButton xbox_a, xbox_x, xbox_y, xbox_b, xbox_start, xbox_rbump;
  private JoystickButton c_xbox_a, c_xbox_x, c_xbox_y, c_xbox_b, c_xbox_start;
  private POVButton xbox_pov_up, xbox_pov_down, xbox_pov_right, xbox_pov_left;
  private rightTriggerBool xbox_rTrig;

  // SendableChooser to pick automomous runs
  SendableChooser<Command> AutoRun_Picker = new SendableChooser<>();

  // Auto configuration
  private cg_1BallHighAuto auto1;
  private cg_1BallLowAuto auto2;
  private cg_2BallHighAuto auto3;
  private cg_2BallLowAuto auto4;

  public RobotContainer() {

    auto1 = new cg_1BallHighAuto(drive);
    auto2 = new cg_1BallLowAuto(drive);
    auto3 = new cg_2BallHighAuto(drive);
    auto4 = new cg_2BallLowAuto(drive);

    // Autonomous Chooser
    AutoRun_Picker.setDefaultOption("1 Ball High Auto Run", auto1);
    AutoRun_Picker.addOption("1 Ball Low Auto Run", auto2);
    AutoRun_Picker.addOption("2 Ball High Auto Run", auto3);
    AutoRun_Picker.addOption("2 Ball Low Auto Run", auto4);
    SmartDashboard.putData(AutoRun_Picker);

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

  }

  private void configureButtonBindingsCompetitionCoDriver() {
    ///// CoDriver Controller /////
    // Set the A button
    c_xbox_a = new JoystickButton(coDriverController, XboxController.Button.kA.value);
    c_xbox_a.toggleWhenPressed(new c_ArmsGoUpMotionMagic(climber));

    // Set the B button
    c_xbox_b = new JoystickButton(coDriverController, XboxController.Button.kB.value);
    c_xbox_b.toggleWhenPressed(new c_ArmsGoDownMotionMagic(climber));

    // Set the Y button
    // Lower the Robot
    c_xbox_y = new JoystickButton(coDriverController, XboxController.Button.kY.value);
    c_xbox_y.toggleWhenPressed(new c_robotClimbsDown(climber).withTimeout(0.75));

    // Raise and lower the intake
    c_xbox_x = new JoystickButton(coDriverController, XboxController.Button.kX.value);
    c_xbox_x.toggleWhenPressed(new c_runIntakeRetractionMotor(intake).withTimeout(0.75));

    // Turn off the shooting subsystem
    c_xbox_start = new JoystickButton(coDriverController, XboxController.Button.kStart.value);
    c_xbox_start.whenPressed(new c_flipshootingSubsystemOn());
  }

  private void configureButtonBindingsCompetitionDriver() {

    ////// Primary Controller /////
    xbox_rTrig = new rightTriggerBool(controller);
    xbox_rTrig.whileActiveContinuous(new c_forceShoot());

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
    // xbox_a.toggleWhenPressed(new ca_moveForwardInches(drive, 12, 0.25));
    // xbox_a.toggleWhenPressed(new c_rotateAndUpClimb(climber));
    // xbox_a.toggleWhenPressed(new c_returnToMiddle(turret));
    xbox_a.toggleWhenPressed(new c_ArmsGoUpMotionMagic(climber));

    xbox_b = new JoystickButton(controller, XboxController.Button.kB.value);
    // xbox_b.toggleWhenPressed(new c_pullUp(climber));
    // xbox_b.toggleWhenPressed(new c_runIntakeMotor(intake));
    xbox_b.toggleWhenPressed(new c_ArmsGoDownMotionMagic(climber));

    xbox_y = new JoystickButton(controller, XboxController.Button.kY.value);
    xbox_y.toggleWhenPressed(new c_ArmsGoUpMotionMagicR(climber));
    // xbox_y.whenHeld(new c_runIntakeRetractionMotor(intake));
    // xbox_y.toggleWhenPressed(new c_shootBall(shooter));
    // xbox_y.toggleWhenPressed(new c_rotateAndUpClimb(climber));
    // xbox_y.toggleWhenPressed(new c_runShooterPID(shooter, 2000));
    // xbox_y.whenHeld(new c_rotateClimbTowardsIntake(climber));

    xbox_x = new JoystickButton(controller, XboxController.Button.kX.value);
    xbox_x.toggleWhenPressed(new c_ArmsGoUpMotionMagicL(climber));
    // xbox_x.toggleWhenPressed(new c_runShooterPID(shooter, 4000));
    // xbox_x.whenHeld(new c_shootBall(shooter));
    // xbox_x.toggleWhenPressed(new c_detectShootingReady(intake, shooter));
    // xbox_x.whenHeld(new c_changeClimberToCoastMode(climber));

    xbox_pov_right = new POVButton(controller, 90);
    xbox_pov_right.toggleWhenPressed(new c_ArmsGoDownMotionMagicR(climber));

    xbox_pov_left = new POVButton(controller, 270);
    xbox_pov_left.toggleWhenPressed(new c_ArmsGoDownMotionMagicL(climber));

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
    xbox_start.whenPressed(new c_flipshootingSubsystemOn());

    xbox_rTrig = new rightTriggerBool(controller);
    xbox_rTrig.whileActiveContinuous(new c_forceShoot());

  }

  // .withInterrupt(Magazine::getShooterSensor).andThen(reverseMagazine2.withTimeout(0.1)).andThen(shoot));

  public Command getAutonomousCommand() {

    // Poll the SmartDashboard for the Autonomus Run Selection
    return AutoRun_Picker.getSelected();

    // releaseIntake.andThen(
    // return auto;
    // return auto.withTimeout(.1).andThen(new
    // WaitCommand(1).andThen(auto).withTimeout(0.4));
  }
}
