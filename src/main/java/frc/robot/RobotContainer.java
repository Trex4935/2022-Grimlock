// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.c_aimWithController;
import frc.robot.commands.c_driveWithController;
import frc.robot.commands.c_motorClimbDown;
import frc.robot.commands.c_motorClimbUp;
import frc.robot.commands.c_readSensor;
import frc.robot.commands.c_redBlueDecision;
import frc.robot.commands.c_returnToMiddle;
import frc.robot.commands.c_rotateClimbLeft;
import frc.robot.commands.c_rotateClimbRight;
import frc.robot.commands.c_runIntakeMotor;
import frc.robot.commands.c_runMagazineMotors;
import frc.robot.commands.c_shootBall;
import frc.robot.commands.c_turnOnSimpleAutoAim;
import frc.robot.extensions.Limelight;
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

  // button variables for the controller
  private JoystickButton xbox_b, xbox_a, xbox_y, xbox_x;
  private POVButton xbox_pov_up, xbox_pov_down, xbox_pov_left, xbox_pov_right;

  public RobotContainer() {
    // Setup default drive controls
    drive.setDefaultCommand(new c_driveWithController(drive, controller));
    turret.setDefaultCommand(new c_aimWithController(turret, controller));
    intake.setDefaultCommand(new c_readSensor(intake));

    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {

    xbox_b = new JoystickButton(controller, XboxController.Button.kB.value);
    xbox_b.toggleWhenPressed(new c_runIntakeMotor(intake));

    xbox_y = new JoystickButton(controller, XboxController.Button.kY.value);
    xbox_y.toggleWhenPressed(new c_runMagazineMotors(intake));

    xbox_a = new JoystickButton(controller, XboxController.Button.kA.value);
    xbox_a.toggleWhenPressed(new c_returnToMiddle(turret));

    xbox_x = new JoystickButton(controller, XboxController.Button.kX.value);
    xbox_x.toggleWhenPressed(new c_shootBall(shooter));

    xbox_pov_down = new POVButton(controller, 180);
    xbox_pov_down.whileHeld(new c_motorClimbDown(climber));

    xbox_pov_up = new POVButton(controller, 0);
    xbox_pov_up.whileHeld(new c_motorClimbUp(climber));

    xbox_pov_left = new POVButton(controller, 270);
    xbox_pov_left.whileHeld(new c_rotateClimbLeft(climber));

    xbox_pov_right = new POVButton(controller, 90);
    xbox_pov_right.whileHeld(new c_rotateClimbRight(climber));

  }

  // .withInterrupt(Magazine::getShooterSensor).andThen(reverseMagazine2.withTimeout(0.1)).andThen(shoot));

  public Command getAutonomousCommand() {
    return null;
  }
}
