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
import frc.robot.commands.c_aimWithController;
import frc.robot.commands.c_driveWithController;
import frc.robot.commands.c_readSensor;
import frc.robot.commands.c_runIntakeMotor;
import frc.robot.commands.c_runMagazineMotors;
import frc.robot.commands.c_turnOnSimpleAutoAim;
import frc.robot.subsystem.Drivetrain;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Turret;
import frc.robot.subsystem.TurretPID;

public class RobotContainer {

  // Declare Subsystems
//  private final Drivetrain drive = new Drivetrain();
  private final Intake intake = new Intake();
  // private final TurretPID turretPID = new TurretPID();
  private final Turret turret = new Turret();

  // Controller
  private static XboxController controller = new XboxController(0);
  // button variables for the controller
  private JoystickButton xbox_b, xbox_a, xbox_y, xbox_x;

  public RobotContainer() {
    // Setup default drive controls
//    drive.setDefaultCommand(new c_driveWithController(drive, controller));
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

   // xbox_a = new JoystickButton(controller, XboxController.Button.kA.value);
   // xbox_a.toggleWhenPressed(new c_turnOnSimpleAutoAim(turret));

    //xbox_x = new JoystickButton(controller,XboxController.Button.kX.value);
    //xbox_x.toggleWhenPressed(new InstantCommand(turret::enable, turret));
  }

  // .withInterrupt(Magazine::getShooterSensor).andThen(reverseMagazine2.withTimeout(0.1)).andThen(shoot));

  public Command getAutonomousCommand() {
    return null;
  }
}
