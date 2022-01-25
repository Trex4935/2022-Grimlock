// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.driveWithController;
import frc.robot.commands.runIntakeMotor;
import frc.robot.commands.runMagazineMotors;
import frc.robot.subsystem.Drivetrain;
import frc.robot.subsystem.Intake;

public class RobotContainer {

  // subsystem
  public final Drivetrain drive;
  public final Drivetrain middle;
  public static XboxController controller;
  public final Intake intake;

  // commands
  private final driveWithController driveWithController;
  private final runIntakeMotor runIntakeMotor;
  private final runMagazineMotors runMagazineMotors;

  public RobotContainer() {
    // Drivetrain
    drive = new Drivetrain();
    driveWithController = new driveWithController(drive);
    //driveWithController.addRequirements(drive);
    drive.setDefaultCommand(driveWithController);
    middle = new Drivetrain();

    // Intake and Magazine
    intake = new Intake();
    runIntakeMotor = new runIntakeMotor(intake);
    //runIntakeMotor.addRequirements(intake);
    runMagazineMotors = new runMagazineMotors(intake);
    //runMagazineMotors.addRequirements(intake);

    controller = new XboxController(0);

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

    new JoystickButton(controller, XboxController.Button.kB.value).toggleWhenPressed(runIntakeMotor);
    new JoystickButton(controller, XboxController.Button.kY.value).toggleWhenPressed(runMagazineMotors);

  }

  // .withInterrupt(Magazine::getShooterSensor).andThen(reverseMagazine2.withTimeout(0.1)).andThen(shoot));

  public Command getAutonomousCommand() {
    return driveWithController;
  }
}
