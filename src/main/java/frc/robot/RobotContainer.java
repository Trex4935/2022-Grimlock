// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.driveWithController;
import frc.robot.subsystem.Drivetrain;

public class RobotContainer {

  // subsystem
  public final Drivetrain drive;
  public final Drivetrain middle;
  public static XboxController controller;

  // commands
  private final driveWithController driveWithController;

  public RobotContainer() {
    // Drivetrain
    drive = new Drivetrain();
    driveWithController = new driveWithController(drive);
    driveWithController.addRequirements(drive);
    drive.setDefaultCommand(driveWithController);
    middle = new Drivetrain();

    controller = new XboxController(0);

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
  }

  public Command getAutonomousCommand() {
    return driveWithController;
  }
}
