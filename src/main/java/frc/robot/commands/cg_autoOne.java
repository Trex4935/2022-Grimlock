// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cg_autoOne extends SequentialCommandGroup {
  /** Creates a new cg_autoOne. */
  public cg_autoOne(Drivetrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new c_driveStraightAuto(drive).withTimeout(0.5),
        new WaitCommand(1),
        new c_driveStraightAuto(drive).withTimeout(0.8),
        new c_driveStraightBACKAuto(drive).withTimeout(0.35));
  }
}
