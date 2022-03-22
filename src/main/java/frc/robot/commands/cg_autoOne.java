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
        new c_forceShoot().withTimeout(2), // shoot ball in intake
        new c_driveStraightAuto(drive).withTimeout(0.5), // move backwards
        new WaitCommand(1), // drop intake
        new c_driveStraightAuto(drive).withTimeout(4.5)); // go in position

    ///////// LOGAN'S AUTO HELPER /////////

    // ** -> new c_forceShoot().withTimeout(SECONDS);
    // ~ run the shooter for a set amount of time ~

    // -- go back (towards driver station)
    // -- stop suddenly to drop the intake using gravity
    // -- pick up the ball in front of intake
    // -- SUDDEN JERK FORWARD to get the ball in the mag (towards target)
    // !! (SECONDS) to run these commands -> convert to distance !!
    // new c_driveStraightAuto(drive).withTimeout(0.5), // move backwards
    // new WaitCommand(1), // drop intake
    // new c_driveStraightAuto(drive).withTimeout(0.8), // ball behind us
    // new c_driveStraightBACKAuto(drive).withTimeout(0.35)); // jerk ball in

  }
}
