// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.Drivetrain;
import frc.robot.subsystem.Pneumatics;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cg_2BallHighAuto extends SequentialCommandGroup {
    /** Creates a new cg_autoOne. */
    public cg_2BallHighAuto(Drivetrain drive, Pneumatics pneum) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ca_jerkIntake(drive, pneum),
                new ca_moveForwardInches(drive, 60, .15), // 34.34 distance between tarmac position line and first red
                                                          // ball
                new c_forceShoot().withTimeout(4));

        // GO OFF THE TARMAC AND PICK UP THE RED BALL BEHIND US
        // SHOOT IN THAT POSITION & STAY FOR TAXI POINTS (2 BALLS)

    }
}
