// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// FIELD : https://firstfrc.blob.core.windows.net/frc2022/FieldAssets/2022LayoutMarkingDiagram.pdf

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.Drivetrain;
import frc.robot.subsystem.Pneumatics;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cg_2BallLowAuto extends SequentialCommandGroup {
    /** Creates a new cg_autoOne. */
    public cg_2BallLowAuto(Drivetrain drive, Pneumatics pneum) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ca_jerkIntake(drive, pneum),
                new ca_moveForwardInches(drive, 42, .50), // 34.34 distance between tarmac position line and first red
                                                          // ball
                new ca_moveForwardInches(drive, -152, -.25), // 153 in from red ball to target
                new c_forceShoot().withTimeout(4),
                new WaitCommand(4),
                new ca_moveForwardInches(drive, 130, .50));

        // GO OFF THE TARMAC AND PICK UP THE RED BALL BEHIND US
        // DRIVE FORWARD TOWARDS THE TARGET & SHOOT 2 BALLS
        // BACK UP OFF THE TAXI

    }
}
