// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystem.LeftClimber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class c_ArmsGoDownMotionMagic extends ParallelCommandGroup {
  /** Creates a new c_ArmsGoDownMotionMagic. */
  public c_ArmsGoDownMotionMagic(LeftClimber cl) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new c_ArmsGoDownMotionMagicR(cl),
        new c_ArmsGoDownMotionMagicL(cl)

    );
  }
}
