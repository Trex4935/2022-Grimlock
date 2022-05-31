// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.a_Default;
import frc.robot.commands.c_ArmsGoDownMotionMagic;
import frc.robot.commands.c_ArmsGoUpMotionMagic;
import frc.robot.commands.c_flipshootingSubsystemOn;
import frc.robot.commands.c_robotClimbsDown;
import frc.robot.commands.c_robotClimbsUp;
import frc.robot.subsystem.Climber;

/** Add your docs here. */
public class Controller_Test {

        public static void configTestController(XboxController controller, Climber climber) {

                // A
                new JoystickButton(controller, Button.kA.value)
                                .whenPressed(new c_ArmsGoDownMotionMagic(climber));

                // B
                new JoystickButton(controller, Button.kB.value)
                                .whenPressed(new c_ArmsGoUpMotionMagic(climber));

                // X
                new JoystickButton(controller, Button.kX.value)
                                .whenHeld(new a_Default());

                // Y
                new JoystickButton(controller, Button.kY.value)
                                .whenHeld(new a_Default());

                // Start
                new JoystickButton(controller, Button.kStart.value)
                                .whenPressed(new c_flipshootingSubsystemOn());

                // Back
                new JoystickButton(controller, Button.kBack.value)
                                .whenHeld(new a_Default());

                // Left Bumper
                new JoystickButton(controller, Button.kLeftBumper.value)
                                .whenHeld(new a_Default());

                // Right Bumper
                new JoystickButton(controller, Button.kRightBumper.value)
                                .whenHeld(new a_Default());

                // Left trigger as button
                new leftTriggerBool(controller)
                                .whileActiveContinuous(new a_Default());

                // Right trigger as button
                new rightTriggerBool(controller)
                                .whileActiveContinuous(new a_Default());

                // Top POV
                new POVButton(controller, 0)
                                .whileHeld(new c_robotClimbsDown(climber));

                // Right POV
                new POVButton(controller, 90)
                                .whenHeld(new a_Default());

                // Bottom POV
                new POVButton(controller, 180)
                                .whileHeld(new c_robotClimbsUp(climber));

                // Left POV
                new POVButton(controller, 270)
                                .whenHeld(new a_Default());

        }

}
