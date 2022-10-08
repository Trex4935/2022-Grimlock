// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystem.LeftClimber;
import frc.robot.subsystem.RightClimber;
import frc.robot.commands.a_Default;
import frc.robot.commands.c_robotLeftClimbDown;
import frc.robot.commands.c_robotLeftClimbUp;
import frc.robot.commands.c_robotRightClimbDown;
import frc.robot.commands.c_robotRightClimbUp;

/** Add your docs here. */
public class Controller_Driver {

    public static void configDriverController(XboxController controller, LeftClimber lclimber, RightClimber rclimber) {

        // A
        new JoystickButton(controller, Button.kA.value)
                .whenHeld(new a_Default());

        // B
        new JoystickButton(controller, Button.kB.value)
                .whenHeld(new a_Default());

        // X
        new JoystickButton(controller, Button.kX.value)
                .whenHeld(new a_Default());

        // Y
        new JoystickButton(controller, Button.kY.value)
                .whenHeld(new a_Default());

        // Start
        new JoystickButton(controller, Button.kStart.value)
                .whenHeld(new a_Default());

        // Back
        new JoystickButton(controller, Button.kBack.value)
                .whenHeld(new a_Default());

        // Left Bumper
        new JoystickButton(controller, Button.kLeftBumper.value)
                .whenHeld(new c_robotLeftClimbDown(lclimber));

        // Right Bumper
        new JoystickButton(controller, Button.kRightBumper.value)
                .whenHeld(new c_robotRightClimbDown(rclimber));

        // Left trigger as button
        new leftTriggerBool(controller)
                .whileActiveContinuous(new c_robotLeftClimbUp(lclimber));

        // Right trigger as button
        new rightTriggerBool(controller)
                .whileActiveContinuous(new c_robotRightClimbUp(rclimber));

        // Top POV
        new POVButton(controller, 0)
                .whenHeld(new a_Default());

        // Right POV
        new POVButton(controller, 90)
                .whenHeld(new a_Default());

        // Bottom POV
        new POVButton(controller, 180)
                .whenHeld(new a_Default());

        // Left POV
        new POVButton(controller, 270)
                .whenHeld(new a_Default());

    }
}
