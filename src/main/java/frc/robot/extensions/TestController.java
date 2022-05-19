// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.c_Default;

/** Add your docs here. */
public class TestController {

    public static void configTestController(XboxController controller) {

        // A
        new JoystickButton(controller, Button.kA.value)
                .whenPressed(new c_Default());

        // B
        new JoystickButton(controller, Button.kB.value)
                .whenPressed(new c_Default());

        // X
        new JoystickButton(controller, Button.kX.value)
                .whenPressed(new c_Default());

        // Y
        new JoystickButton(controller, Button.kY.value)
                .whenPressed(new c_Default());

        // Start
        new JoystickButton(controller, Button.kStart.value)
                .whenPressed(new c_Default());

        // Back
        new JoystickButton(controller, Button.kBack.value)
                .whenPressed(new c_Default());

        // Left Bumper
        new JoystickButton(controller, Button.kLeftBumper.value)
                .whenPressed(new c_Default());

        // Right Bumper
        new JoystickButton(controller, Button.kRightBumper.value)
                .whenPressed(new c_Default());

        // Left trigger as button
        new leftTriggerBool(controller)
                .whileActiveContinuous(new c_Default());

        // Right trigger as button
        new rightTriggerBool(controller)
                .whileActiveContinuous(new c_Default());

    }

}
