// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.a_Default;
import frc.robot.commands.c_flipshootingSubsystemOn;
import frc.robot.commands.c_forceShoot;
import frc.robot.commands.c_intakeRetracted;
import frc.robot.commands.c_intakeExtended;
import frc.robot.subsystem.Pneumatics;

/** Add your docs here. */
public class Controller_Co_Driver {

    public static void configCoDriverController(XboxController controller, Pneumatics pneum) {

        // A
        new JoystickButton(controller, Button.kA.value)
                .whenHeld(new a_Default());

        // B
        new JoystickButton(controller, Button.kB.value)
                .whenHeld(new a_Default());

        // X
        new JoystickButton(controller, Button.kX.value)
                .toggleWhenPressed(new c_intakeExtended(pneum).withTimeout(0.5));

        // Y
        new JoystickButton(controller, Button.kY.value)
                .toggleWhenPressed(new c_intakeRetracted(pneum).withTimeout(0.5));

        // Start
        new JoystickButton(controller, Button.kStart.value)
                .whenPressed(new c_flipshootingSubsystemOn());

        // Back
        new JoystickButton(controller, Button.kBack.value)
                .whenHeld(new a_Default());

        // Left Bumper
        new JoystickButton(controller, Button.kLeftBumper.value)
                .whileHeld(new c_forceShoot());

        // Right Bumper
        new JoystickButton(controller, Button.kRightBumper.value)
                .whenHeld(new a_Default());

        // Left trigger as button
        //// Need these disabled so that they can be read in turret
        // new leftTriggerBool(controller)
        // .whileActiveContinuous(new a_Default());

        // Right trigger as button
        
        //// Need these disabled so that they can be read in turret
        // new rightTriggerBool(controller)
        // .whileActiveContinuous(new a_Default());

        // Top POV
        new POVButton(controller, 0)
                .whileHeld(new a_Default());

        // Right POV
        new POVButton(controller, 90)
                .whenHeld(new a_Default());

        // Bottom POV
        new POVButton(controller, 180)
                .whileHeld(new a_Default());

        // Left POV
        new POVButton(controller, 270)
                .whenHeld(new a_Default());

    }

}