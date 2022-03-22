// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;
import frc.robot.Constants;

/** Add your docs here. */
public class Helper {

    // compare a value to a maximum and minimum and return if it is within that
    // range
    public static boolean RangeCompare(double maximum, double minimum, double value) {
        if (value > minimum && value <= maximum) {
            return true;
        } else {
            return false;
        }
    }

    // Flip pew pew to the other boolean value
    public static void flipPewPew() {
        Constants.pewpew = !(Constants.pewpew);
    }

}
