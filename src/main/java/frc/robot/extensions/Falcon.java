// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Add your docs here. */
public class Falcon {

    public static class DefaultConfiguration {
        public NeutralMode neutralMode = NeutralMode.Brake;
        public double neutralDeadband = 0.04;
        public double openLoopRamp = 0.75;

    }

    private static DefaultConfiguration defaultConfig = new DefaultConfiguration();

    // create a CANTalon with the default (out of the box) configuration
    public static WPI_TalonFX createDefaultFalcon(int id) {
        return createFalcon(id, defaultConfig);
    }

    public static WPI_TalonFX createFalcon(int id, DefaultConfiguration config) {
        WPI_TalonFX talon = new WPI_TalonFX(id);

        talon.configFactoryDefault();
        talon.set(ControlMode.PercentOutput, 0.0);
        talon.setNeutralMode(config.neutralMode);
        talon.configOpenloopRamp(config.openLoopRamp);

        // What do these mean????
        // talon.configSupplyCurrentLimit(new
        // SupplyCurrentLimitConfiguration(config.ENABLE_SUPPLY_CURRENT_LIMIT, 20, 60,
        // .2), kTimeoutMs);
        // talon.configStatorCurrentLimit(new
        // StatorCurrentLimitConfiguration(config.ENABLE_STATOR_CURRENT_LIMIT, 20, 60,
        // .2), kTimeoutMs);

        // Return the configured motor object
        return talon;
    }
}
