// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Add your docs here. */
public class Falcon {

    private static final int kTimeout = 20;

    public static class DefaultConfiguration {

        // Set Motor Brake mode
        public NeutralMode neutralMode = NeutralMode.Brake;

        // Input dead band
        public double neutralDeadband = 0.04;

        // Motor ramp when using open loop
        public double openLoopRamp = 0.75;

        // Set motor limits
        //// normal output forward and reverse = 0% ... i.e. stopped
        public double nominalOutputForward = 0;
        public double nominalOutputReverse = 0;

        //// Max output forward and reverse = 100%
        public double peakOutputForward = 1;
        public double peakOutputReverse = -1;

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

        talon.configNominalOutputForward(config.nominalOutputForward);
        talon.configNominalOutputReverse(config.nominalOutputReverse);

        talon.configPeakOutputForward(config.peakOutputForward);
        talon.configPeakOutputReverse(config.peakOutputReverse);

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

    public static WPI_TalonFX configurePID(WPI_TalonFX motorObject, double kP, double kI, double kD, double kF) {

        // PID configs
        // setting up the pid
        motorObject.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeout);
        // Set kP(proportional); kI(Integral); kD(differential); kF(FeedForward)
        motorObject.config_kP(0, kP, kTimeout);
        motorObject.config_kI(0, kI, kTimeout);
        motorObject.config_kD(0, kD, kTimeout);
        motorObject.config_kF(0, kF, kTimeout);

        motorObject.config_IntegralZone(0, 0, kTimeout);

        return motorObject;
    }
}
