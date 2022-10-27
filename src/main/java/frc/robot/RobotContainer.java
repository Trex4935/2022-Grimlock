// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.c_detectShootingReady;
import frc.robot.commands.c_driveWithController;
import frc.robot.commands.cg_1BallHighAuto;
import frc.robot.commands.cg_1BallLowAuto;
import frc.robot.commands.cg_2BallHighAuto;
import frc.robot.commands.cg_2BallLowAuto;
import frc.robot.subsystem.LeftClimber;
import frc.robot.subsystem.Drivetrain;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Pneumatics;
import frc.robot.subsystem.RightClimber;
import frc.robot.subsystem.Sensors;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Turret;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.extensions.Controller_Co_Driver;
import frc.robot.extensions.Controller_Driver;
import frc.robot.extensions.Controller_Test;

public class RobotContainer {

  ///////////////////////
  private boolean competition = true;

  // Declare Subsystems
  private final Sensors sensor = new Sensors();
  private final Drivetrain drive = new Drivetrain();
  private final Intake intake = new Intake();
  private final Turret turret = new Turret();
  private final Shooter shooter = new Shooter();
  private final LeftClimber lClimber = new LeftClimber();
  private final RightClimber rClimber = new RightClimber();
  private final Pneumatics pneum = new Pneumatics();

  // Controller
  private static XboxController controller = new XboxController(0);
  private static XboxController coDriverController = new XboxController(1);

  // SendableChooser to pick automomous runs
  SendableChooser<Command> AutoRun_Picker = new SendableChooser<>();

  // Auto configuration
  private cg_1BallHighAuto auto1;
  private cg_1BallLowAuto auto2;
  private cg_2BallHighAuto auto3;
  private cg_2BallLowAuto auto4;

  public RobotContainer() {

    auto1 = new cg_1BallHighAuto(drive, pneum);
    auto2 = new cg_1BallLowAuto(drive, pneum);
    auto3 = new cg_2BallHighAuto(drive, pneum);
    auto4 = new cg_2BallLowAuto(drive, pneum);

    // Autonomous Chooser
    AutoRun_Picker.setDefaultOption("1 Ball High Auto Run", auto1);
    AutoRun_Picker.addOption("1 Ball Low Auto Run", auto2);
    AutoRun_Picker.addOption("2 Ball High Auto Run", auto3);
    AutoRun_Picker.addOption("2 Ball Low Auto Run", auto4);
    SmartDashboard.putData(AutoRun_Picker);

    if (competition) {
      competitionConfiguration();
    } else {
      testingConfiguration();
    }

  }

  private void testingConfiguration() {
    // Setup default drive controls
    drive.setDefaultCommand(new c_driveWithController(drive, controller,
        coDriverController));
    shooter.setDefaultCommand(new c_detectShootingReady(intake, shooter, turret,
        coDriverController));

    // Setup controller
    // Controller_Test.configTestController(controller, climber);
  }

  private void competitionConfiguration() {
    // Setup default drive controls
    drive.setDefaultCommand(new c_driveWithController(drive, controller,
        coDriverController));
    shooter.setDefaultCommand(new c_detectShootingReady(intake, shooter, turret,
        coDriverController));

    // Configure the CO_Driver Controller
    Controller_Co_Driver.configCoDriverController(coDriverController, pneum);

    // Configure the Driver Controller
    Controller_Driver.configDriverController(controller, lClimber, rClimber);

  }

  public Command getAutonomousCommand() {

    // Poll the SmartDashboard for the Autonomus Run Selection
    return AutoRun_Picker.getSelected();

    // releaseIntake.andThen(
    // return auto;
    // return auto.withTimeout(.1).andThen(new
    // WaitCommand(1).andThen(auto).withTimeout(0.4));
  }

}