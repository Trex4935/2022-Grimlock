package frc.robot.extensions;
import frc.robot.extensions.Helper;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;


public class ShuffleboardInit {

// Settings
public ShuffleboardTab matchSettings = Shuffleboard.getTab("Settings");

public NetworkTableEntry shooterAdjust = 
    matchSettings.addPersistent("Shooter Speed", 0.0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -500, "max", 500)) // slider range of -500 to 500
    .withSize(2, 1) // make the widget 2x1
    .withPosition(0, 0) // place it in the top-left corner
    .getEntry();
    
public NetworkTableEntry shootHighLow = 
    matchSettings.addBoolean("Shoot High or Low", BooleanSupplier getShootingLow())
    .withWidget(BuiltInWidgets.kToggleButton)
    .withSize(2, 1) // make the widget 2x1
    .withPosition(0, 2) // place it in the top-left corner
    .getEntry();


// Debug Elements
public ShuffleboardTab robotDebug = Shuffleboard.getTab("Robot-Debug");

private NetworkTableEntry distanceEntry =
       robotDebug.add("Distance to target", 0)
          .getEntry();


}