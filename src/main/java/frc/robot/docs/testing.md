# Drive Train #

* Disable all default commands except DRIVE

## Arcade Drive ##
* Left stick forward and back == moves robot forward and back 
    * Validate expected brake mode
* Right stick left and right == rotates robot left and right 
    * Validate brake mode

## H Drive ##
* Left stick left and right == moves robot side to side 
    * Validate brake mode

# Turrent #

* Enable all default commands
* Point the robot so that the turret is facing the target

* Enable Robot
    * Validate that Intake, Shooter, and Magazine turn on
* Rotate robot SLOWLY
    * Validate that the turret is tracking the target


# Shooter #

* Ensure that we are looking at a target otherwise this testing will NOT work

* Enable Robot
    * Validate that Intake turns on
    * Validate the Magazine turns on
* Set Team on Driver Station to RED
* Feed in Blue Ball
    * Validate ball is shot at LOW speed
* Feed in Red Ball
    * Validate ball is shot at HIGH speed
* Set team on Drive Station to BLUE
* Feed in Red Ball
    * Validate ball is shot at LOW speed
* Feed in Blue Ball
    * Validate ball is shot at HIGH Speed

* Feed in balls while rotating robot slowly
    * Validate that the turrent is tracking and balls are shooting correctly

# Singulation #

* Disable all default commands except SHOOTER

* Enable Robot
    * Validate that intake turns on
    * Validate that magazine turns on
* Set Team on Drive Station to RED
* RAPIDLY Feed in BLUE THEN RED Ball
    * Validate that intake STOPS once BLUE balls is in front to top sensor

# Climber #

* Disable all default commands

* Enable Robot
* Press A on controller
    * Validate the arms extend fully and rotate towards shooter
* Press B on controller
    * Validate that arms fully retract

