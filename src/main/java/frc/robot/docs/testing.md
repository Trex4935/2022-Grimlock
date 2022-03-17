# Auto #

Current Auto Open PR [#46](https://github.com/Trex4935/Grimlock/pull/46), to merge after tested.

* Test the default auto for a baseline: 
    * _Back Up Straight Auto_
    * Steps:
        1.  Be sure all auto code is commented.

        2. uncomment theses line:

            **Declaration**
            > c_driveStraightAuto auto;

            **Assignment**
            > auto = new c_driveStraightAuto(drive);

            **Actual Use**
            default auto
            > return auto.withTimeout(1.5);

        3. Build & run Code
* Test autonomous mode #1 
    * _Back Up Straight, turn 45 and back up again Auto_
    * Steps: 
        1. Comment all previous _Back Up Straight Auto_ lines
        2. uncomment these line

            **Declaration**
            > c_driveStraightAngleAuto backingUpAuto;
            c_driveStraightAngleAuto backingUpAt45Auto;
            c_driveTurnAuto turnAuto;

            **Assignment**
            > backingUpAuto = new c_driveStraightAngleAuto(drive,0)
            backingUpAt45Auto = new c_driveStraightAngleAuto(drive,45)
            turnAuto = new c_driveTurnAuto(drive,45)

            **Actual Use**
            > //test auto#1\
            return backingUpAuto.withTimeout(1.5).andThen(turnAuto.andThen(backingUpAt45Auto.withTimeout(1.5))) ;

        3. Run & Build Code
        4. Upon Success, Ajust angle and time backing up  to fit Pick-uping needs
        5. Verify if shooting is done after picking up 3rd ball or at any time during motion. 



* Test Autonomous mode # 3 
    * _Back Up Straight, turn 45 and back up again  and explicitely shoot Auto_
    * Steps: 
        1. Keep uncommented lines of auto# 2, keep lines of auto#1 commented
        2. uncomment these line:

            **Declaration**
            > c_detectShootingReady rdyshot;

            **Assignment**
            
            > rdyshot = new c_detectShootingReady(intake, shooter, turret, coDriverController)

            **Actual Use**
            >return backingUpAuto.withTimeout(1.5).andThen(rdyshot.andThen(turnAuto.andThen(backingUpAt45Auto.withTimeout(1.5).andThen(rdyshot))));
            
            
        3. Run & Build Code
        4. Verify shoothing is done twice, 
            * after 1st ball pick-up, shoots 2 balls. 
            * after last ball pick up , shoots 1 balls

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

