# Welcome to the codebase of our offseason bot, TARS

![Image of TARS in action](374372FC-40C1-4786-9B14-7AB8BF70C7F0.JPG)

This codebase was made based off the [6672 code](https://github.com/FusionCorps/2023-Ignition), and 364's BaseFalconSwerve lib, with heavy chages. 

## Code hilights


- [`team5419.subsystems.arm`](src/main/java/frc/robot/subsystems/arm)

  Contains the `OptimizedArm` class, providing the functionality of the arm, as well as the `GraphStator` class witch provides the logic behind the movement of the arm, 
  esuring it can move in paralell without hitting the chassis of the robot.

  For refrence in the `GraphStaor` class, the sectors are layed out like this:

  ```
  
  E |________| B

        A
    ________ F
  D |        | C
  
  ```
  
  Where A is at (0, 0), moving toward positive x is moving the bicep joint toward the front of the bot, and moving positive is moving the wrist clockwise.

- [`team5419.commands.arm`](src/main/java/frc/robot/commands/arm)

  Contains all the various commands for using the arm.

  The highlght is the `OptimizedMoveV3` command, witch is what we use with the `GraphStator` class to move the arm in paralell. All of the commands are basically wrappers that add extra cotrol of the `BicepToPos` and `WristToPos` commands.

  Most of them are not used, but were once used.

- [`team5419.commands.swerve`](src/main/java/frc/robot/commands/sewerve)

  Contains all the commands moving the robot in various ways.

  The main highlight is the `AutoAlign` command. Its a rudementary autoalign that is ment to be used with Limelight. It donsnt contain any rate limiting, and is only ment to be used for slight adjustment.

- [`team5419.autos`](src/main/java/frc/robot/autos)

  The code used for all of our autos.

  The main highlight is the `Auto.java` file. This file is used to build and execute all of our autos. Sadly, this whole file was rendered useless by the 2024 PathPlanner update, but it was super usefull while it lasted.

  
