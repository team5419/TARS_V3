// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autos.Auto;
import frc.robot.commands.arm.MoveToPos;
import frc.robot.commands.testing.AutoAlignPennTester;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private RobotContainer m_robotContainer;
  private SubsystemlessVision vision = new SubsystemlessVision();
  private static LinkedHashMap<String, Auto> autoMap = new LinkedHashMap<String, Auto>(); 
  private static LinkedHashMap<String, Auto> tempMap = new LinkedHashMap<String, Auto>();
  private static SendableChooser<Auto> chooser;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
  //  */
  // @Override
  // public void robotInit() {
  //   ctreConfigs = new CTREConfigs();
  //   m_robotContainer = new RobotContainer();

  //   chooser = new SendableChooser<Auto>();
  //   autoMap.putAll(configurePrebuiltAutos());
  //   for (HashMap.Entry<String, Auto> choice : autoMap.entrySet()) {
  //     String autoName = choice.getKey().split("[.]")[0];
  //     chooser.addOption(autoName, choice.getValue());
  //     System.out.println("Added " + autoName);
  //   }
  //   chooser.setDefaultOption("Do Nothing", new Auto(new InstantCommand(), m_robotContainer.s_Swerve));
  //   SmartDashboard.putData("Auto Chooser", chooser);
  // }

  // /**
  //  * This function is called every robot packet, no matter the mode. Use this for items like
  //  * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
  //  *
  //  * <p>This runs after the mode specific periodic functions, but before LiveWindow and
  //  * SmartDashboard integrated updating.
  //  */
  // @Override
  // public void robotPeriodic() {
  //   vision.updateEntries();
  //   // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
  //   // commands, running already-scheduled commands, removing finished or interrupted commands,
  //   // and running subsystem periodic() methods.  This must be called from the robot's periodic
  //   // block in order for anything in the Command-based framework to work.
  //   CommandScheduler.getInstance().run();
  // }

  // /** This function is called once each time the robot enters Disabled mode. */
  // @Override
  // public void disabledInit() {}

  // @Override
  // public void disabledPeriodic() {}

  // /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  // @Override
  // public void autonomousInit() {
  //   chooser.getSelected().Run();
  // }

  // /** This function is called periodically during autonomous. */
  // @Override
  // public void autonomousPeriodic() {}

  // @Override
  // public void teleopInit() {
  //   // This makes sure that the autonomous stops running when
  //   // teleop starts running. 
  //   chooser.getSelected().end();

  //   // Try to avoid arm snapping when enabled
  //   new MoveToPos(m_robotContainer.m_arm, m_robotContainer.m_arm.getBicepPosition(), m_robotContainer.m_arm.getWristPosition()); 
  // }

  // /** This function is called periodically during operator control. */
  // @Override
  // public void teleopPeriodic() {}

  // @Override
  // public void testInit() {
  //   // Cancels all running commands at the start of test mode.
  //   CommandScheduler.getInstance().cancelAll();
  // }

  // /** This function is called periodically during test mode. */
  // @Override
  // public void testPeriodic() {}

  // public void simulationPeriodic(){
  // }

  // private LinkedHashMap<String, Auto> configurePrebuiltAutos() {
  //   File pathplannerDir = new File(Filesystem.getDeployDirectory(),"pathplanner");
  //   tempMap.clear();
  //   for (File file : pathplannerDir.listFiles()) {
  //     String fileName = file.getName();
  //     if (!file.isDirectory()) {
  //       String[] autoName = fileName.split("[.]");
  //       tempMap.put(fileName, new Auto(m_robotContainer.s_Swerve, autoName[0]));
  //     }

  //   }
  //   return tempMap;
  // }
}
