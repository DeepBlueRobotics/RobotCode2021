/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021;

import frc.robot.lib.MotorErrors;
import frc.robot.lib.logging.Log;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.team199.robot2021.commands.HomeAbsolute;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  
  private RobotContainer robotContainer;
  private Timer timer;
  private File debugFile;
  private FileWriter writer;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    //SmartDashboard.putBoolean("Arcade Drive", true);
    //SmartDashboard.putBoolean("Characterized Drive", false);
    SmartDashboard.putBoolean("Field Oriented", false);
    robotContainer = new RobotContainer();
    Log.init();
    debugFile = new File("debug.txt");
    try {
      writer = new FileWriter(debugFile);
    } catch (IOException e) {
      e.printStackTrace();
      System.exit(1);
    }
    CommandScheduler.getInstance().schedule(new HomeAbsolute(robotContainer.drivetrain));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Log.logData();
    MotorErrors.printSparkMaxErrorMessages();
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    robotContainer.drivetrain.brake();
    robotContainer.getAutonomousCommand().schedule();
    Log.setDataLoggingDisabled(false);
    timer = new Timer();
    timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double time = timer.get();
    if (time < robotContainer.trajectory.getTotalTimeSeconds()) {
      try {
        writer.write("Trajectory pose at t = " + time + ": " + robotContainer.trajectory.sample(time).poseMeters + "\n");
        writer.write("Odometry pose at t = " + time + ": " + robotContainer.drivetrain.getOdometry().getPoseMeters() + "\n");
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    robotContainer.drivetrain.brake();
    Log.setDataLoggingDisabled(false);
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic(){

  }
  @Override
  public void disabledInit() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        robotContainer.drivetrain.coast();
      } catch(InterruptedException e) {}
    }).start();
    Log.flush();
    Log.setDataLoggingDisabled(true);
  }
}
