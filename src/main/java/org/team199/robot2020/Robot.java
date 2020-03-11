/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020;

import org.team199.lib.MotorErrors;
import org.team199.lib.logging.Log;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
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
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putBoolean("Arcade Drive", true);
    SmartDashboard.putBoolean("Characterized Drive", false);
    robotContainer = new RobotContainer();
    Log.init();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Log.logData();
    MotorErrors.printSparkMaxErrorMessages();
    if(!robotContainer.isDSConnected() && DriverStation.getInstance().isDSAttached()) {
      robotContainer.connectDS();
    }
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    robotContainer.getDrivetrain().toggleBreakMode();
    robotContainer.getAutonomousCommand().schedule();
    Log.setDataLoggingDisabled(false);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    robotContainer.getDrivetrain().toggleBreakMode();
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
        robotContainer.getDrivetrain().toggleBreakMode();
      } catch(InterruptedException e) {}
    }).start();
    Log.flush();
    Log.setDataLoggingDisabled(true);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1.0);
  }
}
