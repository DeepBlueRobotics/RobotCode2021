/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;

import frc.robot.lib.MotorControllerFactory;
import org.team199.robot2021.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final CANSparkMax funnelMotor = MotorControllerFactory.createSparkMax(Constants.Drive.kFeederFunnel);
  private final CANSparkMax hopperMotor = MotorControllerFactory.createSparkMax(Constants.Drive.kFeederHopper);

  private final TimeOfFlight inSensor = new TimeOfFlight(Constants.Drive.kFeederInSensor);
  private final TimeOfFlight outSensor = new TimeOfFlight(Constants.Drive.kFeederOutSensor);

  private static double kFunnelSpeed = 1;
  private static double kHopperIntakeSpeed = .5;
  private static double kHopperShootSpeed = 1;
  private static double kInSensorDistance = Units.inchesToMeters(4)/1000;
  private static double kOutSensorDistance = Units.inchesToMeters(6)/1000;
  
  public Feeder() {
    SmartDashboard.putNumber("Feeder.kFunnelSpeed", kFunnelSpeed);
    SmartDashboard.putNumber("Feeder.kHopperIntakeSpeed", kHopperIntakeSpeed);
    SmartDashboard.putNumber("Feeder.kHopperShootSpeed", kHopperShootSpeed);
    SmartDashboard.putNumber("Feeder.kInSensorDistance", kInSensorDistance);
    SmartDashboard.putNumber("Feeder.kOutSensorDistance", kOutSensorDistance);
  }

  @Override
  public void periodic() {
    kFunnelSpeed = SmartDashboard.getNumber("Feeder.kFunnelSpeed", kFunnelSpeed);
    kHopperIntakeSpeed = SmartDashboard.getNumber("Feeder.kHopperIntakeSpeed", kHopperIntakeSpeed);
    kHopperShootSpeed = SmartDashboard.getNumber("Feeder.kHopperShootSpeed", kHopperShootSpeed);
    kInSensorDistance = SmartDashboard.getNumber("Feeder.kInSensorDistance", kInSensorDistance);
    kOutSensorDistance = SmartDashboard.getNumber("Feeder.kOutSensorDistance", kOutSensorDistance);
  }

  public boolean isCellEntering() {
    return inSensor.getRange() < kInSensorDistance;
  }

  public boolean isCellAtShooter() {
    return outSensor.getRange() < kOutSensorDistance;
  }

  public void intake() {
    funnelMotor.set(kFunnelSpeed);
    hopperMotor.set(kHopperIntakeSpeed);
  }

  public void outtake() {
    funnelMotor.set(-kFunnelSpeed);
    hopperMotor.set(-kHopperIntakeSpeed);
  }

  public void shoot() {
    funnelMotor.set(kFunnelSpeed);
    hopperMotor.set(kHopperShootSpeed);
  }

  public void stop() {
    funnelMotor.set(0);
    hopperMotor.set(0);
  }

  public TimeOfFlight getShooterDistanceSensor() {
    return outSensor;
  }
}