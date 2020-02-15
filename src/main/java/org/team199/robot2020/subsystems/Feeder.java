/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.playingwithfusion.TimeOfFlight;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  // TODO: find good values and then set to final
  private static double kBeltIntakeSpeed = .8;
  private static double kRollerIntakeSpeed = .8;
  private static double kBeltEjectSpeed = 1;

  private static double kRollerEjectSpeed = 1;
  private static double kInSensorDistance = Units.inchesToMeters(5) * 1000; // 5 inches in millimeters
  private static double kOutSensorDistance = Units.inchesToMeters(2.5) * 1000; // 5 inches in millimeters
  private static double kIntakeGapTime = .1;

  private final WPI_TalonSRX beltMotor = MotorControllerFactory.createTalon(Constants.Drive.kFeederBelt);
  private final WPI_TalonSRX ejectMotor = MotorControllerFactory.createTalon(Constants.Drive.kFeederEjector);
  private final TimeOfFlight inSensor = new TimeOfFlight(Constants.Drive.kFeederInSensor);
  private final TimeOfFlight outSensor = new TimeOfFlight(Constants.Drive.kFeederOutSensor);

  private final Timer timer = new Timer();

  /**
   * Takes and stores five balls from intake to give to shooter
   */
  public Feeder() {
    SmartDashboard.putNumber("Feeder.kBeltIntakeSpeed", kBeltIntakeSpeed);
    SmartDashboard.putNumber("Feeder.kRollerIntakeSpeed", kRollerIntakeSpeed);
    SmartDashboard.putNumber("Feeder.kBeltEjectSpeed", kBeltEjectSpeed);
    SmartDashboard.putNumber("Feeder.kRollerEjectSpeed", kRollerEjectSpeed);
    SmartDashboard.putNumber("Feeder.kInSensorDistance", kInSensorDistance);
    SmartDashboard.putNumber("Feeder.kOutSensorDistance", kOutSensorDistance);
    SmartDashboard.putNumber("Feeder.kIntakeGapTime", kIntakeGapTime);
  }

  public void periodic() {
    if (inSensor.getRange() < kInSensorDistance) {
      timer.reset();
      timer.start();
    }

    kBeltIntakeSpeed = SmartDashboard.getNumber("Feeder.kBeltIntakeSpeed", kBeltIntakeSpeed);
    kRollerIntakeSpeed = SmartDashboard.getNumber("Feeder.kRollerIntakeSpeed", kRollerIntakeSpeed);
    kBeltEjectSpeed = SmartDashboard.getNumber("Feeder.kBeltEjectSpeed", kBeltEjectSpeed);
    kRollerEjectSpeed = SmartDashboard.getNumber("Feeder.kRollerEjectSpeed", kRollerEjectSpeed);
    kInSensorDistance = SmartDashboard.getNumber("Feeder.kInSensorDistance", kInSensorDistance);
    kOutSensorDistance = SmartDashboard.getNumber("Feeder.kOutSensorDistance", kOutSensorDistance);
    kIntakeGapTime = SmartDashboard.getNumber("Feeder.kIntakeGapTime", kIntakeGapTime);
  }

  public void runForward() {
    beltMotor.set(kBeltIntakeSpeed);
    ejectMotor.set(kRollerIntakeSpeed);
  }

  public void runBackward() {
    beltMotor.set(-kBeltIntakeSpeed);
    ejectMotor.set(-kRollerIntakeSpeed);
  }

  public void stop() {
    beltMotor.set(0);
    ejectMotor.set(0);
  }

  public void eject() {
    ejectMotor.set(kRollerEjectSpeed);
    beltMotor.set(kBeltEjectSpeed);
  }

  public boolean isCellEntering() {
    return timer.get() < kIntakeGapTime;
  }

  public boolean isCellAtShooter() {
    return outSensor.getRange() < kOutSensorDistance;
  }
}
