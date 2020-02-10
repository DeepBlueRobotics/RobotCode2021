/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.playingwithfusion.TimeOfFlight;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  // TODO: find good values and then set to final
  private static double kBeltIntakeSpeed = .8;
  private static double kBeltEjectSpeed = 1;
  private static double kRollerEjectSpeed = 1;
  private static double kIndexerDistance = 5 * 25.4; // 5 inches in millimeters

  private final WPI_VictorSPX beltMotor = MotorControllerFactory.createVictor(Constants.Drive.kFeederBelt);
  private final WPI_VictorSPX ejectMotor = MotorControllerFactory.createVictor(Constants.Drive.kFeederEjector);
  private final TimeOfFlight inSensor = new TimeOfFlight(Constants.Drive.kFeederInSensor);
  private final TimeOfFlight outSensor = new TimeOfFlight(Constants.Drive.kFeederOutSensor);

  /**
   * Takes and stores five balls from intake to give to shooter
   */
  public Feeder() {
    SmartDashboard.putNumber("Feeder.kBeltIntakeSpeed", kBeltIntakeSpeed);
    SmartDashboard.putNumber("Feeder.kBeltEjectSpeed", kBeltEjectSpeed);
    SmartDashboard.putNumber("Feeder.kRollerEjectSpeed", kRollerEjectSpeed);
    SmartDashboard.putNumber("Feeder.kIndexerDistance", kIndexerDistance);
  }

  public void periodic() {
    kBeltIntakeSpeed = SmartDashboard.getNumber("Feeder.kBeltIntakeSpeed", kBeltIntakeSpeed);
    kBeltEjectSpeed = SmartDashboard.getNumber("Feeder.kBeltEjectSpeed", kBeltEjectSpeed);
    kRollerEjectSpeed = SmartDashboard.getNumber("Feeder.kRollerEjectSpeed", kRollerEjectSpeed);
    kIndexerDistance = SmartDashboard.getNumber("Feeder.kIndexerDistance", kIndexerDistance);
  }

  public void runForward() {
    beltMotor.set(kBeltIntakeSpeed);
  }

  public void runBackward() {
    beltMotor.set(-kBeltIntakeSpeed);
  }

  public void stop() {
    beltMotor.set(0);
  }

  public void eject() {
    ejectMotor.set(kRollerEjectSpeed);
    beltMotor.set(kBeltEjectSpeed);
  }

  public boolean isBallEntering() {
    return inSensor.getRange() < kIndexerDistance;
  }
}
