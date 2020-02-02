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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private static final double kBeltSpeed = .8;
  private static final double kEjectSpeed = 1;
  private static final double kIndexerDistance = 127; // 5 inches in millimeters

  private final WPI_TalonSRX beltMotor = MotorControllerFactory.createTalon(Constants.Drive.kFeederBelt);
  private final WPI_TalonSRX ejectMotor = MotorControllerFactory.createTalon(Constants.Drive.kFeederEjector);
  private final TimeOfFlight inSensor = new TimeOfFlight(Constants.Drive.kFeederInSensor);
  private final TimeOfFlight outSensor = new TimeOfFlight(Constants.Drive.kFeederOutSensor);

  /**
   * Creates a new Feeder.
   */
  public Feeder() {
  }

  public void runForward() {
    beltMotor.set(kBeltSpeed);
  }

  public void runBackward() {
    beltMotor.set(-kBeltSpeed);
  }

  public void stop() {
    beltMotor.set(0);
  }

  public void eject() {
    ejectMotor.set(kEjectSpeed);
  }

  public boolean isBallEntering() {
    return inSensor.getRange() < kIndexerDistance;
  }
}
