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
  private final WPI_TalonSRX beltMotor = MotorControllerFactory.createTalon(Constants.Feeder.BELT_MOTOR);
  private final WPI_TalonSRX ejectMotor = MotorControllerFactory.createTalon(Constants.Feeder.EJECT_MOTOR);
  private final TimeOfFlight indexSensor = new TimeOfFlight(Constants.Feeder.INDEX_SENSOR);

  /**
   * Creates a new Feeder.
   */
  public Feeder() {
    
  }

  public void runForward() {
    beltMotor.set(Constants.Feeder.BELT_SPEED);
  }

  public void runBackward() {
    beltMotor.set(-Constants.Feeder.BELT_SPEED);
  }

  public void stop() {
    beltMotor.set(0);
  }

  public void eject() {
    ejectMotor.set(Constants.Feeder.EJECT_SPEED);
  }

  public boolean isBallEntering() {
    return indexSensor.getRange() < Constants.Feeder.INDEXER_DISTANCE; // 5 inches in millimeters
  }
}
