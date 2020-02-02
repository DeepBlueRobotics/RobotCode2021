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
  private final WPI_TalonSRX beltMotor = MotorControllerFactory.createTalon(Constants.Feeder.kBeltMotor);
  private final WPI_TalonSRX ejectMotor = MotorControllerFactory.createTalon(Constants.Feeder.kEjectMotor);
  private final TimeOfFlight indexSensor = new TimeOfFlight(Constants.Feeder.kIndexSensor);

  /**
   * Creates a new Feeder.
   */
  public Feeder() {
    
  }

  public void runForward() {
    beltMotor.set(Constants.Feeder.kBeltSpeed);
  }

  public void runBackward() {
    beltMotor.set(-Constants.Feeder.kBeltSpeed);
  }

  public void stop() {
    beltMotor.set(0);
  }

  public void eject() {
    ejectMotor.set(Constants.Feeder.kEjectSpeed);
  }

  public boolean isBallEntering() {
    return indexSensor.getRange() < Constants.Feeder.kIndexerDistance; // 5 inches in millimeters
  }
}
