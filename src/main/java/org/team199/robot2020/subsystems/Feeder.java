/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.playingwithfusion.TimeOfFlight;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  // TODO: find good values and then set to final
  private static double kBeltIntakeSpeed = .8;
  private static double kRollerIntakeSpeed = .3;
  private static double kBeltEjectSpeed = 1;
  private static double kRollerEjectSpeed = 1;

  private static double kInSensorMinDistance = 0;
  private static double kInSensorMaxDistance = 65; // 5 inches in millimeters //Old: Units.inchesToMeters(5) * 1000
  private static double kInSensorMinDistance2 = 30;
  private static double kInSensorMaxDistance2 = 50;
  private static double kOutSensorMinDistance = 65;
  private static double kOutSensorMaxDistance = 90;
  
  private final WPI_TalonSRX beltMotor = MotorControllerFactory.createTalon(Constants.Ports.kFeederBelt);
  private final WPI_TalonSRX ejectMotor = MotorControllerFactory.createTalon(Constants.Ports.kFeederEjector);
  private final TimeOfFlight inSensor = new TimeOfFlight(Constants.Ports.kFeederInSensor);
  private final TimeOfFlight outSensor = new TimeOfFlight(Constants.Ports.kFeederOutSensor);
  
  private double limitDistance = 12000;
  private double intakeDelay = 0.1;
  private int startPosition = 0;
  private boolean reachedShooter = false;
  private boolean intaking = false;
  private boolean intakeTimerStarted = false;
  private Timer intakeTimer = new Timer();

  /**
   * Takes and stores five balls from intake to give to shooter
   */
  public Feeder() {
    beltMotor.configPeakOutputForward(1D/3D, 10);
    beltMotor.configPeakOutputReverse(-1D/3D, 10);
    ejectMotor.configPeakOutputForward(1, 10);
    ejectMotor.configPeakOutputReverse(-1, 10);

    beltMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    SmartDashboard.putNumber("Feeder.kBeltIntakeSpeed", kBeltIntakeSpeed);
    SmartDashboard.putNumber("Feeder.kRollerIntakeSpeed", kRollerIntakeSpeed);
    SmartDashboard.putNumber("Feeder.kBeltEjectSpeed", kBeltEjectSpeed);
    SmartDashboard.putNumber("Feeder.kRollerEjectSpeed", kRollerEjectSpeed);
    SmartDashboard.putNumber("Feeder.kInSensorMinDistance", kInSensorMinDistance);
    SmartDashboard.putNumber("Feeder.kInSensorMaxDistance", kInSensorMaxDistance);
    SmartDashboard.putNumber("Feeder.kInSensorMinDistance2", kInSensorMinDistance2);
    SmartDashboard.putNumber("Feeder.kInSensorMaxDistance2", kInSensorMaxDistance2);
    SmartDashboard.putNumber("Feeder.kOutSensorMinDistance", kOutSensorMinDistance);
    SmartDashboard.putNumber("Feeder.kOutSensorMaxDistance", kOutSensorMaxDistance);
    SmartDashboard.putNumber("Feeder.limitDistance", limitDistance);
    SmartDashboard.putNumber("Feeder.intakeDelay", intakeDelay);
  }

  public void periodic() {
    if (inSensor.getRange() <= kInSensorMaxDistance && inSensor.getRange() >= kInSensorMinDistance) {
      if(!intakeTimerStarted) {
        intakeTimerStarted = true;
        intakeTimer.start();
      }
    } else {
      intakeTimerStarted = false;
      intakeTimer.stop();
      intakeTimer.reset();
      intaking = false;
    }
    SmartDashboard.putNumber("Feeder.intakeTimer", intakeTimer.get());

    if(intakeTimer.hasElapsed(intakeDelay)) {
      intaking = true;
    }

    if(intaking) {
      startPosition = beltMotor.getSelectedSensorPosition(0);
    }

    if (!reachedShooter) {
      reachedShooter = outSensor.getRange() <= kOutSensorMaxDistance;
    }

    kBeltIntakeSpeed = SmartDashboard.getNumber("Feeder.kBeltIntakeSpeed", kBeltIntakeSpeed);
    kRollerIntakeSpeed = SmartDashboard.getNumber("Feeder.kRollerIntakeSpeed", kRollerIntakeSpeed);
    kBeltEjectSpeed = SmartDashboard.getNumber("Feeder.kBeltEjectSpeed", kBeltEjectSpeed);
    kRollerEjectSpeed = SmartDashboard.getNumber("Feeder.kRollerEjectSpeed", kRollerEjectSpeed);
    kInSensorMinDistance = SmartDashboard.getNumber("Feeder.kInSensorMinDistance", kInSensorMinDistance);
    kInSensorMaxDistance = SmartDashboard.getNumber("Feeder.kInSensorMaxDistance", kInSensorMaxDistance);
    kInSensorMinDistance2 = SmartDashboard.getNumber("Feeder.kInSensorMinDistance2", kInSensorMinDistance2);
    kInSensorMaxDistance2 = SmartDashboard.getNumber("Feeder.kInSensorMaxDistance2", kInSensorMaxDistance2);
    kOutSensorMinDistance = SmartDashboard.getNumber("Feeder.kOutSensorMinDistance", kOutSensorMinDistance);
    kOutSensorMaxDistance = SmartDashboard.getNumber("Feeder.kOutSensorMaxDistance", kOutSensorMaxDistance);

    limitDistance = SmartDashboard.getNumber("Feeder.limitDistance", limitDistance);
    intakeDelay = SmartDashboard.getNumber("Feeder.intakeDelay", intakeDelay);

    SmartDashboard.putNumber("Feeder.currentInSensorDistance", inSensor.getRange());
    SmartDashboard.putNumber("Feeder.currentOutSensorDistance", outSensor.getRange());
    SmartDashboard.putNumber("Feeder.beltDistance", beltMotor.getSelectedSensorPosition(0));
    SmartDashboard.putBoolean("Feeder.reachedShooter", reachedShooter);
  }

  public void runForward() {
    beltMotor.set(kBeltIntakeSpeed);
    ejectMotor.set(kRollerIntakeSpeed);
  }

  public void runBackward() {
    beltMotor.set(-kBeltIntakeSpeed);
    ejectMotor.set(-kRollerIntakeSpeed);
    reset();
  }

  public void stop() {
    beltMotor.set(0);
    ejectMotor.set(0);
  }

  public void eject() {
    ejectMotor.set(kRollerEjectSpeed);
    beltMotor.set(kBeltEjectSpeed);
    reset();
  }

  public void reset() {
    reachedShooter = false;
  }

  public double getCellPosition() {
    return beltMotor.getSelectedSensorPosition(0) - startPosition;
  }

  public boolean isCellEntering() {
    return beltMotor.getSelectedSensorPosition(0) - startPosition < limitDistance;
  }

  public boolean isIntakeCellEntering() {
    return inSensor.getRange() <= kInSensorMaxDistance;
  }

  public boolean isCellAtShooter() {
    return /*reachedShooter &&*/ outSensor.getRange() <= kOutSensorMinDistance;
  }

  public boolean has5Intake() {
    double dist = inSensor.getRange();
    SmartDashboard.putBoolean("cellAtShooter", isCellAtShooter());
    return isCellAtShooter() && dist >= kInSensorMinDistance2 && dist <= kInSensorMaxDistance2;
  }

  public TimeOfFlight getShooterDistanceSensor() {
    return outSensor;
  }
}
