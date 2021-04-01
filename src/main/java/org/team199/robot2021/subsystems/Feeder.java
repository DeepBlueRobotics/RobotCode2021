/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.playingwithfusion.TimeOfFlight;

import frc.robot.lib.MotorControllerFactory;

import org.mockito.Mockito;
import org.mockito.internal.stubbing.defaultanswers.ReturnsSmartNulls;
import org.team199.robot2021.Constants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  // TODO: find good values and then set to final
  private static double kBeltIntakeSpeed = .8;
  private static double kRollerIntakeSpeed = .1;
  private static double kBeltEjectSpeed = 1;
  private static double kRollerEjectSpeed = 1;

  private static double kInSensorMinDistance = 0;
  private static double kInSensorMaxDistance = 65; // 5 inches in millimeters //Old: Units.inchesToMeters(5) * 1000
  private static double kOutSensorMinDistance = 32;
  private static double kOutSensorMaxDistance = 45;
  
  private final WPI_TalonSRX beltMotor = MotorControllerFactory.createTalon(Constants.DrivePorts.kFeederBelt);
  private final WPI_TalonSRX ejectMotor = MotorControllerFactory.createTalon(Constants.DrivePorts.kFeederEjector);
  private final TimeOfFlight inSensor = createTimeOfFlight(Constants.DrivePorts.kFeederInSensor);
  private final TimeOfFlight outSensor = createTimeOfFlight(Constants.DrivePorts.kFeederOutSensor);
  
  private double limitDistance = 7000;
  private double startPosition = 0;
  private boolean reachedShooter = false;

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
    SmartDashboard.putNumber("Feeder.kOutSensorMinDistance", kOutSensorMinDistance);
    SmartDashboard.putNumber("Feeder.kOutSensorMaxDistance", kOutSensorMaxDistance);
    SmartDashboard.putNumber("Feeder.limitDistance", limitDistance);
  }

  public void periodic() {
    if (inSensor.getRange() <= kInSensorMaxDistance && inSensor.getRange() >= kInSensorMinDistance) {
      //startPosition = beltMotor.getSelectedSensorPosition(0);
    }

    if (!reachedShooter) {
      reachedShooter = outSensor.getRange() <= kOutSensorMinDistance;
    }

    kBeltIntakeSpeed = SmartDashboard.getNumber("Feeder.kBeltIntakeSpeed", kBeltIntakeSpeed);
    kRollerIntakeSpeed = SmartDashboard.getNumber("Feeder.kRollerIntakeSpeed", kRollerIntakeSpeed);
    kBeltEjectSpeed = SmartDashboard.getNumber("Feeder.kBeltEjectSpeed", kBeltEjectSpeed);
    kRollerEjectSpeed = SmartDashboard.getNumber("Feeder.kRollerEjectSpeed", kRollerEjectSpeed);
    kInSensorMinDistance = SmartDashboard.getNumber("Feeder.kInSensorMinDistance", kInSensorMinDistance);
    kInSensorMaxDistance = SmartDashboard.getNumber("Feeder.kInSensorMaxDistance", kInSensorMaxDistance);
    kOutSensorMinDistance = SmartDashboard.getNumber("Feeder.kOutSensorMinDistance", kOutSensorMinDistance);
    kOutSensorMaxDistance = SmartDashboard.getNumber("Feeder.kOutSensorMaxDistance", kOutSensorMaxDistance);

    limitDistance = SmartDashboard.getNumber("Feeder.limitDistance", limitDistance);

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

  public boolean isCellEntering() {
    return beltMotor.getSelectedSensorPosition(0) - startPosition < limitDistance;
  }

  public boolean isCellAtShooter() {
    return reachedShooter && outSensor.getRange() >= kOutSensorMaxDistance;
  }

  public TimeOfFlight getShooterDistanceSensor() {
    return outSensor;
  }

  public static TimeOfFlight createTimeOfFlight(int port) {
    return RobotBase.isReal() ? new TimeOfFlight(port) : Mockito.mock(TimeOfFlight.class, new ReturnsSmartNulls());
  }

}
