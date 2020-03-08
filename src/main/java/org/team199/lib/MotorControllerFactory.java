/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.lib;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * Creates motor controllers
 */
public class MotorControllerFactory {
  /**
   * Creates and configures {@link WPI_VictorSPX}
   * @param id The can port assigned to the {@link WPI_VictorSPX}
   * @return A {@link WPI_VictorSPX} configured to run on the specified port
   */
  public static WPI_VictorSPX createVictor(int id) {
    WPI_VictorSPX victor = new WPI_VictorSPX(id);

    // Put all configurations for the victor motor controllers in here.
    MotorErrors.reportError(victor.configNominalOutputForward(0, 10));
    MotorErrors.reportError(victor.configNominalOutputReverse(0, 10));
    MotorErrors.reportError(victor.configPeakOutputForward(1, 10));
    MotorErrors.reportError(victor.configPeakOutputReverse(-1, 10));
    MotorErrors.reportError(victor.configNeutralDeadband(0.001, 10));
    victor.setNeutralMode(NeutralMode.Brake);

    return victor;
  }

  /**
   * Creates and configures {@link WPI_TalonSRX}
   * @param port The can port assigned to the {@link WPI_TalonSRX}
   * @return A {@link WPI_TalonSRX} configured to run on the specified port
   */
  public static WPI_TalonSRX createTalon(int id) {
    WPI_TalonSRX talon = new WPI_TalonSRX(id);

    // Put all configurations for the talon motor controllers in here.
    // All values are from last year's code.
    MotorErrors.reportError(talon.configNominalOutputForward(0, 10));
    MotorErrors.reportError(talon.configNominalOutputReverse(0, 10));
    MotorErrors.reportError(talon.configPeakOutputForward(1, 10));
    MotorErrors.reportError(talon.configPeakOutputReverse(-1, 10));
    MotorErrors.reportError(talon.configPeakCurrentLimit(0, 0));
    MotorErrors.reportError(talon.configPeakCurrentDuration(0, 0));
    // 40 Amps is the amp limit of a CIM. lThe PDP has 40 amp circuit breakers,
    MotorErrors.reportError(talon.configContinuousCurrentLimit(30, 0));
    talon.enableCurrentLimit(true);
    MotorErrors.reportError(talon.configNeutralDeadband(0.001, 10));
    talon.setNeutralMode(NeutralMode.Brake);

    return talon;
  }

  //checks for spark max errors

  /**
   * Creates and configures {@link CANSparkMax} as well as checking if it is disconnected
   * @param port The can port assigned to the {@link CANSparkMax}
   * @return A {@link CANSparkMax} configured to run on the specified port
   */
  public static CANSparkMax createSparkMax(int id) {
    CANSparkMax spark = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
    if(spark.getFirmwareVersion() == 0) {
      spark.close();
      System.err.println("SparkMax on port: " + id + " is not connected!");
      return MotorErrors.createDummySparkMax();
    }
    MotorErrors.reportError(spark.restoreFactoryDefaults());
    MotorErrors.reportError(spark.follow(ExternalFollower.kFollowerDisabled, 0));
    MotorErrors.reportError(spark.setIdleMode(IdleMode.kBrake));
    MotorErrors.reportError(spark.enableVoltageCompensation(12));
    MotorErrors.reportError(spark.setSmartCurrentLimit(50));

    MotorErrors.checkSparkMaxErrors(spark);

    CANPIDController controller = spark.getPIDController();
    MotorErrors.reportError(controller.setOutputRange(-1, 1));
    MotorErrors.reportError(controller.setP(0));
    MotorErrors.reportError(controller.setI(0));
    MotorErrors.reportError(controller.setD(0));
    MotorErrors.reportError(controller.setFF(0));

    return spark;
  }
}