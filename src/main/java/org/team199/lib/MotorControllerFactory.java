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
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * Add your docs here.
 */
public class MotorControllerFactory {

  public static WPI_VictorSPX createVictor(int port) {
    WPI_VictorSPX victor = new WPI_VictorSPX(port);

    // Put all configurations for the victor motor controllers in here.
    victor.configNominalOutputForward(0, 10);
    victor.configNominalOutputReverse(0, 10);
    victor.configPeakOutputForward(1, 10);
    victor.configPeakOutputReverse(-1, 10);
    victor.configNeutralDeadband(0.001, 10);
    victor.setNeutralMode(NeutralMode.Brake);

    return victor;
  }

  public static WPI_TalonSRX createTalon(int id) {
    WPI_TalonSRX talon = new WPI_TalonSRX(id);

    // Put all configurations for the talon motor controllers in here.
    // All values are from last year's code.
    talon.configNominalOutputForward(0, 10);
    talon.configNominalOutputReverse(0, 10);
    talon.configPeakOutputForward(1, 10);
    talon.configPeakOutputReverse(-1, 10);
    talon.configPeakCurrentLimit(0, 0);
    talon.configPeakCurrentDuration(0, 0);
    // 40 Amps is the amp limit of a CIM. lThe PDP has 40 amp circuit breakers,
    talon.configContinuousCurrentLimit(30, 0);
    talon.enableCurrentLimit(true);
    talon.configNeutralDeadband(0.001, 10);
    talon.setNeutralMode(NeutralMode.Brake);

    return talon;
  }

  public static CANSparkMax createSparkMax(int id) {
    CANSparkMax spark = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
    spark.restoreFactoryDefaults();
    spark.setIdleMode(IdleMode.kBrake);
    spark.enableVoltageCompensation(12);
    spark.setSmartCurrentLimit(50);

    CANPIDController controller = spark.getPIDController();
    controller.setOutputRange(-1, 1);
    controller.setP(0);
    controller.setI(0);
    controller.setD(0);
    controller.setFF(0);

    return spark;
  }
}