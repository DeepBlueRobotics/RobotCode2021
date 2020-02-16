/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.lib;

import java.util.HashMap;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * Add your docs here.
 */
public class MotorControllerFactory {
  private static final HashMap<CANSparkMax, Short> flags = new HashMap<>();
  private static final HashMap<CANSparkMax, Short> stickyFlags = new HashMap<>();
  public static WPI_VictorSPX createVictor(int port) {
    WPI_VictorSPX victor = new WPI_VictorSPX(port);

    ErrorCode[] errors = new ErrorCode[5];
    // Put all configurations for the victor motor controllers in here.
    errors[0] = victor.configNominalOutputForward(0, 10);
    errors[1] = victor.configNominalOutputReverse(0, 10);
    errors[2] = victor.configPeakOutputForward(1, 10);
    errors[3] = victor.configPeakOutputReverse(-1, 10);
    errors[4] = victor.configNeutralDeadband(0.001, 10);
    victor.setNeutralMode(NeutralMode.Brake);
    parseErrors("VictorSPX", port, errors, ErrorCode.OK);

    return victor;
  }

  public static WPI_TalonSRX createTalon(int id) {
    WPI_TalonSRX talon = new WPI_TalonSRX(id);

    ErrorCode[] errors = new ErrorCode[8];
    // Put all configurations for the talon motor controllers in here.
    // All values are from last year's code.
    errors[0] = talon.configNominalOutputForward(0, 10);
    errors[1] = talon.configNominalOutputReverse(0, 10);
    errors[2] = talon.configPeakOutputForward(1, 10);
    errors[3] = talon.configPeakOutputReverse(-1, 10);
    errors[4] = talon.configPeakCurrentLimit(0, 0);
    errors[5] = talon.configPeakCurrentDuration(0, 0);
    // 40 Amps is the amp limit of a CIM. lThe PDP has 40 amp circuit breakers,
    errors[6] = talon.configContinuousCurrentLimit(30, 0);
    talon.enableCurrentLimit(true);
    errors[7] = talon.configNeutralDeadband(0.001, 10);
    talon.setNeutralMode(NeutralMode.Brake);
    parseErrors("TalonSRX", id, errors, ErrorCode.OK);

    return talon;
  }

  //checks for spark max errors
  public static void checkSparkMaxErrors(CANSparkMax spark) {     
    //Purposely obivously impersonal to differentiate from actual computer generated errors
    short faults = spark.getFaults();
    short stickyFaults = spark.getStickyFaults();
    short prevFaults = flags.containsKey(spark)?flags.get(spark) : 0;
    short prevStickyFaults = stickyFlags.containsKey(spark)?stickyFlags.get(spark) : 0;

    if (spark.getFaults() != 0 && prevFaults != faults) {
      System.err.println("Whoops, big oopsie : fault error with spark max id : " + spark.getDeviceId() + ", ooF!");
    }
    if (spark.getStickyFaults() != 0 && prevStickyFaults != stickyFaults) {
      System.err.println("Bruh, you did an Error : sticky fault error with spark max id : " + spark.getDeviceId() + ", Ouch!");
    }
    spark.clearFaults();
    flags.put(spark, faults);
    stickyFlags.put(spark, stickyFaults);
  }

  public static void printMessages() {
    flags.keySet().forEach((spark) -> checkSparkMaxErrors(spark));
  }

  public static CANSparkMax createSparkMax(int id) {
    CANSparkMax spark = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANError[] errors = new CANError[9];
    errors[0] = spark.restoreFactoryDefaults();
    errors[1] = spark.setIdleMode(IdleMode.kBrake);
    errors[2] = spark.enableVoltageCompensation(12);
    errors[3] = spark.setSmartCurrentLimit(50);

    checkSparkMaxErrors(spark);

    CANPIDController controller = spark.getPIDController();
    errors[4] = controller.setOutputRange(-1, 1);
    errors[5] = controller.setP(0);
    errors[6] = controller.setI(0);
    errors[7] = controller.setD(0);
    errors[8] = controller.setFF(0);

    parseErrors("SparkMax", id, errors, CANError.kOk);

    return spark;
  }

  private static <T extends Enum<T>> void parseErrors(String motorType, int id, T[] errors, T ok) {
    for(T error: errors) {
      if(error == null || error == ok) {
        continue;
      }
      System.err.println("Error: " + error.name() + " occured while configuring: " + motorType + " on CAN port: " id);
    }
  }
}