/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    static CANSparkMax leftMaster, rightMaster;
    static CANSparkMax leftSlave, rightSlave;
    static Encoder leftEnc, rightEnc;
    static AHRS ahrs;

    static {
        // Initialize motors on the left side of the drivetrain.
        leftMaster = createConfiguredSparkMax(8); // TODO: set correct motor ports
        leftSlave = createConfiguredSparkMax(9);

        // Initialize motors on the right side of the drivetrain.
        rightMaster = createConfiguredSparkMax(5);
        rightSlave = createConfiguredSparkMax(6);

        // Initialize encoders on right and left side
        leftEnc = new Encoder(new DigitalInput(0), new DigitalInput(1));
        rightEnc = new Encoder(new DigitalInput(2), new DigitalInput(3));
    
        ahrs = new AHRS(SPI.Port.kMXP);
    }

  private static WPI_TalonSRX createConfiguredTalon(int port) {
    WPI_TalonSRX tsrx = new WPI_TalonSRX(port);

    // Put all configurations for the talon motor controllers in here.
    // All values are from last year's code.
    catchError(tsrx.configNominalOutputForward(0, 10));
    catchError(tsrx.configNominalOutputReverse(0, 10));
    catchError(tsrx.configPeakOutputForward(1, 10));
    catchError(tsrx.configPeakOutputReverse(-1, 10));
    catchError(tsrx.configPeakCurrentLimit(0, 0));
    catchError(tsrx.configPeakCurrentDuration(0, 0));
    // 40 Amps is the amp limit of a CIM. lThe PDP has 40 amp circuit breakers,
    catchError(tsrx.configContinuousCurrentLimit(30, 0));
    tsrx.enableCurrentLimit(true);
    catchError(tsrx.configNeutralDeadband(0.001, 10));
    tsrx.setNeutralMode(NeutralMode.Brake);

    return tsrx;
  }

  private static WPI_VictorSPX createConfiguredVictor(int port) {
    WPI_VictorSPX vspx = new WPI_VictorSPX(port);

    // Put all configurations for the victor motor controllers in here.
    catchError(vspx.configNominalOutputForward(0, 10));
    catchError(vspx.configNominalOutputReverse(0, 10));
    catchError(vspx.configPeakOutputForward(1, 10));
    catchError(vspx.configPeakOutputReverse(-1, 10));
    catchError(vspx.configNeutralDeadband(0.001, 10));
    vspx.setNeutralMode(NeutralMode.Brake);

    return vspx;
  }

  private static CANSparkMax createConfiguredSparkMax(int port) {
    CANSparkMax spark = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushless);
    spark.restoreFactoryDefaults();
    spark.setIdleMode(IdleMode.kBrake);
    spark.enableVoltageCompensation(12);
    CANPIDController controller = spark.getPIDController();
    controller.setOutputRange(-1, 1);
    controller.setP(0);
    controller.setI(0);
    controller.setD(0);
    controller.setFF(0);
    return spark;
  }

  public static void catchError(ErrorCode ec) {
    if(ec != ErrorCode.OK) {
      System.out.println("Error configuring in RobotMap.java at line: " + new Throwable().getStackTrace()[1].getLineNumber());
      System.out.println(ec.toString());
    }  
  }
}
