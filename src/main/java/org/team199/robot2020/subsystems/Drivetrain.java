/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.subsystems;

import com.revrobotics.CANSparkMax;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax leftMaster = MotorControllerFactory.createSparkMax(Constants.Drive.LEFT_MOTOR_1);
  private final CANSparkMax leftSlave = MotorControllerFactory.createSparkMax(Constants.Drive.LEFT_MOTOR_2);
  private final CANSparkMax rightMaster = MotorControllerFactory.createSparkMax(Constants.Drive.RIGHT_MOTOR_1);
  private final CANSparkMax rightSlave = MotorControllerFactory.createSparkMax(Constants.Drive.RIGHT_MOTOR_2);

  private final AHRS gyro = new AHRS();

  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

  public Drivetrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    rightMaster.setInverted(true);
    rightSlave.setInverted(true);
  }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double left, double right) {
    diffDrive.tankDrive(left, right);
  }
}

