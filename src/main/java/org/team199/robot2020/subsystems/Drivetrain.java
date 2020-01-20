/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax leftMaster = MotorControllerFactory.createSparkMax(Constants.Drive.LEFT_MOTOR_1);
  private final CANSparkMax leftSlave = MotorControllerFactory.createSparkMax(Constants.Drive.LEFT_MOTOR_2);
  private final CANSparkMax rightMaster = MotorControllerFactory.createSparkMax(Constants.Drive.RIGHT_MOTOR_1);
  private final CANSparkMax rightSlave = MotorControllerFactory.createSparkMax(Constants.Drive.RIGHT_MOTOR_2);

  private final Encoder leftEnc = new Encoder(Constants.Drive.LEFT_ENCODER[0], Constants.Drive.LEFT_ENCODER[1]);
  private final Encoder rightEnc = new Encoder(Constants.Drive.RIGHT_ENCODER[0], Constants.Drive.RIGHT_ENCODER[1]);

  private final AHRS gyro = new AHRS();

  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

  private DifferentialDriveOdometry odometry = null;
  private static final boolean isGyroReversed = false;

  public Drivetrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    rightMaster.setInverted(true);
    rightSlave.setInverted(true);
  }

  public void setOdometry(DifferentialDriveOdometry odometry) {
    this.odometry = odometry;
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (isGyroReversed ? -1.0 : 1.0);
  }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double left, double right) {
    diffDrive.tankDrive(left, right);
  }

  /**
   * Characterizes the left/right motor values.
   * 
   * @param left  from -1 to 1
   * @param right from -1 to 1
   */
  public void characterizedDrive(double left, double right) {
    final double actualLeftVel = leftEnc.getRate();
    final double actualRightVel = rightEnc.getRate();

    final double desiredLeftVel = Constants.Drivetrain.MAX_SPEED * left;
    final double desiredRightVel = Constants.Drivetrain.MAX_SPEED * right;

    double leftAccel = (desiredLeftVel - actualLeftVel) / Constants.Drivetrain.LOOP_TIME;
    double rightAccel = (desiredRightVel - actualRightVel) / Constants.Drivetrain.LOOP_TIME;
    leftAccel = Math.copySign(1.0, leftAccel) * Math.min(Math.abs(leftAccel), Constants.Drivetrain.MAX_ACCEL);
    rightAccel = Math.copySign(1.0, rightAccel) * Math.min(Math.abs(rightAccel), Constants.Drivetrain.MAX_ACCEL);

    double newLeft, newRight;
    if (left >= 0.0) {
          // Front-Left
          newLeft = Constants.Drivetrain.kVOLTS[0] + Constants.Drivetrain.kVELS[0] * desiredLeftVel + Constants.Drivetrain.kACCELS[0] * leftAccel; 
    } else {
          // Back-Left
          newLeft = -Constants.Drivetrain.kVOLTS[2] + Constants.Drivetrain.kVELS[2] * desiredLeftVel + Constants.Drivetrain.kACCELS[2] * leftAccel;
    }
    if (right >= 0.0) {
        // Front-Right
        newRight = Constants.Drivetrain.kVOLTS[1] + Constants.Drivetrain.kVELS[1] * desiredRightVel + Constants.Drivetrain.kACCELS[1] * rightAccel;
    } else {
          // Back-Right
          newRight = -Constants.Drivetrain.kVOLTS[3] + Constants.Drivetrain.kVELS[3] * desiredRightVel + Constants.Drivetrain.kACCELS[3] * rightAccel;
    }
    tankDrive(newLeft, newRight);
  }
}
