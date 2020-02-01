/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  public enum Side {
    LEFT, RIGHT;
  }

  private final CANSparkMax leftMaster = MotorControllerFactory.createSparkMax(Constants.Drive.LEFT_MOTOR_1);
  private final CANSparkMax leftSlave = MotorControllerFactory.createSparkMax(Constants.Drive.LEFT_MOTOR_2);
  private final CANSparkMax rightMaster = MotorControllerFactory.createSparkMax(Constants.Drive.RIGHT_MOTOR_1);
  private final CANSparkMax rightSlave = MotorControllerFactory.createSparkMax(Constants.Drive.RIGHT_MOTOR_2);

  private final CANEncoder leftEnc = leftMaster.getEncoder();
  private final CANEncoder rightEnc = rightMaster.getEncoder();

  private final AHRS gyro = new AHRS();

  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

  private final PIDController leftPIDController = new PIDController(Constants.Drivetrain.kPIDLEFT[0], Constants.Drivetrain.kPIDLEFT[1], Constants.Drivetrain.kPIDLEFT[2]);
  private final PIDController rightPIDController = new PIDController(Constants.Drivetrain.kPIDRIGHT[0], Constants.Drivetrain.kPIDRIGHT[1], Constants.Drivetrain.kPIDRIGHT[2]);
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TRACKWIDTH);
  
  private final SimpleMotorFeedforward forwardLeftFF = new SimpleMotorFeedforward(Constants.Drivetrain.kVOLTS[0], Constants.Drivetrain.kVELS[0] / Constants.METERS_PER_INCH);
  private final SimpleMotorFeedforward backwardLeftFF = new SimpleMotorFeedforward(Constants.Drivetrain.kVOLTS[2], Constants.Drivetrain.kVELS[2] / Constants.METERS_PER_INCH);
  private final SimpleMotorFeedforward forwardRightFF = new SimpleMotorFeedforward(Constants.Drivetrain.kVOLTS[1], Constants.Drivetrain.kVELS[1] / Constants.METERS_PER_INCH);
  private final SimpleMotorFeedforward backwardRightFF = new SimpleMotorFeedforward(Constants.Drivetrain.kVOLTS[3], Constants.Drivetrain.kVELS[3] / Constants.METERS_PER_INCH);
  
  private DifferentialDriveOdometry odometry = null;
  private static final boolean isGyroReversed = false;

  public Drivetrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    //rightSlave.setInverted(true);

    double conversion = (Math.PI * 5.0) / 6.8;
    leftEnc.setPositionConversionFactor(conversion);
    leftEnc.setVelocityConversionFactor(conversion / 60);
    rightEnc.setPositionConversionFactor(conversion);
    rightEnc.setVelocityConversionFactor(conversion / 60);
    //rightEnc.setInverted(true);
  }

  @Override
  public void periodic() {
    if (odometry != null) {
      odometry.update(Rotation2d.fromDegrees(getHeading()), 
                      leftEnc.getPosition() / Constants.METERS_PER_INCH, 
                      rightEnc.getPosition() / Constants.METERS_PER_INCH);
    }
  }

  public void setOdometry(DifferentialDriveOdometry odometry) {
    this.odometry = odometry;
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public void resetOdometry() {
    if (odometry != null) {
      odometry.resetPosition(odometry.getPoseMeters(), Rotation2d.fromDegrees(getHeading()));
    }
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (isGyroReversed ? -1.0 : 1.0);
  }

  public double getEncPos(Side s) {
    if (s == Side.LEFT) {
      return leftEnc.getPosition();
    } else {
      return -rightEnc.getPosition();
    }
  }

  public double getEncRate(Side s) {
    if (s == Side.LEFT) {
      return leftEnc.getVelocity();
    } else {
      return -rightEnc.getVelocity();
    }
  }

  public CANEncoder getEncoder(Side s) {
    if (s == Side.LEFT) {
      return leftEnc;
    } else {
      return rightEnc;
    }
  }

  public DifferentialDriveKinematics getKinematics() { return kinematics; }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double left, double right) {
    diffDrive.tankDrive(left, right);
  }

  public void charDriveArcade(double speed, double rotation) {
    speed = Math.copySign(speed * speed, speed) * Constants.Drivetrain.MAX_SPEED * Constants.METERS_PER_INCH;
    rotation = Math.copySign(rotation * rotation, rotation) * Constants.Drivetrain.MAX_ANGULAR_SPEED;

    DifferentialDriveWheelSpeeds wheelspeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0.0, -rotation));
    //SmartDashboard.putNumber("WheelSpeedLeft", wheelspeeds.leftMetersPerSecond);
    //SmartDashboard.putNumber("WheelSpeedRight", wheelspeeds.rightMetersPerSecond);
    charDrive(wheelspeeds);
  }

  public void charDriveTank(double left, double right) {
    charDrive(new DifferentialDriveWheelSpeeds(left * Constants.Drivetrain.MAX_SPEED * Constants.METERS_PER_INCH, right * Constants.Drivetrain.MAX_SPEED * Constants.METERS_PER_INCH));
  }

  public void charDriveDirect(double left, double right) {
    charDrive(new DifferentialDriveWheelSpeeds(left, right));
  }

  public void charDrive(DifferentialDriveWheelSpeeds wheelSpeeds) {
    double left = wheelSpeeds.leftMetersPerSecond;
    double right = wheelSpeeds.rightMetersPerSecond;
    final double fwdLeftFF = forwardLeftFF.calculate(left);
    final double backLeftFF = backwardLeftFF.calculate(left);
    final double fwdRightFF = forwardRightFF.calculate(right);
    final double backRightFF = backwardRightFF.calculate(right);

    final double leftOutput = leftPIDController.calculate(getEncRate(Side.LEFT) * Constants.METERS_PER_INCH, left);
    final double rightOutput = rightPIDController.calculate(getEncRate(Side.RIGHT) * Constants.METERS_PER_INCH, right);

    double motorLeftOut = leftOutput + (left >= 0.0 ? fwdLeftFF : backLeftFF);
    double motorRightOut = -rightOutput - (right >= 0.0 ? fwdRightFF : backRightFF);
    motorLeftOut = Math.signum(motorLeftOut) * Math.min(Math.abs(motorLeftOut), Math.abs(left) * 12 / (Constants.Drivetrain.MAX_SPEED * 0.025));
    motorRightOut = Math.signum(motorRightOut) * Math.min(Math.abs(motorRightOut), Math.abs(right) * 12 / (Constants.Drivetrain.MAX_SPEED * 0.025));
    //SmartDashboard.putNumber("MotorLeftOut", motorLeftOut);
    //SmartDashboard.putNumber("MotorRightOut", motorRightOut);
    tankDrive(motorLeftOut / 12.0, -motorRightOut / 12.0);
    //leftMaster.setVoltage(motorLeftOut);
    //rightMaster.setVoltage(motorRightOut);
  }

  /**
   * Characterizes the left/right motor values.
   * 
   * @param left  from -1 to 1
   * @param right from -1 to 1
   */
  public void characterizedDrive(double left, double right) {
    final double actualLeftVel = this.getEncRate(Side.LEFT);
    final double actualRightVel = this.getEncRate(Side.RIGHT);

    SmartDashboard.putNumber("LeftEnc", actualLeftVel);
    SmartDashboard.putNumber("RightEnc", actualRightVel);

    final double desiredLeftVel = Constants.Drivetrain.MAX_SPEED * left;
    final double desiredRightVel = Constants.Drivetrain.MAX_SPEED * right;

    SmartDashboard.putNumber("DesiredLeft", desiredLeftVel);
    SmartDashboard.putNumber("DesiredRight", desiredRightVel);

    double leftAccel = (desiredLeftVel - actualLeftVel) / Constants.Drivetrain.LOOP_TIME;
    double rightAccel = (desiredRightVel - actualRightVel) / Constants.Drivetrain.LOOP_TIME;
    leftAccel = Math.signum(leftAccel) * Math.min(Math.abs(leftAccel), Constants.Drivetrain.MAX_ACCEL);
    rightAccel = Math.signum(rightAccel) * Math.min(Math.abs(rightAccel), Constants.Drivetrain.MAX_ACCEL);
    SmartDashboard.putNumber("LeftAccel", leftAccel);
    SmartDashboard.putNumber("RightAccel", rightAccel);

    double newLeft, newRight;
    if (left >= 0.0) {
        // Forward-Left
        newLeft = Constants.Drivetrain.kVOLTS[0] + Constants.Drivetrain.kVELS[0] * actualLeftVel + Constants.Drivetrain.kACCELS[0] * leftAccel; 
    } else {
        // Backward-Left
        newLeft = -Constants.Drivetrain.kVOLTS[2] + Constants.Drivetrain.kVELS[2] * actualLeftVel + Constants.Drivetrain.kACCELS[2] * leftAccel;
    }
    if (right >= 0.0) {
        // Forward-Right
        newRight = Constants.Drivetrain.kVOLTS[1] + Constants.Drivetrain.kVELS[1] * actualRightVel + Constants.Drivetrain.kACCELS[1] * rightAccel;
    } else {
        // Backward-Right
        newRight = -Constants.Drivetrain.kVOLTS[3] + Constants.Drivetrain.kVELS[3] * actualRightVel + Constants.Drivetrain.kACCELS[3] * rightAccel;
    }
    /*SmartDashboard.putNumber("VForwardLeft", Constants.Drivetrain.kVELS[0] * desiredLeftVel);
    SmartDashboard.putNumber("AForwardLeft", Constants.Drivetrain.kACCELS[0] * leftAccel);
    SmartDashboard.putNumber("VBackwardLeft", Constants.Drivetrain.kVELS[2] * desiredLeftVel);
    SmartDashboard.putNumber("ABackwardLeft", Constants.Drivetrain.kACCELS[2] * leftAccel);
    SmartDashboard.putNumber("VForwardRight", Constants.Drivetrain.kVELS[1] * desiredRightVel);
    SmartDashboard.putNumber("AForwardRight", Constants.Drivetrain.kACCELS[1] * rightAccel);
    SmartDashboard.putNumber("VBackwardRight", Constants.Drivetrain.kVELS[3] * desiredRightVel);
    SmartDashboard.putNumber("ABackwardRight", Constants.Drivetrain.kACCELS[3] * rightAccel);
    */
    newLeft = Math.signum(newLeft) * Math.min(Math.abs(newLeft), 12) / 12;
    newRight = Math.signum(newRight) * Math.min(Math.abs(newRight), 12) / 12;

    SmartDashboard.putNumber("newLeft", newLeft);
    SmartDashboard.putNumber("newRight", newRight);

    diffDrive.tankDrive(newLeft, newRight, false);
  }
}
