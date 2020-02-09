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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
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

  private final SimpleMotorFeedforward forwardLeftFF = 
    new SimpleMotorFeedforward(Constants.Drivetrain.kVOLTS[0], 
                               Constants.Drivetrain.kVELS[0], 
                               Constants.Drivetrain.kACCELS[0]);
  private final SimpleMotorFeedforward forwardRightFF = 
    new SimpleMotorFeedforward(Constants.Drivetrain.kVOLTS[1], 
                               Constants.Drivetrain.kVELS[1], 
                               Constants.Drivetrain.kACCELS[1]);
  private final SimpleMotorFeedforward backwardLeftFF = 
    new SimpleMotorFeedforward(Constants.Drivetrain.kVOLTS[2], 
                               Constants.Drivetrain.kVELS[2], 
                               Constants.Drivetrain.kACCELS[2]);
  private final SimpleMotorFeedforward backwardRightFF = 
    new SimpleMotorFeedforward(Constants.Drivetrain.kVOLTS[3], 
                               Constants.Drivetrain.kVELS[3], 
                               Constants.Drivetrain.kACCELS[3]);
  
  private DifferentialDriveOdometry odometry = null;
  private boolean isOdometryInit = false;
  private static final boolean isGyroReversed = true;
  private Timer timey = new Timer();

  private double prevTime = 0;
  private double prevLeftVel = 0;
  private double prevRightVel = 0;

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
    timey.start();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    gyro.reset();
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), 
                    Units.inchesToMeters(getEncPos(Side.LEFT)), 
                    Units.inchesToMeters(getEncPos(Side.RIGHT)));
                  SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getTranslation().getX());
                  SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Left Encoder Distance", getEncPos(Drivetrain.Side.LEFT));
    SmartDashboard.putNumber("Right Encoder Distance", getEncPos(Drivetrain.Side.RIGHT));
    SmartDashboard.putNumber("Left Encoder CPR", leftEnc.getCountsPerRevolution());
    SmartDashboard.putNumber("Right Encoder CPR", rightEnc.getCountsPerRevolution());
    SmartDashboard.putNumber("Gyro Heading", getHeading());
  }

  public void setOdometry(DifferentialDriveOdometry odometry) {
    if(!isOdometryInit) {
      this.odometry = odometry;
      isOdometryInit = true;
    }
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public void resetOdometry() {
    odometry.resetPosition(odometry.getPoseMeters(), Rotation2d.fromDegrees(getHeading()));
    resetEncoders();
    isOdometryInit = false;
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

  public void resetEncoders() {
    leftEnc.setPosition(0);
    rightEnc.setPosition(0);
  }

  public DifferentialDriveKinematics getKinematics() { return kinematics; }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double left, double right, boolean squareInputs) {
    diffDrive.tankDrive(left, right, squareInputs);
    diffDrive.feed();
  }

  public void charDriveArcade(double speed, double rotation) {
    //charDrive(kinematics.toWheelSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0)));
    speed = Math.copySign(speed * speed, speed) * Units.inchesToMeters(Constants.Drivetrain.MAX_SPEED);
    rotation = Math.copySign(rotation * rotation, rotation) * Constants.Drivetrain.MAX_ANGULAR_SPEED;

    DifferentialDriveWheelSpeeds wheelspeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0.0, -rotation));
    //double left = Math.signum(wheelspeeds.leftMetersPerSecond) * Math.min(wheelspeeds.leftMetersPerSecond, 1.27);
    //double right = Math.signum(wheelspeeds.rightMetersPerSecond) * Math.min(wheelspeeds.rightMetersPerSecond, 1.27);
    //SmartDashboard.putNumber("WheelSpeedLeft", wheelspeeds.leftMetersPerSecond);
    //SmartDashboard.putNumber("WheelSpeedRight", wheelspeeds.rightMetersPerSecond);
    charDrive(wheelspeeds);
  }

  public void charDriveTank(double left, double right) {
    left = left * Units.inchesToMeters(Constants.Drivetrain.MAX_SPEED);
    right = right * Units.inchesToMeters(Constants.Drivetrain.MAX_SPEED);
    charDriveDirect(left, right);
  }

  public void charDriveDirect(double left, double right) {
    left = Math.signum(left) * Math.min(Math.abs(left), 1.27);
    right = Math.signum(right) * Math.min(Math.abs(right), 1.27);
    charDrive(new DifferentialDriveWheelSpeeds(left, right));
  }

  public void charDrive(DifferentialDriveWheelSpeeds wheelSpeeds) {
    double currentTime = timey.get();

    double desiredLeftVel = wheelSpeeds.leftMetersPerSecond;
    double actualLeftVel = Units.inchesToMeters(getEncRate(Side.LEFT));
    
    double leftAccel = (desiredLeftVel - prevLeftVel) / (currentTime - prevTime);

    double desiredRightVel = wheelSpeeds.rightMetersPerSecond;
    double actualRightVel = Units.inchesToMeters(getEncRate(Side.RIGHT));
    double rightAccel = (desiredRightVel - prevRightVel) / (currentTime - prevTime);

    SmartDashboard.putNumber("DesiredLeftVel", desiredLeftVel);
    SmartDashboard.putNumber("DesiredRightVel", desiredRightVel);
    SmartDashboard.putNumber("ActualLeftVel", actualLeftVel);
    SmartDashboard.putNumber("ActualRightVel", actualRightVel);
    SmartDashboard.putNumber("LeftAppliedVolts", leftMaster.getAppliedOutput());
    SmartDashboard.putNumber("RightAppliedVolts", rightMaster.getAppliedOutput());

    final double fwdLeftFF = forwardLeftFF.calculate(desiredLeftVel, leftAccel);
    //System.out.println(desiredLeftVel + ", " + leftAccel + ", " + fwdLeftFF);
    final double backLeftFF = backwardLeftFF.calculate(desiredLeftVel, leftAccel);
    final double fwdRightFF = forwardRightFF.calculate(desiredRightVel, rightAccel);
    final double backRightFF = backwardRightFF.calculate(desiredRightVel, rightAccel);

    SmartDashboard.putNumber("fwdLeftFF", fwdLeftFF);

    final double leftOutput = leftPIDController.calculate(actualLeftVel, desiredLeftVel);
    final double rightOutput = rightPIDController.calculate(actualRightVel, desiredRightVel);

    double motorLeftOut = leftOutput + (desiredLeftVel >= 0.0 ? fwdLeftFF : backLeftFF);
    double motorRightOut = rightOutput + (desiredRightVel >= 0.0 ? fwdRightFF : backRightFF);
    // Clip motor voltages at desired voltage. Tank Drive already clips motor inputs.
    //motorLeftOut = Math.signum(motorLeftOut) * Math.min(Math.abs(motorLeftOut), Math.abs(left) * 12.0 / (Constants.Drivetrain.MAX_SPEED * 0.0254));
    //motorRightOut = Math.signum(motorRightOut) * Math.min(Math.abs(motorRightOut), Math.abs(right) * 12.0 / (Constants.Drivetrain.MAX_SPEED * 0.0254));
    //SmartDashboard.putNumber("MotorLeftOut", motorLeftOut);
    //SmartDashboard.putNumber("MotorRightOut", motorRightOut);
    tankDrive(motorLeftOut / 12.0, motorRightOut / 12.0, false);
    //leftMaster.setVoltage(motorLeftOut);
    //rightMaster.setVoltage(motorRightOut);
    prevTime = currentTime;
    prevLeftVel = desiredLeftVel;
    prevRightVel = desiredRightVel;
  }
}
