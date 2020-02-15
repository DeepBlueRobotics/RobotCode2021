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

import edu.wpi.first.wpilibj.SerialPort;
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

  private static final double kTrackWidth = 0.635;    // Distance between the centers of the left and right wheels in meters.
  private static final double[] kPIDLeft = {2.1775, 0.0, 0.0};  // PID values for the left PID Contoller for characterization.
  private static final double[] kPIDRight = {2.875, 0.0, 0.0};  // PID values for the right PID Contoller for characterization.

  // Order of the entries: {Forward-Left, Forward-Right, Backward-Left, Backward-Right}
  public static final double[] kVolts = {0.146, 0.129, 0.154, 0.134};  // Volts
  public static final double[] kVels = {2.11, 2.04, 2.1, 2.04};  // Volt * seconds / meter
  public static final double[] kAccels = {0.222, 0.299, 0.195, 0.275};  // Volt * seconds^2 / meter

  public static final double kAutoMaxSpeed = 2.54;  // Meters / second
  public static final double kAutoMaxAccel = 0.847;  // Meters / seconds^2
  public static final double kMaxAccel = 200;  // Inches / seconds^2
  public static final double kMaxSpeed = 5676 * Math.PI * 5 / 6.8 / 60; // Inches / seconds
  public static final double kMaxAngularSpeed = 4 * Math.PI; // Radians / second
  public static final double kAutoMaxVolt = 10.0;   // For Drivetrain voltage constraint in RobotPath.java
  
  private final CANSparkMax leftMaster = MotorControllerFactory.createSparkMax(Constants.Drive.kDtLeftMaster);
  private final CANSparkMax leftSlave = MotorControllerFactory.createSparkMax(Constants.Drive.kDtLeftSlave);
  private final CANSparkMax rightMaster = MotorControllerFactory.createSparkMax(Constants.Drive.kDtRightMaster);
  private final CANSparkMax rightSlave = MotorControllerFactory.createSparkMax(Constants.Drive.kDtRightSlave);

  private final CANEncoder leftEnc = leftMaster.getEncoder();
  private final CANEncoder rightEnc = rightMaster.getEncoder();

  private final AHRS gyro = new AHRS(SerialPort.Port.kUSB1); //Also try kUSB and kUSB2

  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

  private final PIDController leftPIDController = new PIDController(kPIDLeft[0], kPIDLeft[1], kPIDLeft[2]);
  private final PIDController rightPIDController = new PIDController(kPIDRight[0], kPIDRight[1], kPIDRight[2]);
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);

  /* Feedforward objects for the left and right sides of the drivetrain.
     Use the "forward" prefix when the motor controller is moving forward and the "backward" prefix when
     the motor controller is moving backwards. */
  private final SimpleMotorFeedforward forwardLeftFF = new SimpleMotorFeedforward(kVolts[0], 
                                                                                  kVels[0], 
                                                                                  kAccels[0]);
  private final SimpleMotorFeedforward forwardRightFF = new SimpleMotorFeedforward(kVolts[1], 
                                                                                   kVels[1], 
                                                                                   kAccels[1]);
  private final SimpleMotorFeedforward backwardLeftFF = new SimpleMotorFeedforward(kVolts[2], 
                                                                                   kVels[2], 
                                                                                   kAccels[2]);
  private final SimpleMotorFeedforward backwardRightFF = new SimpleMotorFeedforward(kVolts[3], 
                                                                                    kVels[3], 
                                                                                    kAccels[3]);
  
  private DifferentialDriveOdometry odometry = null;
  private boolean isOdometryInit = false;
  private static final boolean isGyroReversed = true;
  private final Timer timey = new Timer();

  // For drivetrain characterization.
  private double currentTime;
  private double prevTime = 0;
  private double prevLeftVel = 0;
  private double prevRightVel = 0;

  public Drivetrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);

    // Conversion factor = Circumference / Gearing.
    double conversion = (Math.PI * 5.0) / 6.8;
    leftEnc.setPositionConversionFactor(conversion);
    leftEnc.setVelocityConversionFactor(conversion / 60);   // By default, encoder velocity is reported in rpm.
    rightEnc.setPositionConversionFactor(conversion);
    rightEnc.setVelocityConversionFactor(conversion / 60);

    timey.start();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    gyro.reset();
  }

  @Override
  public void periodic() {
    // Update the odometry with current heading and encoder position
    odometry.update(Rotation2d.fromDegrees(getHeading()), 
                    Units.inchesToMeters(getEncPos(Side.LEFT)), 
                    Units.inchesToMeters(getEncPos(Side.RIGHT)));

    SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Left Encoder Distance", getEncPos(Drivetrain.Side.LEFT));
    SmartDashboard.putNumber("Right Encoder Distance", getEncPos(Drivetrain.Side.RIGHT));
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
    // The arguments to wheelspeeds are in m/s and radians.
    speed = Math.copySign(speed * speed, speed) * Units.inchesToMeters(kMaxSpeed);
    rotation = Math.copySign(rotation * rotation, rotation) * kMaxAngularSpeed;

    DifferentialDriveWheelSpeeds wheelspeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0.0, -rotation));
    charDrive(wheelspeeds);
  }

  public void charDriveTank(double left, double right) {
    left = left * Units.inchesToMeters(kMaxSpeed);
    right = right * Units.inchesToMeters(kMaxSpeed);
    charDriveDirect(left, right);
  }

  public void charDriveDirect(double left, double right) {
    charDrive(new DifferentialDriveWheelSpeeds(left, right));
  }

  public void charDrive(DifferentialDriveWheelSpeeds wheelSpeeds) {
    currentTime = timey.get();

    final double desiredLeftVel = wheelSpeeds.leftMetersPerSecond;
    final double actualLeftVel = Units.inchesToMeters(getEncRate(Side.LEFT));
    final double leftAccel = (desiredLeftVel - prevLeftVel) / (currentTime - prevTime);

    final double desiredRightVel = wheelSpeeds.rightMetersPerSecond;
    final double actualRightVel = Units.inchesToMeters(getEncRate(Side.RIGHT));
    final double rightAccel = (desiredRightVel - prevRightVel) / (currentTime - prevTime);

    // FF.calculate = kS + kV * velocity + kA * acceleration
    final double fwdLeftFF = forwardLeftFF.calculate(desiredLeftVel, leftAccel);
    final double backLeftFF = backwardLeftFF.calculate(desiredLeftVel, leftAccel);
    final double fwdRightFF = forwardRightFF.calculate(desiredRightVel, rightAccel);
    final double backRightFF = backwardRightFF.calculate(desiredRightVel, rightAccel);

    final double leftOutput = leftPIDController.calculate(actualLeftVel, desiredLeftVel);
    final double rightOutput = rightPIDController.calculate(actualRightVel, desiredRightVel);

    final double motorLeftOut = leftOutput + (desiredLeftVel >= 0.0 ? fwdLeftFF : backLeftFF);
    final double motorRightOut = rightOutput + (desiredRightVel >= 0.0 ? fwdRightFF : backRightFF);

    tankDrive(motorLeftOut / 12.0, motorRightOut / 12.0, false);

    prevTime = currentTime;
    prevLeftVel = desiredLeftVel;
    prevRightVel = desiredRightVel;
  }
}