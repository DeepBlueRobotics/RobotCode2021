/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2021.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.lib.MotorControllerFactory;
import org.team199.robot2021.Constants;
import org.team199.robot2021.SwerveModule;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // compassOffset is magnetic north relative to the current heading
  private final double compassOffset;
  private final AHRS gyro = new AHRS(SerialPort.Port.kMXP); //Also try kUSB and kUSB2

  private SwerveDriveKinematics kinematics = null;
  private SwerveDriveOdometry odometry = null;
  private SwerveModule modules[];
  private boolean isOdometryInit = false;
  private static final boolean isGyroReversed = true;

  public Drivetrain() {
    gyro.calibrate();
    while (gyro.isCalibrating()) {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        e.printStackTrace();
        break;
      }
      System.out.println("Calibrating the gyro...");
    }
    gyro.reset();
    SmartDashboard.putBoolean("Magnetic Field Disturbance", gyro.isMagneticDisturbance());
    System.out.println("Magnetometer is calibrated: " + gyro.isMagnetometerCalibrated());
    compassOffset = gyro.getCompassHeading();
    SmartDashboard.putNumber("Field Offset from North (degrees)", 134.1);

    // Define the corners of the robot relative to the center of the robot using Translation2d objects.
    // Positive x-values represent moving toward the front of the robot whereas positive y-values represent moving toward the left of the robot.
    Translation2d locationFL = new Translation2d(Constants.DriveConstants.wheelBase / 2, Constants.DriveConstants.trackWidth / 2);
    Translation2d locationFR = new Translation2d(Constants.DriveConstants.wheelBase / 2, -Constants.DriveConstants.trackWidth / 2);
    Translation2d locationBL = new Translation2d(-Constants.DriveConstants.wheelBase / 2, Constants.DriveConstants.trackWidth / 2);
    Translation2d locationBR = new Translation2d(-Constants.DriveConstants.wheelBase / 2, -Constants.DriveConstants.trackWidth / 2);

    kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(Units.degreesToRadians(getHeading())));

    Supplier<Float> pitchSupplier = () -> gyro.getPitch();
    Supplier<Float> rollSupplier = () -> gyro.getRoll();
    SwerveModule moduleFL = new SwerveModule(SwerveModule.ModuleType.FL,
                                MotorControllerFactory.createSparkMax(Constants.DrivePorts.driveFrontLeft), 
                                MotorControllerFactory.createSparkMax(Constants.DrivePorts.turnFrontLeft), 
                                new CANCoder(Constants.DrivePorts.canCoderPortFL), Constants.DriveConstants.driveModifier, 
                                Constants.DriveConstants.maxSpeed, 0, pitchSupplier, rollSupplier);
    // Forward-Right
    SwerveModule moduleFR = new SwerveModule(SwerveModule.ModuleType.FR,
                                MotorControllerFactory.createSparkMax(Constants.DrivePorts.driveFrontRight), 
                                MotorControllerFactory.createSparkMax(Constants.DrivePorts.turnFrontRight),
                                new CANCoder(Constants.DrivePorts.canCoderPortFR), Constants.DriveConstants.driveModifier,
                                Constants.DriveConstants.maxSpeed, 1, pitchSupplier, rollSupplier);
    // Backward-Left
    SwerveModule moduleBL = new SwerveModule(SwerveModule.ModuleType.BL,
                                MotorControllerFactory.createSparkMax(Constants.DrivePorts.driveBackLeft), 
                                MotorControllerFactory.createSparkMax(Constants.DrivePorts.turnBackLeft),
                                new CANCoder(Constants.DrivePorts.canCoderPortBL), Constants.DriveConstants.driveModifier,
                                Constants.DriveConstants.maxSpeed, 2, pitchSupplier, rollSupplier);
    // Backward-Right
    SwerveModule moduleBR = new SwerveModule(SwerveModule.ModuleType.BR,
                                MotorControllerFactory.createSparkMax(Constants.DrivePorts.driveBackRight), 
                                MotorControllerFactory.createSparkMax(Constants.DrivePorts.turnBackRight),
                                new CANCoder(Constants.DrivePorts.canCoderPortBR), Constants.DriveConstants.driveModifier,
                                Constants.DriveConstants.maxSpeed, 3, pitchSupplier, rollSupplier);
    modules = new SwerveModule[]{moduleFL, moduleFR, moduleBL, moduleBR};

    SmartDashboard.putBoolean("Teleop Face Direction of Travel", false);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
      modules[i].updateSmartDashboard();
    }

    // Update the odometry with current heading and encoder position
    odometry.update(Rotation2d.fromDegrees(getHeading()), modules[0].getCurrentState(), modules[1].getCurrentState(),
                    modules[2].getCurrentState(), modules[3].getCurrentState());
  
    SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getTranslation().getY());;
    SmartDashboard.putNumber("Raw gyro angle", gyro.getAngle());
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putNumber("Gyro Compass Heading", gyro.getCompassHeading());
    SmartDashboard.putNumber("Compass Offset", compassOffset);
    SmartDashboard.putBoolean("Current Magnetic Field Disturbance", gyro.isMagneticDisturbance());
  }

  public void setOdometry(SwerveDriveOdometry odometry) {
    if(!isOdometryInit) {
      this.odometry = odometry;
      isOdometryInit = true;
    }
  }

  public SwerveDriveOdometry getOdometry() {
    return odometry;
  }

  public void resetOdometry() {
    odometry.resetPosition(odometry.getPoseMeters(), Rotation2d.fromDegrees(getHeading()));
    isOdometryInit = false;
  }

  public double getHeading() {
    double x = gyro.getAngle();
    if (SmartDashboard.getBoolean("Field Oriented", false)) {
      x += compassOffset - SmartDashboard.getNumber("Field Offset from North (degrees)", 134.1);
    }
    return Math.IEEEremainder(x * (isGyroReversed ? -1.0 : 1.0), 360);
  }

  public SwerveDriveKinematics getKinematics() { return kinematics; }

  public void drive(double forward, double strafe, double rotation) {
    drive(getSwerveStates(forward, strafe, rotation));
  }

  public void drive(SwerveModuleState[] moduleStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.DriveConstants.maxSpeed);
    
    // Move the modules based on desired (normalized) speed, desired angle, max speed, drive modifier, and whether or not to reverse turning.
    for (int i = 0; i < 4; i++) {
      modules[i].move(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getRadians());
    }
  }

  /**
   * Runs homeAbsolute for all of the swerve modules.
   */
  public void homeAbsolute() {
    for (int i = 0; i < 4; i++) modules[i].homeAbsolute();
  }

  /**
   * Constructs and returns a ChassisSpeeds objects using forward, strafe, and rotation values.
   * 
   * @param forward     The desired forward speed, in m/s.
   * @param strafe      The desired strafe speed, in m/s.
   * @param rotation    The desired rotation speed, in rad/s.
   * @return A ChassisSpeeds object.
  */
  private ChassisSpeeds getChassisSpeeds(double forward, double strafe, double rotation) {
    ChassisSpeeds speeds;
    if (SmartDashboard.getBoolean("Field Oriented", false)) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(getHeading()));
    } else {
      speeds = new ChassisSpeeds(forward, strafe, rotation);
    }
    return speeds;
  }

  /**
   * Constructs and returns four SwerveModuleState objects, one for each side, using forward, strafe, and rotation values.
   * 
   * @param forward     The desired forward speed, in m/s.
   * @param strafe      The desired strafe speed, in m/s.
   * @param rotation    The desired rotation speed, in rad/s.
   * @return A SwerveModuleState array, one for each side of the drivetrain (FL, FR, etc.).
  */
  private SwerveModuleState[] getSwerveStates(double forward, double strafe, double rotation) {
    return kinematics.toSwerveModuleStates(getChassisSpeeds(forward, -strafe, -rotation));
  }

  public void toggleMode() {
    for (int i = 0; i < 4; i++) modules[i].toggleMode();
  }

  public void brake() {
    for (int i = 0; i < 4; i++) modules[i].brake();
  }

  public void coast() {
    for (int i = 0; i < 4; i++) modules[i].coast();
  }
}