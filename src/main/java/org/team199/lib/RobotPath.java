package org.team199.lib;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;

import frc.robot.lib.swerve.SwerveDriveVoltageConstraint;

import org.team199.robot2021.Constants;
import org.team199.robot2021.subsystems.Drivetrain;

//Findd Me
public class RobotPath {

    private Trajectory trajectory;
    private Drivetrain dt;

    public RobotPath(String pathName, Drivetrain dt) throws IOException {
        trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(Paths.get("PathWeaver/Paths/" + pathName + ".wpilib.json")));
        this.dt = dt;
    }

    public RobotPath(String pathName, Drivetrain dt, boolean isInverted, Translation2d initPos) throws IOException {
        this(getPointsFromFile(pathName, dt, isInverted, initPos), isInverted, dt);
    }

    public RobotPath(List<Pose2d> poses, boolean isInverted, Drivetrain dt) {
        this(poses, createConfig(isInverted, dt), dt);
    }

    public RobotPath(List<Pose2d> poses, TrajectoryConfig config, Drivetrain dt) {
        this(TrajectoryGenerator.generateTrajectory(poses, config), dt);
    }

    public RobotPath(Trajectory trajectory, Drivetrain dt) {
        this.trajectory = trajectory;
        this.dt = dt;
    }

    public SwerveModuleState[] convertToFieldRelative(SwerveModuleState[] swerveModuleStates, Translation2d centerOfRotation) {
        if (SmartDashboard.getBoolean("Field Oriented", true)) {
            ChassisSpeeds speeds = dt.getKinematics().toChassisSpeeds(swerveModuleStates);
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, 
                                                           speeds.vyMetersPerSecond, 
                                                           speeds.omegaRadiansPerSecond, 
                                                           Rotation2d.fromDegrees(dt.getHeading()));
            return dt.getKinematics().toSwerveModuleStates(speeds, centerOfRotation);
        } else return swerveModuleStates;
    }
    

    public Command getPathCommand(Rotation2d desiredHeading) {
        ProfiledPIDController thetaController =  new ProfiledPIDController(Constants.DriveConstants.thetaPIDController[0],
                                                                           Constants.DriveConstants.thetaPIDController[1],
                                                                           Constants.DriveConstants.thetaPIDController[2],
                                                                           new Constraints(Constants.DriveConstants.autoMaxSpeed, Constants.DriveConstants.autoMaxAccel));
        SwerveControllerCommand ram = new SwerveControllerCommand(
            trajectory,
            () -> dt.getOdometry().getPoseMeters(),
            dt.getKinematics(),
            new PIDController(Constants.DriveConstants.xPIDController[0],
                              Constants.DriveConstants.xPIDController[1],
                              Constants.DriveConstants.xPIDController[2]),

            new PIDController(Constants.DriveConstants.yPIDController[0],
                              Constants.DriveConstants.yPIDController[1],
                              Constants.DriveConstants.yPIDController[2]),
            thetaController,
            () -> desiredHeading,
            (swerveModuleStates) -> dt.drive(convertToFieldRelative(swerveModuleStates, new Translation2d())),
            dt
            );

        return new InstantCommand(this::loadOdometry).andThen(ram, new InstantCommand(() -> dt.drive(0, 0, 0), dt));
    }

    public void loadOdometry() {
        dt.setOdometry(new SwerveDriveOdometry(dt.getKinematics(), Rotation2d.fromDegrees(dt.getHeading()), trajectory.getInitialPose()));
    }

    private static double average(double[] arr) {
        double sum = 0;
        for (double x : arr) sum += x;
        return sum / arr.length;
    }

    public static TrajectoryConfig createConfig(boolean isInverted, Drivetrain dt) {
        TrajectoryConfig config = new TrajectoryConfig(Constants.DriveConstants.autoMaxSpeed, 
                                                       Constants.DriveConstants.autoMaxAccel);
        config.setKinematics(dt.getKinematics());

        /*
        double kVoltAVG = 0.5 * (average(Constants.DriveConstants.kForwardVolts) 
                                 + average(Constants.DriveConstants.kBackwardVolts));
        double kVelsAVG = 0.5 * (average(Constants.DriveConstants.kForwardVels) 
                                 + average(Constants.DriveConstants.kBackwardVels));
        double kAccelAVG = 0.5 * (average(Constants.DriveConstants.kForwardAccels) 
                                 + average(Constants.DriveConstants.kBackwardAccels));
        SwerveDriveVoltageConstraint voltConstraint = new SwerveDriveVoltageConstraint(
            new SimpleMotorFeedforward(kVoltAVG, kVelsAVG, kAccelAVG), 
            dt.getKinematics(),
            Constants.DriveConstants.trackWidth,
            Constants.DriveConstants.autoMaxVolt);
        config.addConstraint(voltConstraint);
        */
        
        if (isInverted) { config.setReversed(true); }
        return config;
    }

    public static List<Pose2d> getPointsFromFile(String pathName, Drivetrain dt, boolean isInverted, Translation2d initPos) throws IOException {
        return getPointsFromFile(getPathFile(pathName), dt, isInverted, initPos);
    }

    public static List<Pose2d> getPointsFromFile(File file, Drivetrain dt, boolean isInverted, Translation2d initPos) throws IOException {
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();

        try {
            CSVParser csvParser = CSVFormat.DEFAULT.parse(new FileReader(file));
            double x, y, tanx, tany;
            Rotation2d rot;
            List<CSVRecord> records = csvParser.getRecords();

            for (int i = 1; i < records.size(); i++) {
                CSVRecord record = records.get(i);
                x = Double.parseDouble(record.get(0)) + initPos.getX();
                y = Double.parseDouble(record.get(1)) + initPos.getY();
                tanx = Double.parseDouble(record.get(2));
                tany = Double.parseDouble(record.get(3));
                rot = new Rotation2d(tanx, tany);
                if (isInverted) { rot = rot.rotateBy(new Rotation2d(Math.PI)); }
                poses.add(new Pose2d(x, y, rot));
            }
            csvParser.close();
        } catch (FileNotFoundException e) {
            System.out.println("File named: " + file.getAbsolutePath() + " not found.");
            e.printStackTrace();
        }

        return poses;
    }

    public static File getPathFile(String pathName) {
        return Filesystem.getDeployDirectory().toPath().resolve(Paths.get("PathWeaver/Paths/" + pathName + ".path")).toFile();
    }
}