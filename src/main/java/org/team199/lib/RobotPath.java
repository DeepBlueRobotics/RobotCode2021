package org.team199.lib;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.spline.Spline.ControlVector;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.lib.swerve.SwerveDriveVoltageConstraint;

import org.team199.robot2021.Constants;
import org.team199.robot2021.subsystems.Drivetrain;

//Findd Me
public class RobotPath {

    public Trajectory trajectory;
    private Drivetrain dt;

    /*
    public RobotPath(String pathName, Drivetrain dt, boolean isInverted, double endVelocity) throws IOException {
        Trajectory fileTrajectory = 
            TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(Paths.get("PathWeaver/Paths/" + pathName + ".wpilib.json")));
        // TODO: Use control vectors instead of poses to generate the constrained trajectory.
        List<Trajectory.State> states = fileTrajectory.getStates();
        List<Pose2d> poses = new ArrayList<>();
        for (Trajectory.State state : states) poses.add(state.poseMeters);
        
        trajectory = TrajectoryGenerator.generateTrajectory(poses, createConfig(isInverted, dt, endVelocity));
        this.dt = dt;
    }
    */
    public RobotPath(String pathName, Drivetrain dt, boolean isInverted, double endVelocity) throws IOException {
       this(getPointsFromFile(pathName, dt), isInverted, dt, endVelocity);
    }

    public RobotPath(ControlVectorList vectors, boolean isInverted, Drivetrain dt, double endVelocity) {
        this(vectors, createConfig(isInverted, dt, endVelocity), dt);
    }

    public RobotPath(ControlVectorList vectors, TrajectoryConfig config, Drivetrain dt) {
        this(TrajectoryGenerator.generateTrajectory(vectors, config), dt);
    }

    public RobotPath(Trajectory trajectory, Drivetrain dt) {
        this.trajectory = trajectory;
        this.dt = dt;
    }

    public SwerveModuleState[] convertToFieldRelative(SwerveModuleState[] swerveModuleStates, Translation2d centerOfRotation) {
        if (SmartDashboard.getBoolean("Field Oriented", false)) {
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
        trajectory = trajectory.relativeTo(trajectory.getInitialPose());
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

    public static TrajectoryConfig createConfig(boolean isInverted, Drivetrain dt, double endVelocity) {
        TrajectoryConfig config = new TrajectoryConfig(Constants.DriveConstants.autoMaxSpeed, 
                                                       Constants.DriveConstants.autoMaxAccel);
        // Limit the modules based on a maximum speed
        config.addConstraint(new SwerveDriveKinematicsConstraint(dt.getKinematics(),
                                                                 Constants.DriveConstants.autoMaxSpeed));
        // Ensure that the robot turns slowly around tight turns and doesn't slip
        config.addConstraint(new CentripetalAccelerationConstraint(Constants.DriveConstants.autoCentripetalAccel));
        config.setEndVelocity(endVelocity);
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

    public static ControlVectorList getPointsFromFile(String pathName, Drivetrain dt) throws IOException {
        return getVectorsFromFile(getPathFile(pathName), dt);
    }

    public static ControlVectorList getVectorsFromFile(File file, Drivetrain dt) throws IOException {
        ControlVectorList vectors = new ControlVectorList();

        try {
            CSVParser csvParser = CSVFormat.DEFAULT.parse(new FileReader(file));
            double x, y, tanx, tany;
            List<CSVRecord> records = csvParser.getRecords();

            for (int i = 1; i < records.size(); i++) {
                CSVRecord record = records.get(i);
                x = Double.parseDouble(record.get(0));
                y = Double.parseDouble(record.get(1));
                tanx = Double.parseDouble(record.get(2));
                tany = Double.parseDouble(record.get(3));
                vectors.add(new ControlVector(new double[]{x, tanx, 0}, new double[]{y, tany, 0}));
            }
            csvParser.close();
        } catch (FileNotFoundException e) {
            System.out.println("File named: " + file.getAbsolutePath() + " not found.");
            e.printStackTrace();
        }
        return vectors;
    }

    public static File getPathFile(String pathName) {
        return Filesystem.getDeployDirectory().toPath().resolve(Paths.get("PathWeaver/Paths/" + pathName + ".path")).toFile();
    }
}