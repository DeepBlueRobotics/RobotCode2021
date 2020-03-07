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
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import org.team199.robot2020.subsystems.Drivetrain;

//Findd Me
public class RobotPath {

    private Trajectory trajectory;
    private Drivetrain dt;

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

    public Command getPathCommand() {
        RamseteCommand ram = new RamseteCommand(trajectory, 
                                                () -> dt.getOdometry().getPoseMeters(), 
                                                new RamseteController(), 
                                                dt.getKinematics(),
                                                dt::charDriveDirect,
                                                dt);
        return new InstantCommand(this::loadOdometry).andThen(ram, new InstantCommand(() -> dt.charDriveTank(0, 0), dt));
    }

    public void loadOdometry() {
        dt.setOdometry(new DifferentialDriveOdometry(Rotation2d.fromDegrees(dt.getHeading()), trajectory.getInitialPose()));
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public static TrajectoryConfig createConfig(boolean isInverted, Drivetrain dt) {
        TrajectoryConfig config = new TrajectoryConfig(Drivetrain.kAutoMaxSpeed, 
                                                       Drivetrain.kAutoMaxAccel);
        config.setKinematics(dt.getKinematics());

        double kVoltAVG = 0.25 * (Drivetrain.kVolts[0] + Drivetrain.kVolts[1] + Drivetrain.kVolts[2] + Drivetrain.kVolts[3]);
        double kVelsAVG = 0.25 * (Drivetrain.kVels[0] + Drivetrain.kVels[1] + Drivetrain.kVels[2] + Drivetrain.kVels[3]);
        double kAccelAVG = 0.25 * (Drivetrain.kAccels[0] + Drivetrain.kAccels[1] + Drivetrain.kAccels[2] + Drivetrain.kAccels[3]);
        DifferentialDriveVoltageConstraint voltConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(kVoltAVG, kVelsAVG, kAccelAVG), dt.getKinematics(), Drivetrain.kAutoMaxVolt);
        config.addConstraint(voltConstraint);
        
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

    public static enum Path {
        PATH1(0), PATH2(1), PATH3(2), PATH4(3), PATH5(4), PATH6(5), PATH7(6), OFF(-1);

        public final int idx;

        private Path(final int idx) {
            this.idx = idx;
        }

        public static final Path fromIdx(int idx) {
            switch(idx) {
                case 0:
                return PATH1;
                case 1:
                return PATH2;
                case 2:
                return PATH3;
                case 3:
                return PATH4;
                case 4:
                return PATH5;
                case 5:
                return PATH6;
                case 6:
                return PATH7;
                case -1:
                default:
                return OFF;
            }
        }
    }
}