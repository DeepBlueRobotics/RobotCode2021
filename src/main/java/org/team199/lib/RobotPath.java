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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.team199.robot2020.subsystems.Drivetrain;

//Findd Me
/**
 * Creates {@link Command}s to make the robot follow an {@link Trajectory}
 */
public class RobotPath {

    private Trajectory trajectory;
    private Drivetrain dt;

    /**
     * Creates a new {@link RobotPath}
     * @param pathName The name of the path file. The name will be assumed to be relative to {@link Filesystem#getDeployDirectory()}
     * @param dt The {@link Drivetrain} of the robot
     * @param isInverted Whether the path should be inverted
     * @param initPos The inital position of the path which will be used to offset the path
     * @throws IOException If an error occures reading the path file
     * @see #RobotPath(List, boolean, Drivetrain)
     * @see #RobotPath(List, TrajectoryConfig, Drivetrain)
     * @see #RobotPath(Trajectory, Drivetrain)
     * @see #getPointsFromFile(String, Drivetrain, boolean, Translation2d)
     * @see #getPointsFromFile(File, Drivetrain, boolean, Translation2d)
     */
    public RobotPath(String pathName, Drivetrain dt, boolean isInverted, Translation2d initPos) throws IOException {
        this(getPointsFromFile(pathName, dt, isInverted, initPos), isInverted, dt);
    }

    /**
     * Creates a new {@link RobotPath}
     * @param poses A {@link List} of {@link Pose2d}s representing waypoints of a path to create
     * @param isInverted Whether the path should be inverted
     * @param dt The {@link Drivetrain} of the robot
     * @see #RobotPath(String, Drivetrain, boolean, Translation2d)
     * @see #RobotPath(List, TrajectoryConfig, Drivetrain)
     * @see #RobotPath(Trajectory, Drivetrain)
     * @see #createConfig(boolean, Drivetrain)
     */
    public RobotPath(List<Pose2d> poses, boolean isInverted, Drivetrain dt) {
        this(poses, createConfig(isInverted, dt), dt);
    }

    /**
     * Creates a new {@link RobotPath}
     * @param poses A {@link List} of {@link Pose2d}s representing waypoints of a path to create
     * @param config The {@link TrajectoryConfig} to use when creating the path
     * @param dt The {@link Drivetrain} of the robot
     * @see #RobotPath(String, Drivetrain, boolean, Translation2d)
     * @see #RobotPath(List, boolean, Drivetrain)
     * @see #RobotPath(Trajectory, Drivetrain)
     * @see TrajectoryGenerator#generateTrajectory(List, TrajectoryConfig)
     */
    public RobotPath(List<Pose2d> poses, TrajectoryConfig config, Drivetrain dt) {
        this(TrajectoryGenerator.generateTrajectory(poses, config), dt);
    }

    /**
     * Creates a new {@link RobotPath}
     * @param trajectory The {@link Trajectory} to follow
     * @param dt The {@link Drivetrain} of the robot
     * @see #RobotPath(String, Drivetrain, boolean, Translation2d)
     * @see #RobotPath(List, boolean, Drivetrain)
     * @see #RobotPath(List, TrajectoryConfig, Drivetrain)
     */
    public RobotPath(Trajectory trajectory, Drivetrain dt) {
        this.trajectory = trajectory;
        this.dt = dt;
    }

    /**
     * Generates a {@link Command} to run the path
     * @return A {@link Command} to run the path
     * @see #loadOdometry()
     * @see RamseteCommand
     */
    public Command getPathCommand() {
        RamseteCommand ram = new RamseteCommand(trajectory, 
                                                () -> dt.getOdometry().getPoseMeters(), 
                                                new RamseteController(), 
                                                dt.getKinematics(),
                                                dt::charDriveDirect,
                                                dt);
        SequentialCommandGroup command = new InstantCommand(this::loadOdometry).andThen(ram, new InstantCommand(() -> dt.charDriveTank(0, 0), dt));
        command.setName("RobotPath");
        return command;
    }

    /**
     * Sets the drivetrain's odometry to the start of the path unless it has already been set
     * @see Drivetrain#setOdometry(DifferentialDriveOdometry)
     */
    public void loadOdometry() {
        dt.setOdometry(trajectory.getInitialPose());
    }

    /**
     * @return The {@link Trajectory} to be followed by this {@link RobotPath}
     */
    public Trajectory getTrajectory() {
        return trajectory;
    }

    /**
     * Creates a {@link TrajectoryConfig} with default settings determined by the {@link Drivetrain}
     * @param isInverted Whether to invert the {@lik TrajectoryConfig}
     * @param dt The {@link Drivetrain} of the robot
     * @return A {@link TrajectoryConfig} with default settings determined by the {@link Drivetrain}
     */
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

    /**
     * Retrieves a {@link List} of {@link Pose2d}s from a file
     * @param pathName The name of the path file. The name will be assumed to be relative to {@link Filesystem#getDeployDirectory()}
     * @param dt The {@link Drivetrain} of the robot
     * @param isInverted Whether the path should be inverted
     * @param initPos The inital position of the path which will be used to offset the path
     * @return A {@link List} of {@link Pose2d}s from the file
     * @throws IOException If an error occures reading the path file
     * @see #RobotPath(String, Drivetrain, boolean, Translation2d)
     */
    public static List<Pose2d> getPointsFromFile(String pathName, Drivetrain dt, boolean isInverted, Translation2d initPos) throws IOException {
        return getPointsFromFile(getPathFile(pathName), dt, isInverted, initPos);
    }

    /**
     * Retrieves a {@link List} of {@link Pose2d}s from a file
     * @param file The file ftom which to load
     * @param dt The {@link Drivetrain} of the robot
     * @param isInverted Whether the path should be inverted
     * @param initPos The inital position of the path which will be used to offset the path
     * @return A {@link List} of {@link Pose2d}s from the file
     * @throws IOException If an error occures reading the path file
     * @see #RobotPath(String, Drivetrain, boolean, Translation2d)
     */
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

    /**
     * @return The {@link File} in which a path is stored based on the path name
     */
    public static File getPathFile(String pathName) {
        return Filesystem.getDeployDirectory().toPath().resolve(Paths.get("PathWeaver/Paths/" + pathName + ".path")).toFile();
    }

    /**
     * An {@link Enum} of path descriptors used for autonomous path selection
     */
    public static enum Path {
        /**
         * Represents the first autonomous path
         */
        PATH1(0),
        /**
         * Represents the second autonomous path
         */
        PATH2(1),
        /**
         * Represents the third autonomous path
         */
        PATH3(2),
        /**
         * Represents the fourth autonomous path
         */
        PATH4(3),
        /**
         * Represents the fifth autonomous path
         */
        PATH5(4),
        /**
         * Represents the sixth autonomous path
         */
        PATH6(5),
        /**
         * Represents the seventh autonomous path
         */
        PATH7(6),
        /**
         * Represents no autonomous path
         */
        OFF(-1);

        /**
         * The array index to be used when storing paths of a {@link Path} type
         */
        public final int idx;

        private Path(final int idx) {
            this.idx = idx;
        }

        /**
         * @param idx The <code>idx</code> of the path to return
         * @return The {@link Path} with the specified <code>idx</code>
         */
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