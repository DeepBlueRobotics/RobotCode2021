package org.team199.lib;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
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
    private boolean isInverted;

    public RobotPath(String filename, Drivetrain dt, boolean isInverted) throws IOException {
        TrajectoryConfig config = new TrajectoryConfig(dt.kAutoMaxSpeed, 
                                                       dt.kAutoMaxAccel);
        config.setKinematics(dt.getKinematics());

        double kVoltAVG = 0.25 * (dt.kVolts[0] + dt.kVolts[1] + dt.kVolts[2] + dt.kVolts[3]);
        double kVelsAVG = 0.25 * (dt.kVels[0] + dt.kVels[1] + dt.kVels[2] + dt.kVels[3]);
        double kAccelAVG = 0.25 * (dt.kAccels[0] + dt.kAccels[1] + dt.kAccels[2] + dt.kAccels[3]);
        DifferentialDriveVoltageConstraint voltConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(kVoltAVG, kVelsAVG, kAccelAVG), dt.getKinematics(), dt.kAutoMaxVolt);
        config.addConstraint(voltConstraint);
        if (isInverted) { config.setReversed(true); }
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        
        try {
            CSVParser csvParser = CSVFormat.DEFAULT.parse(new FileReader("/home/lvuser/deploy/paths/" + filename + ".path"));
            double x, y, tanx, tany;
            Rotation2d rot;
            
            int count = 0;
            for (CSVRecord record : csvParser) {
                if (count > 0) {
                    x = Double.parseDouble(record.get(0));
                    y = Double.parseDouble(record.get(1));
                    if (isInverted) {
                        x *= -1;
                        y *= -1;
                    }
                    tanx = Double.parseDouble(record.get(2));
                    tany = Double.parseDouble(record.get(3));
                    rot = new Rotation2d(tanx, tany);
                    if (isInverted) { rot.rotateBy(new Rotation2d(Math.PI)); }
                    System.out.println("x: " + x + ", y: " + y + ", tanx: " + tanx + ", tany: " + tany);
                    poses.add(new Pose2d(x, y, rot));
                }
                count++;
            }
            csvParser.close();
        } catch (FileNotFoundException e) {
            System.out.println("File named /home/lvuser/deploy/paths/" + filename + ".path not found.");
            e.printStackTrace();
        }

        trajectory = TrajectoryGenerator.generateTrajectory(poses, config);
        this.dt = dt;
        this.isInverted = isInverted;
    }

    public Command getPathCommand() {
        RamseteCommand ram = new RamseteCommand(trajectory, 
                                                () -> dt.getOdometry().getPoseMeters(), 
                                                new RamseteController(), 
                                                dt.getKinematics(),
                                                dt::charDriveDirect,
                                                dt);
        return new InstantCommand(this::loadOdometry).andThen(ram, new InstantCommand(() -> dt.charDriveTank(0, 0), dt)); //TODO: Configure Ramsete Controller Values
    }

    public void loadOdometry() {
        dt.setOdometry(new DifferentialDriveOdometry(Rotation2d.fromDegrees(dt.getHeading()), trajectory.getInitialPose()));
    }
}