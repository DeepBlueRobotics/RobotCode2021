package org.team199.lib;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

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

import org.team199.robot2020.Constants;
import org.team199.robot2020.subsystems.Drivetrain;

//Findd Me
public class RobotPath {

    private Trajectory trajectory;
    private Drivetrain dt;
    private boolean isInverted;

    public RobotPath(String filename, Drivetrain dt, boolean isInverted) throws IOException {
        TrajectoryConfig config = new TrajectoryConfig(Constants.Drivetrain.kAUTOMAXSPEED, 
                                                       Constants.Drivetrain.kAUTOMAXACCEL);
        config.setKinematics(dt.getKinematics());

        double kVoltAVG = 0.25 * (Constants.Drivetrain.kVOLTS[0] + Constants.Drivetrain.kVOLTS[1] + Constants.Drivetrain.kVOLTS[2] + Constants.Drivetrain.kVOLTS[3]);
        double kVelsAVG = 0.25 * (Constants.Drivetrain.kVELS[0] + Constants.Drivetrain.kVELS[1] + Constants.Drivetrain.kVELS[2] + Constants.Drivetrain.kVELS[3]);
        double kAccelAVG = 0.25 * (Constants.Drivetrain.kACCELS[0] + Constants.Drivetrain.kACCELS[1] + Constants.Drivetrain.kACCELS[2] + Constants.Drivetrain.kACCELS[3]);
        DifferentialDriveVoltageConstraint voltConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(kVoltAVG, kVelsAVG, kAccelAVG), dt.getKinematics(), Constants.Drivetrain.kAUTOMAXVOLT);
        config.addConstraint(voltConstraint);
        if (isInverted) { config.setReversed(true); }
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        
        try {
            CSVParser csvParser = CSVFormat.DEFAULT.parse(new FileReader("/home/lvuser/deploy/paths/" + filename + ".path"));
            double x, y, tanx, tany;
            Rotation2d rot;
            //if (csvParser == null) {System.out.println("1");}
            //System.out.println("1");
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
            //System.out.println("2");
            csvParser.close();
        } catch (FileNotFoundException e) {
            System.out.println("File named /home/lvuser/deploy/paths/" + filename + ".path not found.");
            e.printStackTrace();
        }

        //System.out.println("3");
        trajectory = TrajectoryGenerator.generateTrajectory(poses, config);
        this.dt = dt;
        this.isInverted = isInverted;
        //System.out.println("4");
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