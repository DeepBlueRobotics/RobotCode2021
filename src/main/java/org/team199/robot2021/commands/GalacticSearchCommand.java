package org.team199.robot2021.commands;

import frc.robot.lib.Limelight;

import org.team199.robot2021.Constants;
import org.team199.robot2021.subsystems.Drivetrain;
import org.team199.robot2021.subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GalacticSearchCommand extends SequentialCommandGroup {
    public GalacticSearchCommand(Drivetrain dt, Intake intake, Limelight lime, double cameraHeight) {
        addRequirements(dt, intake);

        Rotation2d heading = Rotation2d.fromDegrees(dt.getHeading());
        addCommands(
            new InstantCommand(() -> dt.setOdometry(new SwerveDriveOdometry(dt.getKinematics(), heading, new Pose2d(0, 0, heading)))),
            new ToggleIntake(intake),
            new DriveToBalls(dt, intake, lime, cameraHeight, 3),
            // It is best to set the x-component to be farther than it needs to be
            new DriveToEnd(dt, new Pose2d(Units.inchesToMeters(300), dt.getOdometry().getPoseMeters().getTranslation().getY(), new Rotation2d()))
        );
    }
}

class DriveToBalls extends CommandBase {
    private Drivetrain dt;
    private Intake intake;
    private Limelight lime;
    private double cameraHeight;
    private int ballsToCollect, ballCounter, mode;

    private ProfiledPIDController thetaController;
    private PIDController xPIDController, yPIDController;

    private PowerDistributionPanel pdp;
    private Timer timer = new Timer();
    public DriveToBalls(Drivetrain dt, Intake intake, Limelight lime, double cameraHeight, int ballsToCollect) {
        this.dt = dt;
        this.intake = intake;
        this.lime = lime;
        this.cameraHeight = cameraHeight;
        this.ballsToCollect = ballsToCollect;
        addRequirements(dt, intake);
        SmartDashboard.putBoolean("Field Oriented", false);
        ballCounter = 0;

        // Use the same PID Controllers as in RobotPath
        thetaController = new ProfiledPIDController(.005, 0, 0,
                                                    new Constraints(Double.POSITIVE_INFINITY,
                                                                    Double.POSITIVE_INFINITY));
        thetaController.enableContinuousInput(-180, 180);
        thetaController.setGoal(0);

        xPIDController = new PIDController(.15, 0, 0);
        xPIDController.setSetpoint(0);
        yPIDController = new PIDController(.15, 0, 0);
        yPIDController.setSetpoint(0);

        pdp = new PowerDistributionPanel(Constants.DrivePorts.kPDPCANPort);

        // mode = 0 means searching for the ball, mode = 1 means driving toward the ball
        mode = 0;
    }

    // One flaw with this command is that there is no end condition for if the robot never finds a ball.
    public void execute() {
        SmartDashboard.putNumber("Ball search mode: ", mode);
        // Search for the ball using the limelight
        if (mode == 0) {
            double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
            double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
            if (tv == 1.0 && ta > SmartDashboard.getNumber("Area Threshold", 0.02)) 
                mode = 1;
            
            // lime.steeringAssist() outputs positive adjustment values for when we need to rotate clockwise.
            // dt.drive() has counter-clockwise rotation = +, clockwise rotation = -
            dt.drive(0, 0, Constants.DriveConstants.maxRCW/3 /* 1 * lime.steeringAssist(dt.getHeading())/3*/);
        } else {
            // Check the current draw on the roller motor to see if we have intaked the ball
            //if (pdp.getCurrent(Constants.DrivePorts.kIntakeRollerPDP) > Constants.DriveConstants.intakeCurrentDraw) {
                // If so, start again from the beginning until all balls have been collected.
            //    mode = 0;
            //    ballCounter++;
            //}

            // Compute distances to the ball and use PID to drive to the ball
            double[] setpoints = lime.determineObjectDist(cameraHeight, 0.0, Constants.DriveConstants.cameraMountingAngleDeg);
            double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
            dt.drive(-Constants.DriveConstants.maxForward/2 * MathUtil.clamp(xPIDController.calculate(setpoints[0]), -1.0, 1.0),
                     -Constants.DriveConstants.maxStrafe/2 * MathUtil.clamp(yPIDController.calculate(setpoints[1]), -1.0, 1.0),
                     //0,0,
                     -Constants.DriveConstants.maxRCW * MathUtil.clamp(thetaController.calculate(tx), -1.0, 1.0));
             double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
             double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
             timer.start();
             if (tv == 0.0 && ta < SmartDashboard.getNumber("Area Threshold", 0.02)) {
                 if (timer.hasElapsed(1)){
                    mode = 0;
                    ballCounter++;
                 }
             } else {
                 timer.reset();
             }

        }
    }

    public boolean isFinished() {
        return ballCounter == ballsToCollect;
    }

    public void end(boolean interrupted) {
    }
}

class DriveToEnd extends CommandBase {
    private Drivetrain dt;
    private HolonomicDriveController controller;
    private Pose2d endZone;

    // endZone is the pose of the end zone relative to the robot's initial starting pose
    public DriveToEnd(Drivetrain dt, Pose2d endZone) {
        SmartDashboard.putBoolean("Field Oriented", true);
        addRequirements(this.dt = dt);
        this.endZone = endZone;
        ProfiledPIDController thetaController = new ProfiledPIDController(Constants.DriveConstants.thetaPIDController[0],
                                                                          Constants.DriveConstants.thetaPIDController[1],
                                                                          Constants.DriveConstants.thetaPIDController[2],
                                                                          new Constraints(Double.POSITIVE_INFINITY,
                                                                                          Double.POSITIVE_INFINITY));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController xPIDController = new PIDController(Constants.DriveConstants.xPIDController[0],
                                                         Constants.DriveConstants.xPIDController[1],
                                                         Constants.DriveConstants.xPIDController[2]);
        PIDController yPIDController = new PIDController(Constants.DriveConstants.yPIDController[0],
                                                         Constants.DriveConstants.yPIDController[1],
                                                         Constants.DriveConstants.yPIDController[2]);
        controller = new HolonomicDriveController(xPIDController, yPIDController, thetaController);
    }

    public void execute() {
        ChassisSpeeds speeds = controller.calculate(dt.getOdometry().getPoseMeters(), endZone, 
                                                    Constants.DriveConstants.autoMaxSpeed, Rotation2d.fromDegrees(dt.getHeading()));
        dt.drive(dt.getKinematics().toSwerveModuleStates(speeds));                                            
    }

    public boolean isFinished() {
        return (dt.getOdometry().getPoseMeters().getTranslation().getX() > endZone.getTranslation().getX());
    }

    public void end(boolean interrupted) {
    }
}