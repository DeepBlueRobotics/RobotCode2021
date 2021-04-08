package org.team199.robot2021.commands;

import frc.robot.lib.Limelight;
import jdk.jfr.internal.tool.Command;

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
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

class GalacticSearchCommand extends CommandBase {
    private Drivetrain dt;
    private Intake intake;
    private Limelight lime;
    private double cameraHeight;
    private int path;
    private Command pathCommand;

    public DriveToBalls(Drivetrain dt, Intake intake, Limelight lime, double cameraHeight) {
        this.dt = dt;
        this.intake = intake;
        this.lime = lime;
        this.cameraHeight = cameraHeight;
        addRequirements(dt, intake);

        // Use the same PID Controllers as in RobotPath
        thetaController = new ProfiledPIDController(4, 0, 0,
                                                    new Constraints(Double.POSITIVE_INFINITY,
                                                                    Double.POSITIVE_INFINITY));
        thetaController.enableContinuousInput(-180, 180);
        thetaController.setGoal(0);

        xPIDController = new PIDController(4, 0, 0);
        xPIDController.setSetpoint(0);
        yPIDController = new PIDController(4, 0, 0);
        yPIDController.setSetpoint(0);

        pdp = new PowerDistributionPanel(Constants.DrivePorts.kPDPCANPort);
    }

    public void initialize(){
        double[] distComponents = lime.determineObjectDist(cameraHeight, 0.0);
        double dist = Math.hypot(distComponents[0], distComponents[1]);

        //dist val interpretting -> path index

        path = getPathIndex(dist);

        pathCommand = new RobotPath(Constants.GameConstants.GSPaths[path], dt, intake, true, false, Constants.autoMaxSpeed).getPathCommand();
        pathCommand.initialize();
    }

    // One flaw with this command is that there is no end condition for if the robot never finds a ball.
    public void execute() {
        pathCommand.execute();
    }

    public boolean isFinished() {
        return pathCommand.isFinished();
    }

    public void end(boolean interrupted) {
        pathCommand.end(interrupted);
    }

    private int getPathIndex(double dist){
        if(dist < Constant.GameConstants.GSMidPoints[0]){
            return 0;
        }else if(dist > Constant.GameConstants.GSMidPoints[0] && dist < Constant.GameConstants.GSMidPoints[1]){
            return 1;
        }else if(dist > Constant.GameConstants.GSMidPoints[1] && dist < Constant.GameConstants.GSMidPoints[2]){
            return 2;
        }else{
            return 3;
        }
    }
}