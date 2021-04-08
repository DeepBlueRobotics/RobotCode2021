package org.team199.robot2021.commands;

import frc.robot.lib.Limelight;

import java.io.IOException;

import org.team199.lib.RobotPath;
import org.team199.robot2021.Constants;
import org.team199.robot2021.subsystems.Drivetrain;
import org.team199.robot2021.subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableInstance;

public class GalacticSearchCommand extends CommandBase {
    private Drivetrain dt;
    private Intake intake;
    private Limelight lime;
    private int path;
    private Command pathCommand;

    public GalacticSearchCommand(Drivetrain dt, Intake intake, Limelight lime) {
        this.dt = dt;
        this.intake = intake;
        this.lime = lime;
        addRequirements(dt, intake);
    }

    public void initialize(){
        double[] distComponents = lime.determineObjectDist(Constants.DriveConstants.cameraHeight, 0.0, Constants.DriveConstants.cameraMountingAngleDeg);
        double dist = Math.hypot(distComponents[0], distComponents[1]);
        

        //dist val interpretting -> path index

        path = getPathIndex(dist);

        SmartDashboard.putString("Chosen Path", Constants.GameConstants.GSPaths[path]);

        try {
            pathCommand = new RobotPath(Constants.GameConstants.GSPaths[path], dt, intake, true, false, Constants.DriveConstants.autoMaxSpeed).getPathCommand(false);
        } catch(IOException e) {
            e.printStackTrace();
        }
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
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        if(ty < Constants.GameConstants.GSMidPoints[0]){
            if (tx < Constants.GameConstants.GSMidPoints[1]) {
                return 3;
            } else {
                return 0;
            }
        }else {
            if (tx < Constants.GameConstants.GSMidPoints[2]) {
                return 1;
            }else {
                return 2;
            }

        }
    }
}