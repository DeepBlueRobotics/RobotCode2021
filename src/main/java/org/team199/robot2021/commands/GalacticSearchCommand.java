package org.team199.robot2021.commands;

import frc.robot.lib.Limelight;
import org.team199.robot2021.subsystems.Drivetrain;
import org.team199.robot2021.subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GalacticSearchCommand extends SequentialCommandGroup {
    public GalacticSearchCommand(Drivetrain dt, Intake intake, Limelight lime, double cameraHeight) {
        addRequirements(dt, intake);

        addCommands(
            new ToggleIntake(intake),
            new DriveToBalls(dt, intake, lime, cameraHeight, 3),
            /* Drive to finish command */ new InstantCommand()
        );
    }
}

class DriveToBalls extends CommandBase {
    private Drivetrain dt;
    private Intake intake;
    private Limelight lime;
    private double cameraHeight;
    private int ballsToCollect, ballCounter;

    private ProfiledPIDController thetaController;
    private PIDController xPIDController, yPIDController;

    public DriveToBalls(Drivetrain dt, Intake intake, Limelight lime, double cameraHeight, int ballsToCollect) {
        this.dt = dt;
        this.intake = intake;
        this.lime = lime;
        this.cameraHeight = cameraHeight;
        this.ballsToCollect = ballsToCollect;
        addRequirements(dt, intake);

        ballCounter = 0;

        // Use the same PID Controllers as in RobotPath
        thetaController = new ProfiledPIDController(Constants.DriveConstants.thetaPIDController[0],
                                                    Constants.DriveConstants.thetaPIDController[1],
                                                    Constants.DriveConstants.thetaPIDController[2],
                                                    new Constraints(Double.POSITIVE_INFINITY,
                                                                    Double.POSITIVE_INFINITY));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        xPIDController = new PIDController(Constants.DriveConstants.xPIDController[0],
                                           Constants.DriveConstants.xPIDController[1],
                                           Constants.DriveConstants.xPIDController[2]);
        yPIDController = new PIDController(Constants.DriveConstants.yPIDController[0],
                                           Constants.DriveConstants.yPIDController[1],
                                           Constants.DriveConstants.yPIDController[2]);
    }

    public void execute() {
        // Search for the ball using the limelight
        // Once found, compute distances to the ball and use PID to drive to the ball
        // Check the current draw on the roller motor to see if we have intaked the ball
        // If so, start again from the beginning until all balls have been collected.
        // Otherwise, keep trying to drive toward the ball
    }

    public boolean isFinished() {
        return ballCounter == ballsToCollect;
    }

    public void end(boolean interrupted) {
    }
}