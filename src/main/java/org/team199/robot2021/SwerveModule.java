package org.team199.robot2021;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.lib.swerve.SwerveMath;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class that stores all the variables and methods applicaple to a single swerve module,
 * such as moving, getting encoder values, or configuring PID.
 */
public class SwerveModule {
    public enum ModuleType {FL, FR, BL, BR};

    private ModuleType type;
    private String moduleString;
    private CANSparkMax drive, turn;
    private CANCoder turnEncoder;
    private PIDController drivePIDController, turnPIDController;
    private double driveModifier, maxSpeed, turnZero;
    private boolean reversed;
    private Timer timer;
    private SimpleMotorFeedforward forwardSimpleMotorFF, backwardSimpleMotorFF;

    public SwerveModule(ModuleType type, CANSparkMax drive, CANSparkMax turn, CANCoder turnEncoder, double driveModifier,
                        double maxSpeed, int arrIndex) {
        //SmartDashboard.putNumber("Target Angle (deg)", 0.0);
        this.timer = new Timer();
        timer.start();

        this.type = type;

        switch (type) {
            case FL:
                moduleString = "FL";
                break;
            case FR:
                moduleString = "FR";
                break;
            case BL:
                moduleString = "BL";
                break;
            case BR:
                moduleString = "BR";
                break;
        }

        this.drive = drive;
        drive.getEncoder().setPositionConversionFactor(Constants.DriveConstants.wheelDiameter * Math.PI / Constants.DriveConstants.driveGearing);
        drive.getEncoder().setVelocityConversionFactor(Constants.DriveConstants.wheelDiameter * Math.PI / Constants.DriveConstants.driveGearing / 60);

        this.turn = turn;
        turnPIDController = new PIDController(Constants.DriveConstants.turnkP[arrIndex],
                                              Constants.DriveConstants.turnkI[arrIndex],
                                              Constants.DriveConstants.turnkD[arrIndex]);
        turnPIDController.enableContinuousInput(-180.0, 180.0);

        this.turnEncoder = turnEncoder;
        this.turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        this.driveModifier = driveModifier;
        this.maxSpeed = maxSpeed;
        this.reversed = Constants.DriveConstants.reversed[arrIndex];
        this.turnZero = Constants.DriveConstants.turnZero[arrIndex];

        this.forwardSimpleMotorFF = new SimpleMotorFeedforward(Constants.DriveConstants.kForwardVolts[arrIndex],
                                                               Constants.DriveConstants.kForwardVels[arrIndex],
                                                               Constants.DriveConstants.kForwardAccels[arrIndex]);
        this.backwardSimpleMotorFF = new SimpleMotorFeedforward(Constants.DriveConstants.kBackwardVolts[arrIndex],
                                                                Constants.DriveConstants.kBackwardVels[arrIndex],
                                                                Constants.DriveConstants.kBackwardAccels[arrIndex]);
        drivePIDController = new PIDController(Constants.DriveConstants.drivekP[arrIndex],
                                               Constants.DriveConstants.drivekI[arrIndex],
                                               Constants.DriveConstants.drivekD[arrIndex]);
    }

    public void periodic() {
        if (!turnPIDController.atSetpoint()) {
            turn.set(MathUtil.clamp(turnPIDController.calculate(turnEncoder.getPosition()), -1.0, 1.0));
        }
    }

    /**
     * Move the module to a specified angle and drive at a specified speed.
     * @param normalizedSpeed   The desired speed normalized with respect to a maximum speed, in m/s.
     * @param angle             The desired angle, in radians.
     */
    public void move(double normalizedSpeed, double angle) {
        //SmartDashboard.putNumber(moduleString + " Compute Setpoints Angle:", angle / (2 * Math.PI));
        SmartDashboard.putNumber(moduleString + " Target Angle (deg)", 180 * angle / Math.PI);
        double setpoints[] = SwerveMath.computeSetpoints(normalizedSpeed / maxSpeed,
                                                         angle / (2 * Math.PI),
                                                         turnEncoder.getPosition(),
                                                         360.0);
        //SmartDashboard.putNumber(moduleString + " Setpoint Result: ", setpoints[1] * 360.0D);
        setSpeed(setpoints[0]);
        if(setpoints[0] != 0.0) setAngle(setpoints[1]);
    }

    /**
     * Sets the speed for the drive motor controller.
     * @param speed     The desired speed, from -1.0 (maximum speed directed backwards) to 1.0 (maximum speed directed forwards).
     */
    private void setSpeed(double speed) {
        // Get the change in time (for acceleration limiting calculations)
        double deltaTime = timer.get();

        // Compute desired and actual speeds in m/s
        double desiredSpeed = maxSpeed * speed * driveModifier;
        double actualSpeed = getCurrentSpeed();

        // Calculate acceleration and limit it if greater than maximum acceleration (without slippage and with sufficient motors).
        double desiredAcceleration = (desiredSpeed - actualSpeed) / deltaTime;
        double maxAcceleration = Constants.DriveConstants.mu * 9.8;
        double clippedAcceleration = Math.copySign(Math.min(Math.abs(desiredAcceleration), maxAcceleration), desiredAcceleration);

        // Clip the speed based on the clipped desired acceleration
        //double clippedDesiredSpeed = actualSpeed + clippedAcceleration * deltaTime;

        // Use robot characterization as a simple physical model to account for internal resistance, frcition, etc.
        double appliedVoltage = (desiredSpeed >= 0 ? forwardSimpleMotorFF :
        // Add a PID adjustment for error correction (also "drives" the actual speed to the desired speed)
        appliedVoltage += drivePIDController.calculate(actualSpeed, desiredSpeed);
        appliedVoltage = MathUtil.clamp(appliedVoltage / 12, -1, 1);
        drive.set(appliedVoltage);

        // Reset the timer so get() returns a change in time
        timer.reset();
        timer.start();
    }

    /**
     * Sets the angle for the turn motor controller.
     * @param angle     The desired angle, between -0.5 (180 degrees counterclockwise) and 0.5 (180 degrees clockwise).
     */
    private void setAngle(double angle) {
        //SmartDashboard.putNumber(moduleString + "Target Angle:", 360 * angle * (reversed ? -1 : 1));
        turnPIDController.setSetpoint(360 * angle * (reversed ? -1 : 1));
    }

    private double getModuleAngle() {
        return Math.PI * turnEncoder.getAbsolutePosition() / 180;
    }

    /**
     * Gets the current state (speed and angle) of this module.
     * @return A SwerveModuleState object representing the speed and angle of the module.
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getCurrentSpeed(), new Rotation2d(getModuleAngle()));
    }

    public double getCurrentSpeed() {
        return drive.getEncoder().getVelocity();
    }

    /**
     * Updates SmartDashboard with information about this module.
     */
    public void updateSmartDashboard() {
        // Display the position of the quadrature encoder.
        SmartDashboard.putNumber(moduleString + " Incremental Position", turnEncoder.getPosition());
        // Display the position of the analog encoder.
        SmartDashboard.putNumber(moduleString + " Angle (degrees)", turnEncoder.getAbsolutePosition());
        // Display the module angle as calculated using the absolute encoder.
        SmartDashboard.putNumber(moduleString + " Angle (radians)", getModuleAngle());
        // Display the speed that the robot thinks it is travelling at.
        SmartDashboard.putNumber(moduleString + " Current Speed", getCurrentSpeed());
        SmartDashboard.putNumber(moduleString + " Setpoint Angle", turnPIDController.getSetpoint());
    }

    /**
     * HomeAbsolute is an instant command that ensures that each of the turn motor controllers are in a known configuration,
     * as dictated by the absolute encoder positions turnZero.
     */
    public void homeAbsolute() {
        // Set the position of the quadrature encoder to the position measured by the CANCoder relative to the zeroed absolute position
        turnEncoder.setPosition((turnEncoder.getAbsolutePosition() - turnZero));
        // Ensure that we turn to this angle
        setAngle(0.0);
    }

    public void toggleMode() {
        if (drive.getIdleMode() == IdleMode.kBrake) coast();
        else brake();
    }

    public void brake() {
        drive.setIdleMode(IdleMode.kBrake);
    }

    public void coast() {
        if (DriverStation.getInstance().isDisabled()) drive.setIdleMode(IdleMode.kCoast);
        else drive.setIdleMode(IdleMode.kBrake);
    }
}
