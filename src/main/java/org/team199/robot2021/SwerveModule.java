package org.team199.robot2021;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.lib.SwerveMath;
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
    private PIDController turnPIDController;
    private double driveModifier, maxSpeed;
    private int turnZero;
    private boolean reversed;
    private Timer timer;
    private SimpleMotorFeedforward simpleMotorFeedforward;

    public SwerveModule(ModuleType type, CANSparkMax drive, CANSparkMax turn, CANCoder turnEncoder, double driveModifier, 
                        double maxSpeed, boolean reversed, int turnZero, double kVolt) {
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
        
        this.turn = turn;
        turnPIDController = new PIDController(0.01, 0.0, 0.0);

        this.turnEncoder = turnEncoder;
        this.turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        this.driveModifier = driveModifier;
        this.maxSpeed = maxSpeed;
        this.reversed = reversed;
        this.turnZero = turnZero;

        this.simpleMotorFeedforward = new SimpleMotorFeedforward(kVolt, 1 / maxSpeed);
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
        double setpoints[] = SwerveMath.computeSetpoints(normalizedSpeed / maxSpeed,
                                                         -angle / (2 * Math.PI),
                                                         turnEncoder.getPosition(),
                                                         1.0);
        setSpeed(setpoints[0]);
        if(setpoints[0] != 0.0) setAngle(setpoints[1]);
    }

    /**
     * Sets the speed for the drive motor controller.
     * @param speed     The desired speed, from -1.0 (maximum speed directed backwards) to 1.0 (maximum speed directed forwards).
     */
    private void setSpeed(double speed) {
        double deltaTime = timer.get();

        double desiredSpeed = maxSpeed * speed * Math.abs(driveModifier);

        // Calculate acceleration and limit it if greater than maximum acceleration (without slippage and with sufficient motors).
        double desiredAcceleration = (desiredSpeed - getCurrentSpeed()) / deltaTime;
        double maxAcceleration = Constants.DriveConstants.mu * 9.8;
        double clippedAcceleration = Math.copySign(Math.min(Math.abs(desiredAcceleration), maxAcceleration), desiredAcceleration);
        
        double clippedDesiredSpeed = getCurrentSpeed() + clippedAcceleration * deltaTime;
        double appliedVoltage = simpleMotorFeedforward.calculate(clippedDesiredSpeed); //percent of system (12v)

        // Reset the timer so get() returns a change in time
        timer.reset();
        timer.start();
        SmartDashboard.putNumber(moduleString + " Expected Speed", desiredSpeed);
        SmartDashboard.putNumber(moduleString + " Clipped Acceleration", clippedAcceleration);
        SmartDashboard.putNumber(moduleString + " Clipped Speed", clippedDesiredSpeed);
        
        drive.set(driveModifier * appliedVoltage);
    }

    /**
     * Sets the angle for the turn motor controller.
     * @param angle     The desired angle, between -0.5 (180 degrees counterclockwise) and 0.5 (180 degrees clockwise).
     */
    private void setAngle(double angle) {
        turnPIDController.setSetpoint(180 * angle * (reversed ? -1 : 1));
    }

    /**
     * Sets the PID constants for the turn motor controller.
     * @param kP        The proportionality constant for the "P" (proportional) term.
     * @param kI        The proportionality constant for the "I" (integral) term.
     * @param kD        The proportionality constant for the "D" (derivative) term.
     */
    public void setTurnPID(double kP, double kI, double kD) {
        turnPIDController.setP(kP);
        turnPIDController.setI(kI);
        turnPIDController.setD(kD);
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
        SmartDashboard.putNumber(moduleString + " Analog Position", turnEncoder.getAbsolutePosition());
        // Display the module angle as calculated using the absolute encoder.
        SmartDashboard.putNumber(moduleString + " Module Angle", getModuleAngle());
        // Display the speed that the robot thinks it is travelling at.
        SmartDashboard.putNumber(moduleString + " Current Speed", getCurrentSpeed());
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
