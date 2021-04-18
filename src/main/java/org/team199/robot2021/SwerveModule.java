package org.team199.robot2021;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.lib.swerve.SwerveMath;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

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
    private PIDController drivePIDController;
    private ProfiledPIDController turnPIDController;
    private TrapezoidProfile.Constraints turnConstraints;
    private double driveModifier, maxSpeed, turnZero;
    private Supplier<Float> pitchDegSupplier, rollDegSupplier;
    private boolean reversed;
    private Timer timer;
    private SimpleMotorFeedforward forwardSimpleMotorFF, backwardSimpleMotorFF, turnSimpleMotorFeedforward;
    private double lastAngle, maxTurnVelocity;

    public SwerveModule(ModuleType type, CANSparkMax drive, CANSparkMax turn, CANCoder turnEncoder, double driveModifier,
                        double maxSpeed, int arrIndex, Supplier<Float> pitchDegSupplier, Supplier<Float> rollDegSupplier) {
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
        double positionConstant = Constants.DriveConstants.wheelDiameter * Math.PI / Constants.DriveConstants.driveGearing;
        drive.setInverted(Constants.DriveConstants.driveInversion[arrIndex]);
        drive.getEncoder().setPositionConversionFactor(positionConstant);
        drive.getEncoder().setVelocityConversionFactor(positionConstant / 60);

        this.forwardSimpleMotorFF = new SimpleMotorFeedforward(Constants.DriveConstants.kForwardVolts[arrIndex],
                                                               Constants.DriveConstants.kForwardVels[arrIndex],
                                                               Constants.DriveConstants.kForwardAccels[arrIndex]);
        this.backwardSimpleMotorFF = new SimpleMotorFeedforward(Constants.DriveConstants.kBackwardVolts[arrIndex],
                                                                Constants.DriveConstants.kBackwardVels[arrIndex],
                                                                Constants.DriveConstants.kBackwardAccels[arrIndex]);

        drivePIDController = new PIDController(2 * Constants.DriveConstants.drivekP[arrIndex],
                                               Constants.DriveConstants.drivekI[arrIndex],
                                               Constants.DriveConstants.drivekD[arrIndex]);
    

        //System.out.println("Velocity Constant: " + (positionConstant / 60));

        this.turn = turn;

        this.turnSimpleMotorFeedforward = new SimpleMotorFeedforward(Constants.DriveConstants.turnkS[arrIndex],
                                                                     Constants.DriveConstants.turnkV[arrIndex],
                                                                     Constants.DriveConstants.turnkA[arrIndex]);

        // Voltage = kS + kV * velocity + kA * acceleration
        // Assume cruising at maximum velocity --> 12 = kS + kV * max velocity + kA * 0 --> max velocity = (12 - kS) / kV
        // Limit the velocity by a factor of 0.5
        double maxTurnVelocity = 0.5 * turnSimpleMotorFeedforward.maxAchievableVelocity(12.0, 0);
        // Assume accelerating while at limited speed --> 12 = kS + kV * limited speed + kA * acceleration
        // acceleration = (12 - kS - kV * limiedSpeed) / kA
        double turnAccelrationConstraint = turnSimpleMotorFeedforward.maxAchievableAcceleration(12.0, maxTurnVelocity);
        turnConstraints = new TrapezoidProfile.Constraints(maxTurnVelocity, turnAccelrationConstraint);
        lastAngle = 0.0;
        //SmartDashboard.putNumber("turn velocity constraint", turnConstraints.maxVelocity);
        //SmartDashboard.putNumber("turn accel constraint", turnConstraints.maxAcceleration);
        turnPIDController = new ProfiledPIDController(Constants.DriveConstants.turnkP[arrIndex] * 1.0,
                                              Constants.DriveConstants.turnkI[arrIndex],
                                              Constants.DriveConstants.turnkD[arrIndex],
                                              turnConstraints);
        turnPIDController.enableContinuousInput(-180.0, 180.0);
        turnPIDController.setTolerance(3 * 360/4096.0);
        turnPIDController.reset(0);

        this.turnEncoder = turnEncoder;
        this.turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        this.driveModifier = driveModifier;
        this.maxSpeed = maxSpeed;
        this.reversed = Constants.DriveConstants.reversed[arrIndex];
        this.turnZero = Constants.DriveConstants.turnZero[arrIndex];

        this.rollDegSupplier = rollDegSupplier;
        this.pitchDegSupplier = pitchDegSupplier;
    }

    public void periodic() {
        double turnSpeed = turnPIDController.calculate(getModuleAngle()*180/Math.PI);
        TrapezoidProfile.State state = turnPIDController.getSetpoint();
        SmartDashboard.putNumber(moduleString + "state position", state.position);
        SmartDashboard.putNumber(moduleString + "state velocity", state.velocity);
        turnSpeed += turnSimpleMotorFeedforward.calculate(state.velocity) / 12.0;
        if (!turnPIDController.atGoal()) {
            turn.set(MathUtil.clamp(turnSpeed, -1.0, 1.0));
            //System.out.println("stuff is being turned");
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
        // if(setpoints[0] != 0.0) 
            setAngle(setpoints[1]);
    }

    /**
     * Calculates the amount of additional forward accelration needed to combat gravity
     * @param gyroPitchDeg Pitch of gyro in degrees
     * @param gyroRollDeg  Roll of gyro in degrees
     * @return the amount of additional forward accelration needed to combat gravity in m/s^2
     */
    private double calculateAntiGravitationalA(Float gyroPitchDeg, Float gyroRollDeg)
    {
        double moduleAngle = getModuleAngle();
        double moduleRollComponent = Math.sin(moduleAngle);
        double modulePitchComponent = Math.cos(moduleAngle);
        // gravitationalA is estimated to work for small angles, not 100% accurate at large angles
        double antiGravitationalA = Constants.g * (modulePitchComponent * Math.sin(Math.PI * gyroPitchDeg / 180) - moduleRollComponent * Math.sin(Math.PI * gyroRollDeg / 180));
        SmartDashboard.putNumber("AntiGravitational accelration", antiGravitationalA);
        return antiGravitationalA;
    }
    /**
     * Sets the speed for the drive motor controller.
     * @param speed     The desired speed, from -1.0 (maximum speed directed backwards) to 1.0 (maximum speed directed forwards).
     */
    private void setSpeed(double speed) {
        // Get the change in time (for acceleration limiting calculations)
        
        SmartDashboard.putNumber(moduleString + " Desired Speed (unitless)", speed);

        // Compute desired and actual speeds in m/s
        double desiredSpeed = maxSpeed * speed * driveModifier;
        double actualSpeed = getCurrentSpeed();
        SmartDashboard.putNumber(moduleString + " Desired Speed (mps)", desiredSpeed);
        SmartDashboard.putNumber(moduleString + " Actual Speed (mps)", actualSpeed);
        double targetVoltage = (actualSpeed >= 0 ? forwardSimpleMotorFF :
                                 backwardSimpleMotorFF).calculate(desiredSpeed, calculateAntiGravitationalA(pitchDegSupplier.get(), rollDegSupplier.get()));//clippedAcceleration);
        /* double holdingVoltage = (actualSpeed >= 0 ? forwardSimpleMotorFF :
                                 backwardSimpleMotorFF).calculate(actualSpeed, calculateAntiGravitationalA(pitchDegSupplier.get(), rollDegSupplier.get()));//clippedAcceleration);
        */
        // Calculate acceleration and limit it if greater than maximum acceleration (without slippage and with sufficient motors).
        //double desiredAcceleration = (desiredSpeed - actualSpeed) / deltaTime;// + calculateAntiGravitationalA(pitchDegSupplier.get(), rollDegSupplier.get());
        //double maxAcceleration = Constants.DriveConstants.mu * Constants.g;
        //double maxVoltageDifference = maxAcceleration*(actualSpeed >= 0 ? forwardSimpleMotorFF.ka : backwardSimpleMotorFF.ka);
        double clippedVoltage =  targetVoltage; //MathUtil.clamp(targetVoltage, holdingVoltage - maxVoltageDifference, holdingVoltage + maxVoltageDifference);
        //SmartDashboard.putNumber(moduleString + " Clipped Acceleration", clippedAcceleration);
        //clippedAcceleration = 0;
        // Clip the speed based on the clipped desired acceleration
        //double clippedDesiredSpeed = actualSpeed + clippedAcceleration * deltaTime;
        //SmartDashboard.putNumber(moduleString + " Clipped Desired Speed", clippedDesiredSpeed);
        
        // Use robot characterization as a simple physical model to account for internal resistance, frcition, etc.
        // Add a PID adjustment for error correction (also "drives" the actual speed to the desired speed)
        //SmartDashboard.putNumber(moduleString + " Voltage no PID", appliedVoltage);
        clippedVoltage += drivePIDController.calculate(actualSpeed, desiredSpeed);
        //SmartDashboard.putNumber(moduleString + " Unclamped Voltage", appliedVoltage);
        double appliedVoltage = MathUtil.clamp(clippedVoltage / 12, -1, 1);
        //SmartDashboard.putNumber(moduleString + " Drive Speed", driveModifier * appliedVoltage);
        drive.set(appliedVoltage);

        // Reset the timer so get() returns a change in time
    }

    /**
     * Sets the angle for the turn motor controller.
     * @param angle     The desired angle, between -0.5 (180 degrees counterclockwise) and 0.5 (180 degrees clockwise).
     */
    private void setAngle(double angle) {
        //SmartDashboard.putNumber(moduleString + "Target Angle:", 360 * angle * (reversed ? -1 : 1));

        double deltaTime = timer.get();
        timer.reset();
        timer.start();
        // Omega is the angular velocity at which the module is turning.
        // v = omega * r (for small changes in module angle)
        // a = v^2 / r --> r = v * v / a --> r = omega * r * v / a --> 1 = omega * v / a
        // omega = a / v
        double maxOmega = Constants.DriveConstants.autoCentripetalAccel / getCurrentSpeed();
        // Omega is in radians so it must be converted to degrees. Also, we don't want to
        // turn faster than what we are capable of (maxTurnVelocity)
        turnConstraints.maxVelocity = Math.min(maxOmega * 180 / Math.PI, maxTurnVelocity);
		
        // Find the minimum distance to travel from lastAngle to angle and determine the
        // correct direction to trvel the minimum distance. This is used in order to accurately
        // calculate the goal velocity.
        double angleDiff = (360 * Math.abs(angle - lastAngle)) % 360;
        double optimalDiff = Math.min(angleDiff, 360 - angleDiff);
        double s1 = Math.signum(angle);
        double s2 = Math.signum(lastAngle);
        if (s1 != s2) {
	        if (optimalDiff == 360 - angleDiff) optimalDiff *= s2;
	        else optimalDiff *= s1;
        } else optimalDiff *= Math.signum(angle - lastAngle);

        turnPIDController.setGoal(new TrapezoidProfile.State(360 * angle * (reversed ? -1 : 1), optimalDiff / deltaTime));
        lastAngle = angle;
    }

    private double getModuleAngle() {
        return Math.PI * (turnEncoder.getAbsolutePosition() - turnZero) / 180;
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
        SmartDashboard.putNumber(moduleString + " Absolute Angle (degrees)", turnEncoder.getAbsolutePosition());
        // Display the module angle as calculated using the absolute encoder.
        SmartDashboard.putNumber(moduleString + " Relative Angle (degrees)", getModuleAngle()/Math.PI*180);
        SmartDashboard.putNumber(moduleString + " Encoder Position", drive.getEncoder().getPosition());
        // Display the speed that the robot thinks it is travelling at.
        SmartDashboard.putNumber(moduleString + " Current Speed", getCurrentSpeed());
        SmartDashboard.putNumber(moduleString + " Setpoint Angle", turnPIDController.getGoal().position);
        //SmartDashboard.putNumber("Gyro Pitch", pitchDegSupplier.get());
        //SmartDashboard.putNumber("Gyro Roll", rollDegSupplier.get());
        SmartDashboard.putNumber(moduleString + "Antigravitational Acceleration", calculateAntiGravitationalA(pitchDegSupplier.get(), rollDegSupplier.get()));
        SmartDashboard.putBoolean(moduleString + " Turn is at Goal", turnPIDController.atGoal());
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
