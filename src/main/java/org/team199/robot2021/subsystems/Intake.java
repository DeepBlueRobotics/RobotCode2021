package org.team199.robot2021.subsystems;

import com.revrobotics.CANSparkMax;

import org.team199.robot2021.Constants;

//import frc.robot.lib.MotorControllerFactory;
//import org.team199.robot2021.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Intake extends SubsystemBase {
    // TODO: find good values and then set to final
    private static double kIntakeSpeed = .6;
    private static double kSlowSpeed = -0.1;
    private static double kLimeBottomAngleDegs = 0; //TODO: set correct angle
    private static double kLimeTopAngleDegs = 0; //TODO: set correct angle

    private final CANSparkMax rollerMotor = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kIntakeRoller);
    private final Servo limelightServo = new Servo(Constants.DrivePorts.kLimelightServoPort); 
    private final DoubleSolenoid intakePistons1 = new DoubleSolenoid(Constants.DrivePorts.kIntakePistons[2], Constants.DrivePorts.kIntakePistons[3]);
    private final DoubleSolenoid intakePistons2 = new DoubleSolenoid(Constants.DrivePorts.kIntakePistons[0], Constants.DrivePorts.kIntakePistons[1]);

    private boolean deployed = false;
    private boolean isLimelightSearching = false;

    /**
     * Vectored intake that rolls balls through the bumper gap and into feeder.
     */
    public Intake() {
        rollerMotor.setInverted(true);
        rollerMotor.setSmartCurrentLimit(30);

        SmartDashboard.putNumber("Intake.kIntakeSpeed", kIntakeSpeed);
        SmartDashboard.putNumber("Intake.kSlowSpeed", kSlowSpeed);
    }

    public void periodic() {
        kIntakeSpeed = SmartDashboard.getNumber("Intake.kIntakeSpeed", kIntakeSpeed);
        kSlowSpeed = SmartDashboard.getNumber("Intake.kSlowSpeed", kSlowSpeed);
    }

    public void slow() {
        rollerMotor.set(kSlowSpeed);
    }

    public void intake() {
        rollerMotor.set(kIntakeSpeed);
    }

    public void outtake() {
        rollerMotor.set(-kIntakeSpeed);
    }

    public void stop() {
        rollerMotor.set(0);
    }

    public void deploy() {
        intakePistons1.set(DoubleSolenoid.Value.kReverse);
        intakePistons2.set(DoubleSolenoid.Value.kForward);
        deployed = true;
    }

    public void retract() {
        intakePistons1.set(DoubleSolenoid.Value.kForward);
        intakePistons2.set(DoubleSolenoid.Value.kReverse);
        deployed = false;
    }

    // for if we want to try making our intake less rigid
    public void doTheFlop() {
        intakePistons1.set(DoubleSolenoid.Value.kReverse);
        intakePistons2.set(DoubleSolenoid.Value.kReverse);
        deployed = true;
    }

    public boolean isDeployed() {
        return deployed;
    }
    public boolean isLimelightSearching() {
        return isLimelightSearching;
    }
    public void setLimelightSearching(boolean status) {
        isLimelightSearching = status;
        if (isLimelightSearching) {
            limelightServo.setAngle(kLimeBottomAngleDegs); // TODO: set correct angle
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
        } else {
            limelightServo.setAngle(kLimeTopAngleDegs); // TODO: set correct angle
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

        }

    }


}