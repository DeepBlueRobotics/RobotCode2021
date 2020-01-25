package org.team199.robot2020.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

public class Climber {
    private final CANSparkMax liftRobotMotor = MotorControllerFactory.createSparkMax(Constants.Climber.LIFT_ROBOT_MOTOR);
    private final VictorSPX liftMotor = MotorControllerFactory.createVictor(Constants.Climber.LIFT_MOTOR);
    private final Encoder liftEncoder = new Encoder(4, 5); // TODO: set correct values
    private final Encoder robotEncoder = new Encoder(6, 7);

    public Climber(){
        liftMotor.setNeutralMode(NeutralMode.Brake); // to assist keeping lift up
        liftRobotMotor.setIdleMode(CANSparkMax.IdleMode.kCoast); // hook can be passively pulled up
        liftEncoder.setDistancePerPulse(1.0 / 256); // TODO: set unit to inches
        robotEncoder.setDistancePerPulse(1.0 / 256); // TODO: set convenient rate
        liftEncoder.reset();
        robotEncoder.reset();
        /*
        Couldn't find a method to set encoder position, so robotEncoder starts at 0
        Add the initial winding distance to the final height
        */
    }

    public void raiseLift(){
        liftMotor.set(ControlMode.PercentOutput, Constants.Climber.AUTO_LIFT_SPEED);
    }

    public void lowerLift(double speed){
        liftMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopLift(){
        liftMotor.set(ControlMode.PercentOutput, 0);
    }

    public void raiseRobot() {
        liftRobotMotor.set(Constants.Climber.ROBOT_SPEED);
    }

    public void stopRobot() {
        liftRobotMotor.set(0);
    }

    public void reset() {
        liftRobotMotor.set(Constants.Climber.RESET_SPEED);
    }

    public double getLiftHeight() {
        return liftEncoder.getDistance();
    }

    public double getRobotHeight() {
        return robotEncoder.getDistance();
    }
}