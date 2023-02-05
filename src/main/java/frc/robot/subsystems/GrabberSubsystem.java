package frc.robot.subsystems;

import frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
  
public class GrabberSubsystem extends SubsystemBase {
    private WPI_TalonFX grabberMotor;

    private double thresh;
    private PIDController pidController;
    private SimpleMotorFeedforward feedForward;

    public GrabberSubsystem() {
        grabberMotor = new WPI_TalonFX(Grabber.kMotorID);
        grabberMotor.configFactoryDefault();
        thresh = Grabber.kThreshold;

        pidController = new PIDController(Grabber.kP, Grabber.kI, Grabber.kD);
        feedForward = new SimpleMotorFeedforward(Grabber.kS, Grabber.kV, Grabber.kA);
    }

    public void grab() {
        grabberMotor.setVoltage(Grabber.kVoltage);
    }

    public void stop() {
        grabberMotor.setVoltage(0);
    }

    public void reverse() {
        grabberMotor.setInverted(true);
        grabberMotor.setVoltage(Grabber.kVoltage);
    }

    
    
         
    }
