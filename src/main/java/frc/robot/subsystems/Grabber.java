package frc.robot.subsystems;

import frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
  
public class Grabber extends SubsystemBase {
    private WPI_TalonFX grabberMotor;

    private double thresh;
    private PIDController pidController;
    private SimpleMotorFeedforward feedForward;

    public Grabber() {
        grabberMotor = new WPI_TalonFX(Grab.kGrabberMotorID);
        grabberMotor.configFactoryDefault();
        thresh = Grab.kThresh;

        pidController = new PIDController(Grab.kP, Grab.kI, Grab.kD);
        feedForward = new SimpleMotorFeedforward(Grab.kS, Grab.kV, Grab.kA);
    }

    public void grab() {
        grabberMotor.setVoltage(Grab.kVoltage);
    }

    public void stop() {
        grabberMotor.setVoltage(0);
    }

    public void reverse() {
        grabberMotor.setInverted(true);
        grabberMotor.setVoltage(Grab.kVoltage);
    }

    
    
         
    }
