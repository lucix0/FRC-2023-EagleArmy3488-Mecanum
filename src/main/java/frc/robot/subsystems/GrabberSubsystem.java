package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.Constants.Grabber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
  
public class GrabberSubsystem extends SubsystemBase {
    private WPI_TalonFX grabberMotor;

    private double speed;

    public GrabberSubsystem() {
        grabberMotor = new WPI_TalonFX(RobotMap.kGrabberMotorID);
        grabberMotor.configFactoryDefault();

        speed = Grabber.kGrabSpeed;
    }

    public void run() {
        grabberMotor.set(speed);
    }

    public void stop() {
        grabberMotor.set(0);
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
}
