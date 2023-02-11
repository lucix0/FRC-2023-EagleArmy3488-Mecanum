package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class SetGrabberSpeedCmd extends CommandBase {
    private GrabberSubsystem m_GrabberSubsystem;
    private double speed;

    public SetGrabberSpeedCmd(GrabberSubsystem grabberSubsystem, double speed) {
        m_GrabberSubsystem = grabberSubsystem;
        this.speed = speed;

        addRequirements(m_GrabberSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {  
        m_GrabberSubsystem.setSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {  }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
