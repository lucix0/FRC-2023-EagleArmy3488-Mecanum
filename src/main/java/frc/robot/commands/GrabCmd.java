package frc.robot.commands;

import frc.robot.subsystems.GrabberSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabCmd extends CommandBase {
    private GrabberSubsystem m_GrabberSubsystem;

    public GrabCmd(GrabberSubsystem grabberSubsystem) {
        m_GrabberSubsystem = grabberSubsystem;
        addRequirements(m_GrabberSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_GrabberSubsystem.grab();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_GrabberSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
