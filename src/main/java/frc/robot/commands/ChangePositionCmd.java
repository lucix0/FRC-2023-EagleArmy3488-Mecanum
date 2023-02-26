package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChangePositionCmd extends CommandBase {
    private PositionalSubsystem m_Subsystem;
    private double m_position;

    public ChangePositionCmd(PositionalSubsystem subsystem, double position) {
        m_Subsystem = subsystem;
        m_position = position;
        addRequirements(m_Subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
        m_Subsystem.changePosition(m_position);
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
