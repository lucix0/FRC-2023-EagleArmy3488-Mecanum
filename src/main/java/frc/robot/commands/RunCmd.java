package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PositionalSubsystem;

public class RunCmd extends CommandBase {
    private PositionalSubsystem m_Subsystem;

    public RunCmd(PositionalSubsystem subsystem) {
        m_Subsystem = subsystem;
        addRequirements(m_Subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
        m_Subsystem.run();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {  }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
