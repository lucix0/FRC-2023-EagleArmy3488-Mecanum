package frc.robot.commands;

import frc.robot.subsystems.FourBarSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FourBarActCmd extends CommandBase {
    private FourBarSubsystem m_FourBarSubsystem;
    private boolean complete = false;

    public FourBarActCmd(FourBarSubsystem fourBarSubsystem) {
        m_FourBarSubsystem = fourBarSubsystem;
        addRequirements(m_FourBarSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_FourBarSubsystem.run();
        complete = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {  }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return complete;
    }
}
