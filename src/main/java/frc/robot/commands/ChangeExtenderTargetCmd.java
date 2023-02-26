package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Extender;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;

public class ChangeExtenderTargetCmd extends CommandBase {
    private ExtenderSubsystem m_ExtenderSubsystem;

    public ChangeExtenderTargetCmd(ExtenderSubsystem extenderSubsystem) {
        m_ExtenderSubsystem = extenderSubsystem;
        addRequirements(m_ExtenderSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_ExtenderSubsystem.grabbing = !m_ExtenderSubsystem.grabbing;
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
