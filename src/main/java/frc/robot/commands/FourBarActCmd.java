package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FourBarSubsystem;

public class FourBarActCmd extends CommandBase {
    private final FourBarSubsystem fourBar;
    private boolean complete = false;

    public FourBarActCmd(FourBarSubsystem fourBar) {
        this.fourBar = fourBar;
        addRequirements(fourBar);
    }

    @Override
    public void execute() {
        fourBar.run();
        complete = true;
    }

    @Override
    public boolean isFinished() {
        return complete;
    }
}
