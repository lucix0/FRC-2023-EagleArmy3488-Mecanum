package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FourBar;

public class FourBarActCmd extends CommandBase {
    private final FourBar fourBar;
    private boolean complete = false;

    public FourBarActCmd(FourBar fourBar) {
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
