package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/*
 *  Autonomous command which makes robot drive forward, reducing speed as
 *  the rotation of the robot on the x axis nears zero.
 */
public class BalanceCSCmd extends CommandBase {
    private final DriveSubsystem driveTrain;
    private Timer timer;
    private boolean complete = false;

    public BalanceCSCmd(DriveSubsystem driveTrain) {
        this.driveTrain = driveTrain;
        this.timer = new Timer();
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double pitch = driveTrain.getGyro().getPitch();
        
        final double kP = 0.005;
        // Error is zero since goal always 0.
        double error = pitch;
        double velocity = MathUtil.clamp(error * kP, -0.4, 0.4);

        if (Math.abs(error) > 5) {
            driveTrain.mecanumDrive(velocity, 0, 0);
        } else {
            complete = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.mecanumDrive(0, 0, 0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return complete;
    }
}
