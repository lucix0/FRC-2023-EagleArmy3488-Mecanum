package frc.robot.commands;

import frc.robot.Constants.*;
import frc.lib.Range;
import frc.lib.Util;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class DriveCmd extends CommandBase {
    private DriveSubsystem m_DriveSubsystem;
    private Supplier<Double> zSpeedFunc, xSpeedFunc, zRotationFunc;

    public DriveCmd(DriveSubsystem driveSubsystem, Supplier<Double> zSpeedFunc, Supplier<Double> xSpeedFunc, Supplier<Double> zRotationFunc) {
        m_DriveSubsystem = driveSubsystem;
        this.zSpeedFunc = zSpeedFunc;
        this.xSpeedFunc = xSpeedFunc;
        this.zRotationFunc = zRotationFunc;
        addRequirements(m_DriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double zSpeed = zSpeedFunc.get();
        double xSpeed = xSpeedFunc.get();
        double zRotation = zRotationFunc.get();
        
        Range initial = new Range(1.00, Controller.kDeadband);
        Range limited = new Range(1.00, 0.00);
        
        double newZSpeed = 0.0;
        if (Math.abs(zSpeed) > Controller.kDeadband - 0.01) {
            newZSpeed = Range.convert(Math.abs(zSpeed), initial, limited);
            newZSpeed = Util.matchSign(zSpeed, newZSpeed);
        }
        
        double newXSpeed = 0.0;
        if (Math.abs(xSpeed) > Controller.kDeadband - 0.01) {
            newXSpeed = Range.convert(Math.abs(xSpeed), initial, limited);
            newXSpeed = Util.matchSign(xSpeed, newXSpeed);
        }

        double newZRotation = 0.0;
        if (Math.abs(zRotation) > Controller.kDeadband - 0.01) {
            newZRotation = Range.convert(Math.abs(zRotation), initial, limited);
            newZRotation = Util.matchSign(zRotation, newZRotation);
        }   

        // Y-axis must be flipped because up on the left stick is neg.
        m_DriveSubsystem.drive(-newZSpeed, newXSpeed, newZRotation);
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
