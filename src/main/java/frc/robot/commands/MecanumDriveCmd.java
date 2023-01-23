package frc.robot.commands;

import static frc.robot.Constants.ControllerConstants;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Range;
import frc.robot.Util;
import frc.robot.subsystems.DriveTrain;

public class MecanumDriveCmd extends CommandBase {
    private final DriveTrain driveTrain;
    private Supplier<Double> zSpeedFunc, xSpeedFunc, zRotationFunc;

    public MecanumDriveCmd(DriveTrain driveTrain, Supplier<Double> zSpeedFunc, Supplier<Double> xSpeedFunc, Supplier<Double> zRotationFunc) {
        this.driveTrain = driveTrain;
        this.zSpeedFunc = zSpeedFunc;
        this.xSpeedFunc = xSpeedFunc;
        this.zRotationFunc = zRotationFunc;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double zSpeed = zSpeedFunc.get();
        double xSpeed = xSpeedFunc.get();
        double zRotation = zRotationFunc.get();

        /*  
         *  This section of code restricts the range of input of the sticks
         *  to counter stick drift. Input can be between 0.15 and 1.00,
         *  which is then strected to 0.00 to 1.00,
         *  for example, 0.15 becomes 0.00.
         */ 
        
        Range initial = new Range(1.00, ControllerConstants.kStickDeadzone);
        Range limited = new Range(1.00, 0.00);
        
        double newZSpeed = 0.0;
        if (Math.abs(zSpeed) > ControllerConstants.kStickDeadzone - 0.01) {
            newZSpeed = Range.convert(Math.abs(zSpeed), initial, limited);
            newZSpeed = Util.matchSign(zSpeed, newZSpeed);
        }
        
        double newXSpeed = 0.0;
        if (Math.abs(xSpeed) > ControllerConstants.kStickDeadzone - 0.01) {
            newXSpeed = Range.convert(Math.abs(xSpeed), initial, limited);
            newXSpeed = Util.matchSign(xSpeed, newXSpeed);
        }

        double newZRotation = 0.0;
        if (Math.abs(zRotation) > ControllerConstants.kStickDeadzone - 0.01) {
            newZRotation = Range.convert(Math.abs(zRotation), initial, limited);
            newZRotation = Util.matchSign(zRotation, newZRotation);
        }   

        // Y-axis must be flipped because up on the left stick is neg.
        driveTrain.mecanumDrive(-newZSpeed, newXSpeed, newZRotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
