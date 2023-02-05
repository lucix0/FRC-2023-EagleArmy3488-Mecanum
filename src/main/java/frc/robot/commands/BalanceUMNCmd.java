// MIT License
//
// Copyright (c) 2023 Anthony Brogni
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

// This command self-balances on the charging station using gyroscope pitch as feedback
public class BalanceUMNCmd extends CommandBase {
    private DriveSubsystem m_driveSubsystem;

    private double error;
    private double currentAngle;
    private double drivePower;

    /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
    public BalanceUMNCmd(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
        //Double currentAngle = -1 * Robot.driveC.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
        this.currentAngle = m_driveSubsystem.getPitch();

        error = Balance.kGoalDegrees - currentAngle;
        drivePower = -Math.min(Balance.kP * error, 1);

        // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
        if (drivePower < 0) {
          drivePower *= Balance.kBackwardsBoostMultiplier;
        }

        // Limit the max power
        if (Math.abs(drivePower) > 0.4) {
          drivePower = Math.copySign(0.4, drivePower);
        }

        m_driveSubsystem.drive(drivePower, 0, 0);
    
        // Debugging Print Statments
        System.out.println("Current Angle: " + currentAngle);
        System.out.println("Error " + error);
        System.out.println("Drive Power: " + drivePower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {  }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return Math.abs(error) < Balance.kAngleThreshold;
    }
}