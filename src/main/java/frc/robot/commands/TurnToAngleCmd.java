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

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.*;
import frc.robot.subsystems.DriveSubsystem;

// Uses PID control to align to the target angle using gyroscope feedback
public class TurnToAngleCmd extends CommandBase {
    DriveSubsystem m_DriveSubsystem;
    double degreesToTurn;
    double error;
    double targetAngle; // targetAngle = initial angle + degreesToTurn

    /** Turns to an angle relative to the current angle using the gyro */
    public TurnToAngleCmd(DriveSubsystem driveSubsystem, double degreesToTurn) {
        m_DriveSubsystem = driveSubsystem;
        addRequirements(m_DriveSubsystem);
        this.degreesToTurn = degreesToTurn;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.targetAngle = degreesToTurn + m_DriveSubsystem.getAngle();
        System.out.println("CURRENT ANGLE:" + m_DriveSubsystem.getAngle());
        System.out.println("TARGET ANGLE:" + targetAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        error = targetAngle - m_DriveSubsystem.getAngle(); // Our target angle, being the angle we want the robot in, vs m_DriveSubsystem.getAngle(), which gets our current angle from the robot

        double value = error * Turn.kP; // Multiply by scaling factor kp to determine motor percent power between 0 and 100 percent
        if (Math.abs(value) > 0.75) { // Maximum drive value we want
            value = Math.copySign(0.75, value);
        }
        if (Math.abs(value) < 0.15) { // Minimum drive value we want
            value = Math.copySign(0.15, value);
        }
        // Print statements for debugging //
        System.out.println("error:" + error);
        System.out.println("value:" + value);

        m_DriveSubsystem.mecanumDrive(0, 0, value); // drive with the calculated values
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(error) < Turn.kAngleThreshold; // End the command when we are within the specified threshold of our target
    }
}