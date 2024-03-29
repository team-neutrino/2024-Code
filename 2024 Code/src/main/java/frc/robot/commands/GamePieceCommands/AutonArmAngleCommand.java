// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

;;

public class AutonArmAngleCommand extends GamePieceCommand {
    private double m_angle;

    public AutonArmAngleCommand(double p_angle) {
        m_angle = p_angle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_armSubsystem.setArmReferenceAngle(m_angle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (m_armSubsystem.getArmAngleDegrees() <= (m_angle + 5)
                && m_armSubsystem.getArmAngleDegrees() >= (m_angle - 5));

    }
}
