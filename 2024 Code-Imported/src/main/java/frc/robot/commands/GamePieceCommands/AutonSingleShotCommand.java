package frc.robot.commands.GamePieceCommands;

import frc.robot.Constants;

public class AutonSingleShotCommand extends GamePieceCommand {
    private double m_angle;
    private double m_rpm;
    double i = 0;

    public AutonSingleShotCommand(double p_angle, double p_rpm) {
        m_angle = p_angle;
        m_rpm = p_rpm;
    }

    public void initialize() {
        m_armSubsystem.commandStart();
        m_shooterSubsystem.useHighCurrentLimits(true);
    }

    @Override
    public void execute() {
        m_armSubsystem.setArmReferenceAngle(m_angle);
        m_shooterSubsystem.setTargetRPM(m_rpm);
        if (m_shooterSubsystem.aboveRPM(Constants.ShooterSpeeds.LOW_SPEED_THRESHOLD)) {
            m_intakeSubsystem.runIndexShoot();
        } else {
            m_intakeSubsystem.stopIndex();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.useHighCurrentLimits(false);
    }

    @Override
    public boolean isFinished() {
        if (!m_intakeSubsystem.isBeamBrokenIntake()) {
            i++;
        } else {
            i = 0;
        }
        if (i >= 20) {
            return true;
        }

        return false;
    }
}