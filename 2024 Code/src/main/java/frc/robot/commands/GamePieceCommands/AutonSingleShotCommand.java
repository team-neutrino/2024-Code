package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj.Timer;

public class AutonSingleShotCommand extends GamePieceCommand {
    private double m_angle;
    private double m_rpm;
    Timer timer = new Timer();

    public AutonSingleShotCommand(double p_angle, double p_rpm) {
        m_angle = p_angle;
        m_rpm = p_rpm;
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        m_armSubsystem.setArmReferenceAngle(m_angle);
        m_shooterSubsystem.setTargetRPM(m_rpm);
        if (m_armSubsystem.isInPosition() && m_shooterSubsystem.approveShoot()) {
            m_intakeSubsystem.runIndexShoot();
        } else {
            m_intakeSubsystem.stopIndex();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (!m_intakeSubsystem.isBeamBrokenIntake()) {
            timer.start();
        } else {
            timer.stop();
            timer.reset();
        }
        if (timer.hasElapsed(.2)) {
            return true;
        }

        return false;
    }
}