package frc.robot.commands.GamePieceCommands;

public class AutonSingleShotCommand extends GamePieceCommand {
    private double m_angle;
    private double m_rpm;
    double i = 0;

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
        if (m_armSubsystem.getInPosition() && m_shooterSubsystem.approveShoot()) {
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