package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ShootManualCommand extends GamePieceCommand {
    private double m_angle;
    private double m_rpm;
    private CommandXboxController m_Controller;
    double i = 0;

    public ShootManualCommand(double p_angle, double p_rpm, CommandXboxController p_Controller) {
        m_angle = p_angle;
        m_rpm = p_rpm;
        m_Controller = p_Controller;
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        m_armSubsystem.setArmReferenceAngle(m_angle);
        m_shooterSubsystem.setTargetRPM(m_rpm);
        m_intakeSubsystem.runIndexFeed();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_Controller.getHID().getLeftBumper()
                && m_shooterSubsystem.aboveRPM(2800) && m_intakeSubsystem.isNoteReady();
    }
}