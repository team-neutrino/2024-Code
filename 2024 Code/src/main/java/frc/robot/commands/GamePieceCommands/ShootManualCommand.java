package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ShootManualCommand extends GamePieceCommand {
    private double m_angle;
    private double m_rpm;
    private double m_thresholdrpm;
    private CommandXboxController m_Controller;
    double i = 0;

    public ShootManualCommand(double p_angle, double p_rpm, double p_thresholdrpm, CommandXboxController p_Controller) {
        m_angle = p_angle;
        m_rpm = p_rpm;
        m_thresholdrpm = p_thresholdrpm;
        m_Controller = p_Controller;
    }

    public void initialize() {
        m_armSubsystem.commandStart();
    }

    @Override
    public void execute() {
        m_armSubsystem.setArmReferenceAngle(m_angle);
        m_shooterSubsystem.setTargetRPM(m_rpm);
        m_intakeSubsystem.runIndexFeed();
        // System.out.println("above RPM: " + m_shooterSubsystem.aboveRPM(m_thresholdrpm));
        // System.out.println("note ready: " + m_intakeSubsystem.isNoteReady());
        // System.out.println("arm in position: " + m_armSubsystem.getInPosition());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_Controller.getHID().getLeftBumper()
                && m_shooterSubsystem.aboveRPM(m_thresholdrpm)
                && m_intakeSubsystem.isNoteReady()
                && m_armSubsystem.getInPosition();
    }
}