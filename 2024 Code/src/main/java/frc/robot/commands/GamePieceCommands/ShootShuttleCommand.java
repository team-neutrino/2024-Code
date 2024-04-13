package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class ShootShuttleCommand extends GamePieceCommand {
    private double m_angle;
    private double m_rpm;
    private double m_thresholdrpm;
    private CommandXboxController m_Controller;
    double i = 0;
    double speedCalc;

    public ShootShuttleCommand(double p_angle, CommandXboxController p_Controller) {
        m_angle = p_angle;
        m_Controller = p_Controller;
    }

    public void initialize() {
        m_armSubsystem.commandStart();
    }

    @Override
    public void execute() {
        m_armSubsystem.setArmReferenceAngle(m_angle);
        speedCalc = Constants.ShooterSpeeds.SHUTTLE_SPEED
                * SubsystemContainer.swerveSubsystem.GetSpeakerToRobot().getRadius();
        speedCalc = Math.min(Constants.ShooterSpeeds.MAX_SHUTTLE_SPEED, speedCalc);
        speedCalc = Math.max(Constants.ShooterSpeeds.MIN_SHUTTLE_SPEED, speedCalc);
        System.out.println(speedCalc);
        m_shooterSubsystem.setTargetRPM(speedCalc);
        m_intakeSubsystem.runIndexFeed();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_Controller.getHID().getLeftBumper()
                && m_shooterSubsystem.aboveRPM(speedCalc - Constants.ShooterSpeeds.SHUTTLE_THRESHOLD_SUBTRACTOR)
                && m_intakeSubsystem.isNoteReady()
                && m_armSubsystem.getInPosition();
    }
}