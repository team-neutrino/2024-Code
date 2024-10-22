package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.util.SubsystemContainer;

public class ShootShuttleCommand extends GamePieceCommand {
    private double m_angle;
    private CommandXboxController m_Controller;
    private double speedCalc;

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
        // speedCalc = Constants.ShooterSpeeds.SHUTTLE_SPEED
        //         * SubsystemContainer.swerveSubsystem.GetSpeakerToRobot().getRadius();
        // speedCalc = Math.min(Constants.ShooterSpeeds.MAX_SHUTTLE_SPEED, speedCalc);
        // speedCalc = Math.max(Constants.ShooterSpeeds.MIN_SHUTTLE_SPEED, speedCalc);
        m_shooterSubsystem.setTargetRPM(Constants.ShooterSpeeds.SHUTTLE_SPEED_CONSTANT);
        m_intakeSubsystem.runIndexFeed();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_Controller.getHID().getLeftBumper()
                && m_shooterSubsystem.aboveRPM(speedCalc - Constants.ShooterSpeeds.SHUTTLE_THRESHOLD_ERROR)
                && m_intakeSubsystem.hasNote()
                && m_armSubsystem.aboveAngle(Constants.ArmConstants.SHUTTLE_ANGLE_THRESHOLD);
    }
}