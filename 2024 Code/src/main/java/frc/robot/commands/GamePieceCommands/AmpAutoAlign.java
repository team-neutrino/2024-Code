// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.AprilTagConstants.BLUE_ALLIANCE_IDS;
import frc.robot.Constants.AprilTagConstants.RED_ALLIANCE_IDS;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class AmpAutoAlign extends GamePieceCommand {
  private double m_ampYaw;
  private double m_commandMod;
  private int speakerTagID;

  private SwerveSubsystem m_swerveSubsystem;
  private CommandXboxController m_controller;

  /** Creates a new AmpAutoAlign. */
  public AmpAutoAlign(CommandXboxController p_xboxController) {
    m_swerveSubsystem = SubsystemContainer.swerveSubsystem;
    m_controller = p_xboxController;
    // added for (maybe) special case to the usual don't-require-swerve convention:
    // we want both omega and left-right movement to be locked, so by overriding the
    // swerve default control I can force the only valid input to be the y-axis of
    // the left stick (up-down control)
    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (SubsystemContainer.alliance.isRedAlliance()) {
      m_ampYaw = SwerveConstants.AMP_ORIENTATION_RED_ALLIANCE;
      m_commandMod = 1;
      speakerTagID = RED_ALLIANCE_IDS.SPEAKER_ID;
    } else {
      m_ampYaw = SwerveConstants.AMP_ORIENTATION_BLUE_ALLIANCE;
      m_commandMod = -1;
      speakerTagID = BLUE_ALLIANCE_IDS.SPEAKER_ID;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SubsystemContainer.limelightSubsystem.getTv()) {
      SubsystemContainer.limelightSubsystem.resetOdometryToLimelightPose();
    }

    m_swerveSubsystem.setRobotYaw(m_ampYaw);

    double command = SwerveConstants.AMP_ALIGN_KP * m_swerveSubsystem.getAmpDx();
    command = Math.min(Math.max(command, -SwerveConstants.AMP_DX_LIMIT_VALUE), SwerveConstants.AMP_DX_LIMIT_VALUE);

    m_swerveSubsystem.SwerveWithoutDeadzone(command * m_commandMod, m_controller.getLeftX() * -1, 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SubsystemContainer.limelightSubsystem.setPriorityID(speakerTagID);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
