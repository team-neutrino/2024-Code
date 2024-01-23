// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.SubsystemContainer;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class MagicAmpCommand extends Command {
  private ArmSubsystem m_arm;
  private ShooterSubsystem m_shooter;
  private LimelightSubsystem m_limelight;
  private IntakeSubsystem m_index;
  private int i;

  /** Creates a new MagicAmpCommand. */
  public MagicAmpCommand() {
    m_arm = SubsystemContainer.armSubsystem;
    m_shooter = SubsystemContainer.ShooterSubsystem;
    m_limelight = SubsystemContainer.limelightSubsystem;
    m_index = SubsystemContainer.intakeSubsystem;
    addRequirements(m_arm, m_shooter, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.getID() == 6 || m_limelight.getID() == 5) {
      m_arm.armChecker(m_arm.armPID(Constants.ArmConstants.AMP_POSE));
      if (Math.abs(m_arm.getArmPose() - Constants.ArmConstants.AMP_POSE) <= 5) {
       i++;
       if (i == 10) {
        m_shooter.setTargetRPM(1000);
        if (m_shooter.approveShoot()) {
            m_index.runIndex();
        } else {
            m_index.stopIndex();
        }
       }
       else {
        m_shooter.stopShooter();
       }
      }
      else {
        i = 0;
      }
    }
    else {
      m_arm.armChecker(m_arm.armPID(m_arm.getArmPose()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
