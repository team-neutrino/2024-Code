// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SubsystemContainer;
import frc.robot.util.CalculateAngle;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class MagicSpeakerCommand extends Command {
  CalculateAngle m_calculateAngle;
  ArmSubsystem m_arm;
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;

  public MagicSpeakerCommand(CalculateAngle p_calculateAngle) {
    m_calculateAngle = p_calculateAngle;
    m_arm = SubsystemContainer.armSubsystem;
    m_shooter = SubsystemContainer.ShooterSubsystem;
    m_intake = SubsystemContainer.intakeSubsystem;
    addRequirements(m_arm, m_shooter, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.armPID(m_calculateAngle.InterpolateAngle());
    m_shooter.setTargetRPM(2000);
    if (m_arm.getInPosition() && m_shooter.approveShoot()) {
      m_intake.runIndex();
    } else {
      m_intake.stopIndex();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
