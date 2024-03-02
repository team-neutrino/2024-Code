// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.management.InstanceAlreadyExistsException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.SubsystemContainer;
import frc.robot.util.CalculateAngle;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class MagicSpeakerCommand extends Command {
  CalculateAngle m_calculateAngle;
  ArmSubsystem m_armSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  CommandXboxController m_controller;

  public MagicSpeakerCommand(CalculateAngle p_calculateAngle, CommandXboxController p_controller) {
    m_calculateAngle = p_calculateAngle;
    m_armSubsystem = SubsystemContainer.armSubsystem;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    m_controller = p_controller;
    addRequirements(m_armSubsystem, m_shooterSubsystem, m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setArmReferenceAngle(m_calculateAngle.InterpolateAngle());
    m_shooterSubsystem.setTargetRPM(3000);
    if (m_armSubsystem.getInPosition() && m_shooterSubsystem.approveShoot()) {
      m_controller.leftBumper().onTrue(new InstantCommand(() -> m_intakeSubsystem.runIndexShoot()));
    } else {
      m_intakeSubsystem.stopIndex();
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
