// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.subsystems.*;
import frc.robot.util.SubsystemContainer;

public class MagicAmpChargeCommand extends Command {

  private ArmSubsystem m_armSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private CommandXboxController m_controller;

  /** Creates a new MagicAmpChargeCommand. */
  public MagicAmpChargeCommand(CommandXboxController p_xboxcontroller) {
    m_controller = p_xboxcontroller;
    m_armSubsystem = SubsystemContainer.armSubsystem;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;

    addRequirements(m_shooterSubsystem, m_intakeSubsystem, m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setArmReferenceAngle(ArmConstants.AMP_POSE);
    m_shooterSubsystem.setTargetRPM(ShooterSpeeds.AMP_SPEED);
    m_intakeSubsystem.runIndexFeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armSubsystem.getInPosition() && m_shooterSubsystem.approveShoot() && m_controller.getHID().getLeftBumper();
  }
}
