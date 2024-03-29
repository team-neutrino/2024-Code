// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;;;;;

/**
 * Skeleton requirements class that all GamePieceCommands should extend rather
 * than Command, general protected fields of the required subsystems provided
 * here for use in all direct child classes.
 * 
 * DO NOT IMPLEMENT FUNCTIONALITY METHODS LIKE EXECUTE!!
 */
public class GamePieceCommand extends Command {
  protected ArmSubsystem m_armSubsystem;
  protected ShooterSubsystem m_shooterSubsystem;
  protected IntakeSubsystem m_intakeSubsystem;

  /** Creates a new GamePieceCommand. */
  public GamePieceCommand() {
    m_armSubsystem = SubsystemContainer.armSubsystem;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;

    addRequirements(m_armSubsystem, m_shooterSubsystem,
        m_intakeSubsystem);
  }
}
