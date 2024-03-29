// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;

public class GamePieceCommand extends Command {
  protected IntakeSubsystem m_intakeSubsystem;
  protected ShooterSubsystem m_shooterSubsystem;
  protected ArmSubsystem m_armSubsystem;

  /** Creates a new GamePieceCommand. */
  public GamePieceCommand() {
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_armSubsystem = SubsystemContainer.armSubsystem;

    addRequirements(m_intakeSubsystem, m_shooterSubsystem, m_armSubsystem);
  }
}