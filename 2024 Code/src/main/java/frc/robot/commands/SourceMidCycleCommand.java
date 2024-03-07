// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.util.SubsystemContainer;

public class SourceMidCycleCommand extends Command {

  private ArmSubsystem m_armSubystem;
  private ShooterSubsystem m_shooterSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private CommandXboxController controller;

  /** Creates a new SourceMidCycleCommand. */
  public SourceMidCycleCommand(CommandXboxController p_controller) {
    m_armSubystem = SubsystemContainer.armSubsystem;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    controller = p_controller;

    addRequirements(m_armSubystem, m_shooterSubsystem, m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_intakeSubsystem.isBeamBroken()) {
      m_intakeSubsystem.runIndexIntake();
    } else {
      m_intakeSubsystem.stopIndex();
      m_armSubystem.setArmReferenceAngle(ArmConstants.SOURCE_MID_CYCLE_ANGLE);
      m_shooterSubsystem.setTargetRPM(ShooterSpeeds.SOURCE_MID_CYCLE_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armSubystem.getInPosition() && m_shooterSubsystem.approveShoot() && controller.getHID().getLeftBumper();
  }
}
