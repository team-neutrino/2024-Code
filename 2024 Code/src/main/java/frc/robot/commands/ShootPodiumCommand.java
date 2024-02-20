// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterSpeeds;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.SubsystemContainer;

public class ShootPodiumCommand extends Command {
  ArmSubsystem m_armSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  IntakeSubsystem m_intakeSubsystem;

  public ShootPodiumCommand() {
    m_armSubsystem = SubsystemContainer.armSubsystem;
    m_shooterSubsystem = SubsystemContainer.shooterSubsystem;
    m_intakeSubsystem = SubsystemContainer.intakeSubsystem;
    addRequirements(m_armSubsystem, m_shooterSubsystem, m_intakeSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_armSubsystem.setArmReferenceAngle(Constants.ArmConstants.PODIUM_ANGLE);
    m_shooterSubsystem.setTargetRPM(ShooterSpeeds.PODIUM_SPEED);
    if (m_armSubsystem.getInPosition() && m_shooterSubsystem.approveShoot()) {
      m_intakeSubsystem.runIndex();
    } else {
      m_intakeSubsystem.stopIndex();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
