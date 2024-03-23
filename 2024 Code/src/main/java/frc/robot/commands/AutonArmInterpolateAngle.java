// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SubsystemContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CalculateAngle;

public class AutonArmInterpolateAngle extends Command {
  private ArmSubsystem m_armSubsystem;
  private SwerveSubsystem m_swerve;
  private CalculateAngle m_angleCalculate;

  public AutonArmInterpolateAngle(CalculateAngle p_angleCalculate) {
    m_angleCalculate = p_angleCalculate;
    m_armSubsystem = SubsystemContainer.armSubsystem;
    m_swerve = SubsystemContainer.swerveSubsystem;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setArmReferenceAngle(m_angleCalculate.InterpolateAngle(m_swerve.GetSpeakerToRobot()));
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
