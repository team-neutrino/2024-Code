// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.SubsystemContainer;

public class ArmDefaultCommand extends Command {
  /** Creates a new ArmClimbCommandUp. */

  private ArmSubsystem m_armSubsystem;

  public ArmDefaultCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = SubsystemContainer.armSubsystem;
    addRequirements(SubsystemContainer.armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SubsystemContainer.armSubsystem.commandStart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SubsystemContainer.armSubsystem.setArmReferenceAngle(ArmConstants.INTAKE_POSE);
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
