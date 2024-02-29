// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.SubsystemContainer;

public class ArmClimbCommandDown extends Command {

  ArmSubsystem m_armSubsystem;

  /** Creates a new ArmClimbCommand. */
  public ArmClimbCommandDown() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = SubsystemContainer.armSubsystem;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("climb enabled------------------");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setClimbReferenceAngle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("climb disabled---------------------");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}