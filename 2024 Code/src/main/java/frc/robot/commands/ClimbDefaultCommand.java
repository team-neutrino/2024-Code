// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbDefaultCommand extends Command {

  private ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  /** Creates a new ClimbDefaultCommand. */
  public ClimbDefaultCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Default command called");
  }

  // Called every 20 ms
  @Override
  public void execute() {
    m_climbSubsystem.extendClimber();
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
