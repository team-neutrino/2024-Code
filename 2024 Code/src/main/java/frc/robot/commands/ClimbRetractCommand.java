// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.util.SubsystemContainer;

public class ClimbRetractCommand extends Command {
  /** Creates a new ClimbRetractCommand. */
  public ClimbRetractCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SubsystemContainer.climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SubsystemContainer.climbSubsystem.rectractClimber();
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
