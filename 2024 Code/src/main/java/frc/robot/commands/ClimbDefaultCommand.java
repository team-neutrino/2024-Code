// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SubsystemContainer;

public class ClimbDefaultCommand extends Command {

  /** Creates a new ClimbDefaultCommand. */
  public ClimbDefaultCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SubsystemContainer.climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every 20 ms
  @Override
  public void execute() {
    SubsystemContainer.climbSubsystem.stopClimber();
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
