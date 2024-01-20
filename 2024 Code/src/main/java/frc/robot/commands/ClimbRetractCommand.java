// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.util.SubsystemContainer;

public class ClimbRetractCommand extends Command {

  /**
   * A reference to the ONE instance of the climb subsystem in the subsystem
   * container initialized here for easy access.
   */
  private ClimbSubsystem m_climbSubsystem = SubsystemContainer.climbSubsystem;

  /** Creates a new ClimbRetractCommand. */
  public ClimbRetractCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every 20ms
  @Override
  public void execute() {
    m_climbSubsystem.rectractClimberArms();
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
