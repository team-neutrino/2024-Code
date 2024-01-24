// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.util.SubsystemContainer;

public class ClimbRetractCommand extends Command {

  /**
   * A reference to the ONE instance of the climb subsystem in the subsystem
   * container initialized here for easy access.
   */
  private ClimbSubsystem m_climbSubsystem = SubsystemContainer.climbSubsystem;

  private ArmSubsystem m_armSubsystem = SubsystemContainer.armSubsystem;

  /** Creates a new ClimbRetractCommand. */
  public ClimbRetractCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climbSubsystem, m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every 20ms
  @Override
  public void execute() {
    /**
     * this logic is to prevent the climber from retracting until the arm
     * is in the designated position for climbing (otherwise the arm
     * would bend the chain and we'd get a foul)
     */
    if (m_armSubsystem.getCurrentTargetAngle() != Constants.ArmConstants.CLIMB_POSITION) {
      m_armSubsystem.armPID(Constants.ArmConstants.CLIMB_POSITION);
    } else if (m_armSubsystem.getInPosisition()) {
      m_climbSubsystem.rectractClimberArms();
    }
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
