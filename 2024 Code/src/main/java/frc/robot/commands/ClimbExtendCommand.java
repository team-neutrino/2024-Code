// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.util.SubsystemContainer;

public class ClimbExtendCommand extends Command {

  /**
   * A reference to the ONE instance of the climb and arm subsystems in the
   * subsystem container initialized here for easy access.
   */
  private ClimbSubsystem m_climbSubsystem = SubsystemContainer.climbSubsystem;

  private ArmSubsystem m_armSubsystem = SubsystemContainer.armSubsystem;

  /** Creates a new ClimbExtendCommand. */
  public ClimbExtendCommand() {
    addRequirements(m_climbSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every 20 ms
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * This code assumes that if we are raising the climber we are going to
     * pull ourselves up directly afterward, so this command proactively puts
     * the arm to the correct position for climbing.
     */
    m_armSubsystem.armPID(Constants.ArmConstants.CLIMB_POSITION);
    m_climbSubsystem.extendClimberArms();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.removeDefaultCommand();
    m_armSubsystem.setDefaultCommand(new ArmAngleCommand(Constants.ArmConstants.CLIMB_POSITION));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_climbSubsystem.getArmEncoderPosition()[1] >=
    // Constants.ClimbConstants.CLIMB_LIMIT_UP;
    return m_armSubsystem.getArmPose() >= Constants.ClimbConstants.CLIMB_LIMIT_UP;
  }
}
