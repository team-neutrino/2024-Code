// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

public class ArmClimbCommandDown extends GamePieceCommand {
  /** Creates a new ArmClimbCommand. */
  public ArmClimbCommandDown() {
    // Use () here to declare subsystem dependencies.
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