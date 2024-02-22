// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class SwervePOVCommand extends Command {

  /**
   * The angle of the POV input on the xbox controller,
   * neutral is -1, up is 0, and updates positively to
   * the right, as in right = 90, down = 180, left = 270.
   * No negative values for non-neutral positions.
   */
  double POV = -1;

  /**
   * A copy of the swerve subsystem from the subsystem container
   * for ease of access.
   */
  SwerveSubsystem swerveSub = SubsystemContainer.swerveSubsystem;

  /**
   * Constructor, takes in current user POV angle and
   * assigns it to its corresponding field. Also does
   * addreq's as needed.
   * 
   * @param POVangle The current angle representation of the POV buttons on the
   *                 driver's controller.
   */
  public SwervePOVCommand(double POVangle) {
    POV = POVangle;
    addRequirements(SubsystemContainer.swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (POV == 0) {

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      // TODO: REMOVE AFTER TESTING
      for (int i = 0; i < 50; i++) {
        System.out.println("isFinished run: field POV was seen at an invalid angle (< 0 or >= 360)");
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return POV < 0 || POV >= 360;
  }
}
