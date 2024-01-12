// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.FieldConstants;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoAlign extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final LimelightSubsystem m_limelight;
  boolean contact = false;

  public AutoAlign(LimelightSubsystem limelight) {
    m_limelight = limelight;
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double[] pose = m_limelight.getBotPose();
    double mNaught = (FieldConstants.autoAlignPoint[2] - pose[2]) / (FieldConstants.autoAlignPoint[0] - pose[0]);
    double cNaught = pose[2] - mNaught;
    for (Double[] region : FieldConstants.bannedRegions) {
      if (region[0] != null) {
        double x_i = (region[1] - cNaught) / (region[0] - mNaught);
        if (x_i > region[3] && x_i < region[4]) {
          contact = true;
        }
      } else {
        double z_i = mNaught * region[1] + cNaught;

      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
