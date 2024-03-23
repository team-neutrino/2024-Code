// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

public class AutonAlignAndChargeCommand extends Command {

  private double currentYaw;
  private double offsetYaw;
  private Timer timer = new Timer();
  private double[] pose;
  private LimelightSubsystem m_limlightSubsystem = SubsystemContainer.limelightSubsystem;


  /** Creates a new AutonAlignAndChargeCommand. */
  public AutonAlignAndChargeCommand() {
    addRequirements(m_limlightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (SubsystemContainer.swerveSubsystem.isRedAlliance()) {
      SubsystemContainer.limelightSubsystem.setPriorityID(4);
  } else {
      SubsystemContainer.limelightSubsystem.setPriorityID(7);
  }
  timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentYaw = SubsystemContainer.swerveSubsystem.getYaw();
        offsetYaw = SubsystemContainer.limelightSubsystem.getTx();
        pose = SubsystemContainer.limelightSubsystem.getBotPose();
        if (!SubsystemContainer.swerveSubsystem.isRedAlliance()) {
            if (pose[5] > 0) {
                pose[5] -= 180;
            } else {
                pose[5] += 180;
            }

        }
        SubsystemContainer.swerveSubsystem
                .autonRotateSwerve(SwerveSubsystem.calculateLimelightOffsetAngle(currentYaw, offsetYaw, pose[5]));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
