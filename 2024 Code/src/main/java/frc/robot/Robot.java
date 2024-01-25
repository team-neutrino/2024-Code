// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.SubsystemContainer;

// =============================================================================
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // ===========================================================================
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  // ===========================================================================
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // ===========================================================================
  @Override
  public void disabledInit() {
  }

  // ===========================================================================
  @Override
  public void disabledPeriodic() {
  }

  // ===========================================================================
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    Translation2d startPose;
    if (SubsystemContainer.swerveSubsystem.isRedAlliance()) {
      startPose = SwerveConstants.m_redCoordMap.get(m_autonomousCommand);
    } else {
      startPose = SwerveConstants.m_blueCoordMap.get(m_autonomousCommand);
    }
    SubsystemContainer.swerveSubsystem.resetStartPosition(startPose);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  // ===========================================================================
  @Override
  public void autonomousPeriodic() {
  }

  // ===========================================================================
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  // ===========================================================================
  @Override
  public void teleopPeriodic() {
  }

  // ===========================================================================
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  // ===========================================================================
  @Override
  public void testPeriodic() {
  }

  // ===========================================================================
  // ===========================================================================

  public void simulationInit() {
    m_robotContainer.simulationInit();
  }

  public void simulationPeriodic() {
    m_robotContainer.simulationPeriodic();
  }

}
