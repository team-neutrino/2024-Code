// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

public class AutonShooterCommand extends GamePieceCommand {

  private double m_rpm;
  private ArmSubsystem m_armSubsystem;
  private SwerveSubsystem m_swerve;
  private CalculateAngle m_angleCalculate;
  private IntakeSubsystem m_intakeSubsystem;

  public AutonShooterCommand(double p_rpm) {
    m_rpm = p_rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setTargetRPM(m_rpm);
    m_armSubsystem.setArmReferenceAngle(m_angleCalculate.InterpolateAngle(m_swerve.GetSpeakerToRobot()));
    if (m_armSubsystem.getInPosition() && m_shooterSubsystem.approveShoot()) {
      m_intakeSubsystem.runIndexShoot();
    } else {
      m_intakeSubsystem.stopIndex();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return SubsystemContainer.intakeSubsystem.hasNoNote();
  }
}
