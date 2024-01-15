// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

<<<<<<<< HEAD:2024 Code/src/main/java/frc/robot/commands/ShooterdefaultCommand.java
<<<<<<< Updated upstream
import frc.robot.subsystems.ExampleSubsystem;
=======
>>>>>>> Stashed changes
import frc.robot.subsystems.ShooterSubsystem;
========
import edu.wpi.first.wpilibj.XboxController;
>>>>>>>> main:2024 Code/src/main/java/frc/robot/commands/SwerveDefaultCommand.java
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SubsystemContainer;

<<<<<<<< HEAD:2024 Code/src/main/java/frc/robot/commands/ShooterdefaultCommand.java
/** An example command that uses an example subsystem. */
public class ShooterdefaultCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterdefaultCommand(ShooterSubsystem p_shooter) {
    m_shooter = p_shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
========
public class SwerveDefaultCommand extends Command {
  XboxController m_xboxController;
  public SwerveDefaultCommand(XboxController p_controller) {
    m_xboxController = p_controller;
    addRequirements(SubsystemContainer.swerveSubsystem);
>>>>>>>> main:2024 Code/src/main/java/frc/robot/commands/SwerveDefaultCommand.java
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<<< HEAD:2024 Code/src/main/java/frc/robot/commands/ShooterdefaultCommand.java
    m_shooter.setTargetRPM(500);
========
    SubsystemContainer.swerveSubsystem.Swerve(m_xboxController.getLeftY()*-1, m_xboxController.getLeftX()*-1, m_xboxController.getRightX()*-1);
>>>>>>>> main:2024 Code/src/main/java/frc/robot/commands/SwerveDefaultCommand.java
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
