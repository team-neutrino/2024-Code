package frc.robot.subsystems.simulation;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveSim extends SwerveSubsystem {
    public Field2d m_Field2d = new Field2d();
    public Pose2d m_Pose2d = new Pose2d();
    // update with correct values

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    public SwerveSim() {
        SmartDashboard.putData("Drivetrain", m_Field2d);
    }

    public void simulationInit() {

    }

    public void simulationPeriodic() {

    }

    public void periodic() {
        super.periodic();

    }

    @Override
    public void Swerve(double vx, double vy, double omega) {
        m_Field2d.setRobotPose(vy, omega, null);
    }
}
