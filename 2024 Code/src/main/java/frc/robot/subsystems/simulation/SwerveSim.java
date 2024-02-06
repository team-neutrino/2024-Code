package frc.robot.subsystems.simulation;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    public double x;
    public double y;
    public double angle;

    public SwerveSim() {
    }

    public void simulationInit() {
        x = 0.0;
        y = 0.0;
        angle = 0.0;
    }

    public void simulationPeriodic() {
        field.setRobotPose(x, y, new Rotation2d(angle));
    }

    public void periodic() {
        super.periodic();

    }

    @Override
    public void Swerve(double vx, double vy, double omega) {
        // how many times command runs in a second and divide by that number
        x += vx;
        y += vy;
        angle += omega;
    }
}
