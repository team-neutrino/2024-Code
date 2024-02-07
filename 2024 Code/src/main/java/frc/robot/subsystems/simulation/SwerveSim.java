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
import frc.robot.util.SubsystemContainer;

public class SwerveSim extends SwerveSubsystem {
    public double fieldX;
    public double fieldY;
    public double fieldTheta;

    public double botX;
    public double botY;
    public double botTheta;

    public double convX;
    public double convY;
    public double convTheta;

    @Override
    public void resetNavX() {
        convX = fieldX;
        convY = fieldY;
        convTheta = fieldTheta;

        botX = 0.0;
        botY = 0.0;
        botTheta = 0.0;
    }

    public void simulationInit() {
        fieldX = 0.0;
        fieldY = 0.0;
        fieldTheta = 90.0;
    }

    public void simulationPeriodic() {
        fieldX = Math.cos(convTheta) * botX - Math.sin(convTheta) * botY;
        fieldY = Math.sin(convTheta) * botX + Math.cos(convTheta) * botY;
        fieldTheta = convTheta + botTheta;
        field.getObject("robot").setPose(fieldX, fieldY, new Rotation2d(fieldTheta));

    }

    public void periodic() {
        super.periodic();
        System.out.println(fieldX + " " + fieldY);

    }

    @Override
    public void Swerve(double vx, double vy, double omega) {
        // how m a ny times command runs in a second and divide by that number
        botX += (vx / 50.0);
        botY += (vy / 50.0);
        botTheta += (omega / 50.0);
    }
}
