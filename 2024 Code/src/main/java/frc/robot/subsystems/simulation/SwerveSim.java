package frc.robot.subsystems.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.SwerveSubsystem;

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

    public void simulationInit() {
        botX = 0.0;
        botY = 0.0;
        botTheta = 90.0;
    }

    public void simulationPeriodic() {

        // Thanks Urbana!!!
        // https://motion.cs.illinois.edu/RoboticSystems/CoordinateTransformations.html
        fieldX = Math.cos(convTheta) * (botX) - Math.sin(convTheta) * (botY) + convX;
        fieldY = Math.sin(convTheta) * (botX) + Math.cos(convTheta) * (botY) + convY;
        fieldTheta = convTheta + botTheta;
        field.getObject("robot").setPose(fieldX, fieldY, new Rotation2d(fieldTheta));

    }

    public void periodic() {
        super.periodic();
        System.out.println(fieldX + " " + fieldY);

    }

    @Override
    public void Swerve(double vx, double vy, double omega) {
        super.Swerve(vx, vy, omega);

        // how many times command runs in a second and divide by that number
        botX += (vx / 50.0);
        botY += (vy / 50.0);
        botTheta += (omega / 50.0);
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);

        convX = fieldX;
        convY = fieldY;
        convTheta = fieldTheta;

        botX = pose.getX();
        botY = pose.getY();
        botTheta = pose.getRotation().getDegrees();
    }

    @Override
    public double getYaw() {
        if (RobotBase.isSimulation()) {
            return botTheta;
        } else {
            return super.getYaw();
        }
    }

    @Override
    public Pose2d getPose() {

        if (RobotBase.isSimulation()) {
            return new Pose2d(botX, botY, new Rotation2d(botTheta));
        } else {
            return super.getPose();
        }
    }
}
