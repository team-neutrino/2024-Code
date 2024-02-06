package frc.robot.subsystems.simulation;

import com.revrobotics.REVPhysicsSim;

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

public class Swerve extends SwerveSubsystem {
    public Field2d m_Field2d = new Field2d();
    //update with correct values
    public DifferentialDrivetrainSim m_DrivetrainSim;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoubleTopic swerve_sim_speed_topic = inst.getDoubleTopic("swerve/sim_speed");
    DoubleTopic swerve_enc_speed_topic = inst.getDoubleTopic("swerve/encoder_speed");
    final DoublePublisher swerve_sim_speed_pub;
    final DoublePublisher swerve_speed_pub;

    public Swerve() {
        m_DrivetrainSim = new DifferentialDrivetrainSim(DCMotor.getNEO(8), 25.0, 25.0, 46.0, 0.5, 0.5, null);
        SmartDashboard.putData("Drivetrain", m_Field2d);

        swerve_sim_speed_pub = swerve_sim_speed_topic.publish();
        swerve_sim_speed_pub.setDefault(0.0);

        swerve_speed_pub = wheel_enc_speed_topic.publish();
        swerve_speed_pub.setDefault(0.0);

        swerve_target_speed_pub = wheel_target_speed_topic.publish();
        swerve_target_speed_pub.setDefault(0.0);
    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(m_shooter, DCMotor.getNEO(1));
        m_spark_max_pid_sim = new CanSparkMaxPidSim();
    }

    public void simulationPeriodic() {
        double motor_volts = 0.0;
        if (m_spark_max_pid_sim != null) {
            motor_volts = m_spark_max_pid_sim.runPid(WHEEL_P, WHEEL_I, WHEEL_D, WHEEL_FF, m_targetRPM,
                    m_flywheel_sim.getAngularVelocityRPM(), m_rpm_izone, 0.0, 100.0);
        }

        m_flywheel_sim.setInputVoltage(motor_volts);
        m_flywheel_sim.update(0.02);

        double rev_per_s = m_flywheel_sim.getAngularVelocityRPM();
        m_last_position_rev = m_last_position_rev + rev_per_s * 0.02;
        m_wheel_ligament.setAngle(m_last_position_rev * 6);

        wheel_sim_speed_pub.set(rev_per_s, NetworkTablesJNI.now());
    }

    public void periodic() {
        super.periodic();
        wheel_speed_pub.set(m_shooterEncoder.getVelocity(), NetworkTablesJNI.now());
        wheel_target_speed_pub.set(getTargetRPM(), NetworkTablesJNI.now());
    }

}

    
}
