package frc.robot.subsystems.simulation;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.SubsystemContainer;

public class Shooter extends ShooterSubsystem {
    Mechanism2d m_mech = SubsystemContainer.simOverview.m_mech;
    MechanismRoot2d m_root = m_mech.getRoot("shooter_root", 6, 28);
    public static MechanismLigament2d m_wheel_ligament;

    FlywheelSim m_flywheel_sim;
    double m_last_position_rev = 0.0;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoubleTopic wheel_sim_speed_topic = inst.getDoubleTopic("shooter/sim_speed");
    DoubleTopic wheel_enc_speed_topic = inst.getDoubleTopic("shooter/encoder_speed");
    DoubleTopic wheel_target_speed_topic = inst.getDoubleTopic("shooter/target_speed");
    DoubleTopic wheel_sim_speed_topic2 = inst.getDoubleTopic("shooter/sim_speed2");
    DoubleTopic wheel_enc_speed_topic2 = inst.getDoubleTopic("shooter/encoder_speed2");
    DoubleTopic wheel_target_speed_topic2 = inst.getDoubleTopic("shooter/target_speed2");
    final DoublePublisher wheel_sim_speed_pub;
    final DoublePublisher wheel_speed_pub;
    final DoublePublisher wheel_target_speed_pub;
    final DoublePublisher wheel_sim_speed_pub2;
    final DoublePublisher wheel_speed_pub2;
    final DoublePublisher wheel_target_speed_pub2;

    CanSparkMaxPidSim m_spark_max_pid_sim = null;

    Color8Bit blue = new Color8Bit(0, 0, 255);
    Color8Bit grey = new Color8Bit(128, 128, 128);

    public Shooter() {
        m_flywheel_sim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.02);

        wheel_sim_speed_pub = wheel_sim_speed_topic.publish();
        wheel_sim_speed_pub.setDefault(0.0);

        wheel_speed_pub = wheel_enc_speed_topic.publish();
        wheel_speed_pub.setDefault(0.0);

        wheel_target_speed_pub = wheel_target_speed_topic.publish();
        wheel_target_speed_pub.setDefault(0.0);

        wheel_sim_speed_pub2 = wheel_sim_speed_topic2.publish();
        wheel_sim_speed_pub2.setDefault(0.0);

        wheel_speed_pub2 = wheel_enc_speed_topic2.publish();
        wheel_speed_pub2.setDefault(0.0);

        wheel_target_speed_pub2 = wheel_target_speed_topic2.publish();
        wheel_target_speed_pub2.setDefault(0.0);
    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(m_shooterMotor, DCMotor.getNEO(1));
        m_spark_max_pid_sim = new CanSparkMaxPidSim();
        REVPhysicsSim.getInstance().addSparkMax(m_followerMotor, DCMotor.getNEO(1));
    }

    public void simulationPeriodic() {
        double motor_volts = 0.0;
        if (m_spark_max_pid_sim != null) {
            motor_volts = m_spark_max_pid_sim.runPid(WHEEL_P, WHEEL_I, WHEEL_D, WHEEL_FF, m_targetRPM,
                    m_flywheel_sim.getAngularVelocityRPM(), WHEEL_IZONE, 0.0, 100.0);
        }

        m_flywheel_sim.setInputVoltage(motor_volts);
        m_flywheel_sim.update(0.02);

        double rev_per_s = m_flywheel_sim.getAngularVelocityRPM();
        m_last_position_rev = m_last_position_rev + rev_per_s * 0.02;
        m_wheel_ligament.setAngle(m_last_position_rev * 6);

        wheel_sim_speed_pub.set(rev_per_s, NetworkTablesJNI.now());

        if (m_targetRPM > 0) {
            m_wheel_ligament.setColor(blue);
        } else {
            m_wheel_ligament.setColor(grey);
        }
    }

    public void periodic() {
        super.periodic();
        wheel_speed_pub.set(m_shooterEncoder.getVelocity(), NetworkTablesJNI.now());
        wheel_target_speed_pub.set(getTargetRPM(), NetworkTablesJNI.now());
        wheel_speed_pub2.set(m_followerEncoder.getVelocity(), NetworkTablesJNI.now());
        wheel_target_speed_pub2.set(getTargetRPM(), NetworkTablesJNI.now());
    }
}
