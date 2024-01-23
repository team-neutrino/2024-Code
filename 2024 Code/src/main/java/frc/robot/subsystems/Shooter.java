package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends ShooterSubsystem {
    Mechanism2d m_mech = new Mechanism2d(3, 3);
    MechanismRoot2d m_root = m_mech.getRoot("chassis", 2, 2);
    MechanismLigament2d m_wheel_ligament;

    FlywheelSim m_flywheel_sim;
    double m_last_position_rev = 0.0;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoubleTopic wheel_sim_speed_topic = inst.getDoubleTopic("/wheel/sim/speed");
    DoubleTopic wheel_enc_speed_topic = inst.getDoubleTopic("wheel/speed");
    final DoublePublisher wheel_sim_speed_pub;
    final DoublePublisher wheel_speed_pub;

    public Shooter() {
        m_wheel_ligament = m_root.append(new MechanismLigament2d("wheel", 1, 0));
        m_flywheel_sim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.02);
        SmartDashboard.putData("Mech2d", m_mech);

        wheel_sim_speed_pub = wheel_sim_speed_topic.publish();
        wheel_sim_speed_pub.setDefault(0.0);

        wheel_speed_pub = wheel_enc_speed_topic.publish();
        wheel_speed_pub.setDefault(0.0);
    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(m_shooter, DCMotor.getNEO(1));
    }

    public void simulationPeriodic() {
        double motor_volts = m_shooter.getAppliedOutput() * m_shooter.getBusVoltage();
        m_flywheel_sim.setInputVoltage(motor_volts);
        m_flywheel_sim.update(0.02);

        double rev_per_s = m_flywheel_sim.getAngularVelocityRPM();
        System.out.println("flywheel velocity ()RPM " + m_flywheel_sim.getAngularVelocityRPM());
        m_last_position_rev = m_last_position_rev + rev_per_s * 0.02;
        m_wheel_ligament.setAngle(m_last_position_rev * 6);

        wheel_sim_speed_pub.set(rev_per_s, NetworkTablesJNI.now());
    }

    public void periodic() {
        super.periodic();
        wheel_speed_pub.set(m_shooterEncoder.getVelocity(), NetworkTablesJNI.now());
    }

}
