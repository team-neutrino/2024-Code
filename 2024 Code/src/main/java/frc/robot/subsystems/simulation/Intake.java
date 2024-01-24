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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends IntakeSubsystem {
    Mechanism2d m_mech2 = new Mechanism2d(3, 3);
    MechanismRoot2d m_root2 = m_mech2.getRoot("chassis", 2, 2);
    MechanismLigament2d m_wheel_ligament2;

    FlywheelSim m_flywheel_sim2;
    double m_last_position_rev2 = 0.0;

    NetworkTableInstance inst2 = NetworkTableInstance.getDefault();
    DoubleTopic wheel_sim_speed_topic2 = inst2.getDoubleTopic("/wheel/sim/speed");
    DoubleTopic wheel_enc_speed_topic2 = inst2.getDoubleTopic("wheel/speed");
    final DoublePublisher wheel_sim_speed_pub2;
    final DoublePublisher wheel_speed_pub2;

    public Intake() {
        m_wheel_ligament2 = m_root2.append(new MechanismLigament2d("wheel", 1, 0));
        m_flywheel_sim2 = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.02);
        SmartDashboard.putData("Mech2d", m_mech2);

        wheel_sim_speed_pub2 = wheel_sim_speed_topic2.publish();
        wheel_sim_speed_pub2.setDefault(0.0);

        wheel_speed_pub2 = wheel_enc_speed_topic2.publish();
        wheel_speed_pub2.setDefault(0.0);
    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(m_intakeMotor, DCMotor.getNEO(1));
    }

    public void simulationPeriodic() {
        double motor_volts = m_intakeMotor.getAppliedOutput() * m_intakeMotor.getBusVoltage();
        m_flywheel_sim2.setInputVoltage(motor_volts);
        m_flywheel_sim2.update(0.02);

        double rev_per_s = m_flywheel_sim2.getAngularVelocityRPM();
        // System.out.println("flywheel velocity ()RPM " +
        // m_flywheel_sim.getAngularVelocityRPM());
        m_last_position_rev2 = m_last_position_rev2 + rev_per_s * 0.02;
        m_wheel_ligament2.setAngle(m_last_position_rev2 * 6);

        wheel_sim_speed_pub2.set(rev_per_s, NetworkTablesJNI.now());
    }

    public void periodic() {
        super.periodic();
        wheel_speed_pub2.set(m_intakeEncoder.getVelocity(), NetworkTablesJNI.now());
    }

}
