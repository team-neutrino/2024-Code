package frc.robot.subsystems.simulation;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb extends ClimbSubsystem {

    Mechanism2d m_mech = new Mechanism2d(6, 6);
    MechanismRoot2d m_root1 = m_mech.getRoot("chassis", 3, 3);
    MechanismRoot2d m_root2 = m_mech.getRoot("chassis", 1, 1);
    MechanismLigament2d m_elevator_ligament1;
    MechanismLigament2d m_elevator_ligament2;

    ElevatorSim m_elevator_sim1;
    ElevatorSim m_elevator_sim2;
    double m_last_position_rev1 = 0.0;
    double m_last_position_rev2 = 0.0;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoubleTopic elevator_sim_speed_topic1 = inst.getDoubleTopic("climb1/sim_speed1");
    DoubleTopic elevator_enc_speed_topic1 = inst.getDoubleTopic("climb1/encoder_speed1");
    DoubleTopic elevator_target_speed_topic1 = inst.getDoubleTopic("climb1/target_speed1");
    final DoublePublisher elevator_sim_speed_pub1;
    final DoublePublisher elevator_speed_pub1;
    final DoublePublisher elevator_target_speed_pub1;

    NetworkTableInstance inst2 = NetworkTableInstance.getDefault();
    DoubleTopic elevator_sim_speed_topic2 = inst.getDoubleTopic("climb2/sim_speed2");
    DoubleTopic elevator_enc_speed_topic2 = inst.getDoubleTopic("climb2/encoder_speed2");
    DoubleTopic elevator_target_speed_topic2 = inst.getDoubleTopic("climb2/target_speed2");
    final DoublePublisher elevator_sim_speed_pub2;
    final DoublePublisher elevator_speed_pub2;
    final DoublePublisher elevator_target_speed_pub2;

    public Climb() {

        //fix smartdashboard so equals one button
        m_elevator_ligament1 = m_root1.append(new MechanismLigament2d("elevator1", 2, 0));
        m_elevator_sim1 = new Ele
        SmartDashboard.putData("Climb1", m_mech);

        m_elevator_ligament2 = m_root2.append(new MechanismLigament2d("elevator2", 1, 0));
        m_elevator_sim2 = new 
        SmartDashboard.putData("Climb1", m_mech);

        elevator_sim_speed_pub1 = elevator_sim_speed_topic1.publish();
        elevator_sim_speed_pub1.setDefault(0.0);

        elevator_sim_speed_pub2 = elevator_sim_speed_topic2.publish();
        elevator_sim_speed_pub2.setDefault(0.0);

        elevator_target_speed_pub1 = elevator_target_speed_topic1.publish();
        elevator_target_speed_pub1.setDefault(0.0);

        elevator_target_speed_pub2 = elevator_target_speed_topic2.publish();
        elevator_target_speed_pub2.setDefault(0.0);

        elevator_speed_pub1 = elevator_enc_speed_topic1.publish();
        elevator_speed_pub1.setDefault(0.0);

        elevator_speed_pub2 = elevator_enc_speed_topic2.publish();
        elevator_speed_pub2.setDefault(0.0);
    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(m_climbArm1, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(m_climbArm2, DCMotor.getNEO(1));
    }

    public void simulationPeriodic() {

        // redo logic?
        double motor_volts1 = m_climbArm1.getAppliedOutput() * m_climbArm1.getBusVoltage();
        m_elevator_sim1.setInputVoltage(motor_volts1);
        m_elevator_sim1.update(0.02);

        double motor_volts2 = m_climbArm2.getAppliedOutput() * m_climbArm2.getBusVoltage();
        m_elevator_sim2.setInputVoltage(motor_volts2);
        m_elevator_sim2.update(0.02);

        // redo math
        double rev_per_s1 = m_elevator_sim1.getAngularVelocityRPM();
        m_last_position_rev1 = m_last_position_rev1 + rev_per_s1 * 0.02;
        m_elevator_ligament1.setAngle(m_last_position_rev1 * 6);

        // redo math
        double rev_per_s2 = m_elevator_sim2.getAngularVelocityRPM();
        m_last_position_rev2 = m_last_position_rev2 + rev_per_s2 * 0.02;
        m_elevator_ligament2.setAngle(m_last_position_rev2 * 6);

        elevator_sim_speed_pub1.set(rev_per_s1, NetworkTablesJNI.now());
        elevator_sim_speed_pub2.set(rev_per_s2, NetworkTablesJNI.now());
    }

    public void periodic() {
        super.periodic();
        elevator_speed_pub1.set(m_armEncoder1.getVelocity(), NetworkTablesJNI.now());
        elevator_speed_pub2.set(m_armEncoder2.getVelocity(), NetworkTablesJNI.now());
    }

}