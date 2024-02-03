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

public class IntakeSimulation extends IntakeSubsystem {
    Mechanism2d m_intakeMech = new Mechanism2d(3, 3);
    MechanismRoot2d m_intakeRoot = m_intakeMech.getRoot("chassis", 2, 2);
    MechanismLigament2d m_intakeWheelLigament;

    FlywheelSim m_intakeFlywheelSim;
    double m_lastPosition = 0.0;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoubleTopic intakeWheelSimSpeed_topic = inst.getDoubleTopic("intake/sim_speed");
    DoubleTopic intakeWheelEncSpeed_topic = inst.getDoubleTopic("intake/encoder_speed");
    DoubleTopic intakeWheelTargetSpeed_topic = inst.getDoubleTopic("intake/target_speed");
    final DoublePublisher intakeWheelSimSpeed_pub;
    final DoublePublisher intakeWheelEncSpeed_pub;
    final DoublePublisher intakeWheelTargetSpeed_pub;
    CanSparkMaxPidSim m_spark_max_pid_sim = null;

    public IntakeSimulation() {
        m_intakeFlywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, 0.02);
        SmartDashboard.putData("intake", m_intakeMech);

        intakeWheelSimSpeed_pub = intakeWheelSimSpeed_topic.publish();
        intakeWheelSimSpeed_pub.setDefault(0.0);

        intakeWheelEncSpeed_pub = intakeWheelEncSpeed_topic.publish();
        intakeWheelEncSpeed_pub.setDefault(0.0);

        intakeWheelTargetSpeed_pub = intakeWheelTargetSpeed_topic.publish();
        intakeWheelTargetSpeed_pub.setDefault(0.0);
    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(m_intakeMotor, DCMotor.getNEO(1));
        m_spark_max_pid_sim = new CanSparkMaxPidSim();
    }

    public void simulationPeriodic() {
        double motor_volts = 0.0;

        m_intakeFlywheelSim.setInputVoltage(motor_volts);
        m_intakeFlywheelSim.update(0.02);

        double revPerSec = m_intakeFlywheelSim.getAngularVelocityRPM();
        m_lastPosition = m_lastPosition + revPerSec * 0.02;
        m_intakeWheelLigament.setAngle(m_lastPosition * 6);

        intakeWheelSimSpeed_pub.set(revPerSec, NetworkTablesJNI.now());
    }

    public void periodic() {
        super.periodic();
        intakeWheelEncSpeed_pub.set(m_intakeEncoder.getVelocity(), NetworkTablesJNI.now());
    }
}