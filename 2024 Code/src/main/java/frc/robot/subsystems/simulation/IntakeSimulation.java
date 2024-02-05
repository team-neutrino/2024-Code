package frc.robot.subsystems.simulation;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSimulation extends IntakeSubsystem {
    Mechanism2d m_intakeMech = new Mechanism2d(3, 3);
    Mechanism2d m_beamBreak = new Mechanism2d(1, 1);
    MechanismRoot2d m_intakeRoot = m_intakeMech.getRoot("chassis", 2, 2);
    MechanismRoot2d m_beamRoot = m_beamBreak.getRoot("beam", 2, 2);
    MechanismLigament2d m_intakeWheelLigament;
    static MechanismLigament2d m_beambreakLigament;

    FlywheelSim m_intakeFlywheelSim;
    double m_lastPosition = 0.0;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoubleTopic intakeWheelSimSpeed_topic = inst.getDoubleTopic("Intake/sim_speed");
    DoubleTopic intakeWheelEncSpeed_topic = inst.getDoubleTopic("Intake/encoder_speed");
    BooleanTopic beambreakStatus_topic = inst.getBooleanTopic("Intake/beam_break");
    final DoublePublisher intakeWheelSimSpeed_pub;
    final DoublePublisher intakeWheelEncSpeed_pub;
    final BooleanPublisher beambreakStatus_pub;
    CanSparkMaxPidSim m_SparkMaxPidSim = null;
    Color8Bit green = new Color8Bit(0, 255, 0);
    Color8Bit red = new Color8Bit(255, 0, 0);

    public IntakeSimulation() {
        m_intakeFlywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, 0.002);
        SmartDashboard.putData("Intake", m_intakeMech);

        intakeWheelSimSpeed_pub = intakeWheelSimSpeed_topic.publish();
        intakeWheelSimSpeed_pub.setDefault(0.0);

        intakeWheelEncSpeed_pub = intakeWheelEncSpeed_topic.publish();
        intakeWheelEncSpeed_pub.setDefault(0.0);

        beambreakStatus_pub = beambreakStatus_topic.publish();
        beambreakStatus_pub.setDefault(false);

        m_intakeWheelLigament = m_intakeRoot.append(new MechanismLigament2d("intake", 2, 0));
    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(m_intakeMotor, DCMotor.getNEO(1));
        m_SparkMaxPidSim = new CanSparkMaxPidSim();
    }

    public void simulationPeriodic() {
        double motor_volts = m_intakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage();

        m_intakeFlywheelSim.setInputVoltage(motor_volts);
        m_intakeFlywheelSim.update(0.02);

        double revPerSec = m_intakeFlywheelSim.getAngularVelocityRPM();
        m_lastPosition = m_lastPosition + revPerSec * 0.02;
        m_intakeWheelLigament.setAngle(m_lastPosition * 6);
        intakeWheelSimSpeed_pub.set(revPerSec, NetworkTablesJNI.now());
        if (getBeamBreak()) {
            m_beambreakLigament.setColor(green);
        } else {
            m_beambreakLigament.setColor(red);
        }
    }

    public void periodic() {
        super.periodic();
        intakeWheelEncSpeed_pub.set(m_intakeEncoder.getVelocity(), NetworkTablesJNI.now());
        beambreakStatus_pub.set(getBeamBreak());
    }
}