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
import frc.robot.util.SubsystemContainer;

public class IntakeSimulation extends IntakeSubsystem {
    Mechanism2d m_mech = SubsystemContainer.simOverview.m_mech;
    MechanismRoot2d m_intakeRoot = m_mech.getRoot("intake_root", 26, 10);

    MechanismRoot2d m_beamRoot = m_mech.getRoot("beam", 2, 2);

    Mechanism2d m_beamBreak = new Mechanism2d(1, 1);
    MechanismLigament2d m_intakeWheelLigament;
    static MechanismLigament2d m_beambreakLigament;

    MechanismRoot2d m_indexRoot = m_mech.getRoot("index_root", 14, 20);
    MechanismLigament2d m_indexWheelLigament;

    Color8Bit green = new Color8Bit(0, 255, 0);
    Color8Bit red = new Color8Bit(255, 0, 0);
    Color8Bit color = new Color8Bit(100, 100, 0);

    FlywheelSim m_intakeFlywheelSim;
    double m_intakeLastPosition = 0.0;

    FlywheelSim m_indexFlywheelSim;
    double m_indexLastPosition = 0.0;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoubleTopic intakeWheelSimSpeed_topic = inst.getDoubleTopic("Intake/sim_speed");
    DoubleTopic intakeWheelEncSpeed_topic = inst.getDoubleTopic("Intake/encoder_speed");
    BooleanTopic beambreakStatus_topic = inst.getBooleanTopic("Intake/beam_break");
    final DoublePublisher intakeWheelSimSpeed_pub;
    final DoublePublisher intakeWheelEncSpeed_pub;
    final BooleanPublisher beambreakStatus_pub;

    DoubleTopic indexWheelSimSpeed_topic = inst.getDoubleTopic("Index/sim_speed");
    DoubleTopic indexWheelEncSpeed_topic = inst.getDoubleTopic("Index/encoder_speed");
    final DoublePublisher indexWheelSimSpeed_pub;
    final DoublePublisher indexWheelEncSpeed_pub;

    public IntakeSimulation() {
        m_intakeFlywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, 0.002);

        intakeWheelSimSpeed_pub = intakeWheelSimSpeed_topic.publish();
        intakeWheelSimSpeed_pub.setDefault(0.0);

        intakeWheelEncSpeed_pub = intakeWheelEncSpeed_topic.publish();
        intakeWheelEncSpeed_pub.setDefault(0.0);

        indexWheelSimSpeed_pub = indexWheelSimSpeed_topic.publish();
        indexWheelSimSpeed_pub.setDefault(0.0);

        indexWheelEncSpeed_pub = indexWheelEncSpeed_topic.publish();
        indexWheelEncSpeed_pub.setDefault(0.0);
        beambreakStatus_pub = beambreakStatus_topic.publish();
        beambreakStatus_pub.setDefault(false);

        m_intakeWheelLigament = m_intakeRoot.append(new MechanismLigament2d("intake", 2, 0));
    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(m_intakeMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(m_indexMotor, DCMotor.getNEO(1));
    }

    public void simulationPeriodic() {
        double motor_volts = m_intakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage();

        m_intakeFlywheelSim.setInputVoltage(motor_volts);
        m_intakeFlywheelSim.update(0.02);

        double revPerSec = m_intakeFlywheelSim.getAngularVelocityRPM();
        m_intakeLastPosition = m_intakeLastPosition + revPerSec * 0.02;
        m_intakeWheelLigament.setAngle(m_intakeLastPosition * 6);

        intakeWheelSimSpeed_pub.set(revPerSec, NetworkTablesJNI.now());

        if (motor_volts > 0.0) {
            m_intakeWheelLigament.setColor(green);
        } else if (motor_volts < 0.0) {
            m_intakeWheelLigament.setColor(red);
        } else {
            m_intakeWheelLigament.setColor(color);
        }
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