package frc.robot.subsystems.simulation;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.SubsystemContainer;

public class IntakeSimulation extends IntakeSubsystem {
    Mechanism2d m_mech = SubsystemContainer.simOverview.m_mech;
    MechanismRoot2d m_intakeRoot = m_mech.getRoot("chassis", 2, 4);
    MechanismLigament2d m_intakeWheelLigament;
    Color8Bit green = new Color8Bit(0, 255, 0);
    Color8Bit red = new Color8Bit(255, 0, 0);
    Color8Bit color = new Color8Bit(100, 100, 0);

    FlywheelSim m_intakeFlywheelSim;
    double m_lastPosition = 0.0;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoubleTopic intakeWheelSimSpeed_topic = inst.getDoubleTopic("Intake/sim_speed");
    DoubleTopic intakeWheelEncSpeed_topic = inst.getDoubleTopic("Intake/encoder_speed");
    final DoublePublisher intakeWheelSimSpeed_pub;
    final DoublePublisher intakeWheelEncSpeed_pub;
    CanSparkMaxPidSim m_SparkMaxPidSim = null;

    public IntakeSimulation() {
        m_intakeFlywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, 0.002);

        intakeWheelSimSpeed_pub = intakeWheelSimSpeed_topic.publish();
        intakeWheelSimSpeed_pub.setDefault(0.0);

        intakeWheelEncSpeed_pub = intakeWheelEncSpeed_topic.publish();
        intakeWheelEncSpeed_pub.setDefault(0.0);

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

        if (motor_volts > 0.0) {
            m_intakeWheelLigament.setColor(green);
        } else if (motor_volts < 0.0) {
            m_intakeWheelLigament.setColor(red);
        } else {
            m_intakeWheelLigament.setColor(color);
        }
    }

    public void periodic() {
        super.periodic();
        intakeWheelEncSpeed_pub.set(m_intakeEncoder.getVelocity(), NetworkTablesJNI.now());
    }
}