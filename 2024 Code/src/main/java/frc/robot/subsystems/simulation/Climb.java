package frc.robot.subsystems.simulation;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb extends ClimbSubsystem {

    Mechanism2d m_mech1 = new Mechanism2d(3, 3);
    MechanismRoot2d m_root1 = m_mech1.getRoot("chassis", 1.5, 0);
    MechanismLigament2d m_elevator_ligament1;
    ElevatorSim m_elevator_sim1;
    double m_last_position_rev1 = 0.0;
    Color8Bit green = new Color8Bit(0, 255, 0);
    Color8Bit red = new Color8Bit(255, 0, 0);
    Color8Bit color = new Color8Bit(100, 100, 0);

    NetworkTableInstance inst1 = NetworkTableInstance.getDefault();
    DoubleTopic elevator_sim_position_topic1 = inst1.getDoubleTopic("climb1/sim_position1");
    DoubleTopic elevator_enc_position_topic1 = inst1.getDoubleTopic("climb1/encoder_position1");
    final DoublePublisher elevator_sim_position_pub1;
    final DoublePublisher elevator_position_pub1;

    public Climb() {

        m_elevator_ligament1 = m_root1.append(new MechanismLigament2d("elevator1", 2, 0));
        m_elevator_sim1 = new ElevatorSim(DCMotor.getNEO(1), 25.0, 36.29, 0.022225, 0.00, 0.5334, true, 0.00);
        SmartDashboard.putData("Climb1", m_mech1);

        elevator_sim_position_pub1 = elevator_sim_position_topic1.publish();
        elevator_sim_position_pub1.setDefault(0.0);

        elevator_position_pub1 = elevator_enc_position_topic1.publish();
        elevator_position_pub1.setDefault(0.0);
    }

    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(m_climb1, DCMotor.getNEO(1));
    }

    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevator_sim1.setInputVoltage(m_climb1.getAppliedOutput() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_elevator_sim1.update(0.020);
        elevator_sim_position_pub1.set(m_elevator_sim1.getPositionMeters());

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        // m_climbEncoder1.setPosition(m_elevator_sim1.getPositionMeters());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevator_sim1.getCurrentDrawAmps()));

        m_root1.setPosition(1.5, m_elevator_sim1.getPositionMeters());

        m_elevator_ligament1.setColor(color);
        if (m_elevator_sim1.hasHitUpperLimit()) {
            m_elevator_ligament1.setColor(green);
        } else if (m_elevator_sim1.hasHitLowerLimit()) {
            m_elevator_ligament1.setColor(red);
        } else {
            m_elevator_ligament1.setColor(color);
        }
    }

    public void periodic() {
        super.periodic();
        elevator_position_pub1.set(m_climbEncoder1.getVelocity(), NetworkTablesJNI.now());
    }

}