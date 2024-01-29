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
import frc.robot.subsystems.ClimbSubsystem;

public class Climb extends ClimbSubsystem {

    Mechanism2d m_mech1 = new Mechanism2d(3, 3);
    MechanismRoot2d m_root1 = m_mech1.getRoot("chassis", 2, 2);
    MechanismLigament2d m_elevator_ligament1;
    ElevatorSim m_elevator_sim1;
    double m_last_position_rev1 = 0.0;

    Mechanism2d m_mech2 = new Mechanism2d(3, 3);
    MechanismRoot2d m_root2 = m_mech2.getRoot("chassis", 2, 2);
    MechanismLigament2d m_elevator_ligament2;
    ElevatorSim m_elevator_sim2;
    double m_last_position_rev2 = 0.0;

    NetworkTableInstance inst1 = NetworkTableInstance.getDefault();
    DoubleTopic elevator_sim_speed_topic1 = inst1.getDoubleTopic("climb1/sim_speed1");
    DoubleTopic elevator_enc_speed_topic1 = inst1.getDoubleTopic("climb1/encoder_speed1");
    DoubleTopic elevator_target_speed_topic1 = inst1.getDoubleTopic("climb1/target_speed1");
    final DoublePublisher elevator_sim_speed_pub1;
    final DoublePublisher elevator_speed_pub1;
    final DoublePublisher elevator_target_speed_pub1;

    NetworkTableInstance inst2 = NetworkTableInstance.getDefault();
    DoubleTopic elevator_sim_speed_topic2 = inst2.getDoubleTopic("climb2/sim_speed2");
    DoubleTopic elevator_enc_speed_topic2 = inst2.getDoubleTopic("climb2/encoder_speed2");
    DoubleTopic elevator_target_speed_topic2 = inst2.getDoubleTopic("climb2/target_speed2");
    final DoublePublisher elevator_sim_speed_pub2;
    final DoublePublisher elevator_speed_pub2;
    final DoublePublisher elevator_target_speed_pub2;

    public Climb() {

        // fix smartdashboard so equals one button
        m_elevator_ligament1 = m_root1.append(new MechanismLigament2d("elevator1", 2, 0));
        m_elevator_sim1 = new ElevatorSim(DCMotor.getNEO(1), 1.0, 4.0, 0.03, 0.01, 1.0, true, 0.01);
        SmartDashboard.putData("Climb1", m_mech1);

        m_elevator_ligament2 = m_root2.append(new MechanismLigament2d("elevator2", 1, 0));
        m_elevator_sim2 = new ElevatorSim(DCMotor.getNEO(1), 1.0, 4.0, 0.03, 0.01, 1.0, true, 0.01);
        SmartDashboard.putData("Climb2", m_mech2);

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
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevator_sim1.setInput(m_armEncoder1.getVelocity() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_elevator_sim1.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        m_armEncoder1.setPosition(m_elevator_sim1.getPositionMeters());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevator_sim1.getCurrentDrawAmps()));

        m_elevator_sim2.setInput(m_armEncoder2.getVelocity() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_elevator_sim2.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        m_armEncoder2.setPosition(m_elevator_sim2.getPositionMeters());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevator_sim2.getCurrentDrawAmps()));
    }

    public void periodic() {
        super.periodic();
        elevator_speed_pub1.set(m_armEncoder1.getVelocity(), NetworkTablesJNI.now());
        elevator_speed_pub2.set(m_armEncoder2.getVelocity(), NetworkTablesJNI.now());
    }

}