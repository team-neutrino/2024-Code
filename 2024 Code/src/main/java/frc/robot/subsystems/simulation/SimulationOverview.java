package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimulationOverview {
    public Mechanism2d m_mech = new Mechanism2d(36, 48);
    // MechanismRoot2d m_armRoot = m_mech.getRoot("shoulder", 6, 22);
    // MechanismLigament2d m_upperArm;
    // MechanismRoot2d m_climbRoot = m_mech.getRoot("chassis", 18, 0);
    // MechanismLigament2d m_elevator_ligament1;

    public SimulationOverview() {
        // m_upperArm = m_armRoot.append(new MechanismLigament2d("upperarm", 24, 0));
        // m_elevator_ligament1 = m_climbRoot.append(new MechanismLigament2d("elevator",
        // 12, 0));
        SmartDashboard.putData("robot", m_mech);

    }

    public void simulationInit() {
    }

    public void simulationPeriodic() {
    }

}
