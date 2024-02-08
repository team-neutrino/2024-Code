package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimulationOverview {
    public Mechanism2d m_mech = new Mechanism2d(36, 48);

    public SimulationOverview() {
        SmartDashboard.putData("robot", m_mech);
    }

    public void simulationInit() {
    }

    public void simulationPeriodic() {
    }
}
