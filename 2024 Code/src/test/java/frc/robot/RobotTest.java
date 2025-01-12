package frc.robot;

import org.junit.jupiter.api.Test;
import frc.robot.subsystems.ArmSubsystem;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class RobotTest {

    @Test
    void testSomething() {
        assertEquals(10, 5+5);
        var armSubsysem = new ArmSubsystem();
        var armDeafault = armSubsysem.getArmAngleDegrees();
        System.out.println("armDefault " + armDefault);
        asserTrue(1.92 < armDefault);
        asserTrue(armDefault < 1.93);
    }
}
