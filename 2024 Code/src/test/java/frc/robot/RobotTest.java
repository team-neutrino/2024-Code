package frc.robot;

import org.junit.jupiter.api.Test;
import frc.robot.subsystems.ArmSubsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class RobotTest {

    @Test
    void testSomething() {
        assertEquals(10, 5+5);
        var armSubsysem = new ArmSubsystem();
        System.out.println("actual " + armSubsysem.getArmAngleDegrees());
        assertEquals(armSubsysem.getArmAngleDegrees(), 100);
    }
}
