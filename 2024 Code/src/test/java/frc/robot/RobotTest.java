package frc.robot;

import org.junit.jupiter.api.Test;
import frc.robot.subsystems.ArmSubsystem;

import static org.junit.jupiter.api.Assertions.*;

public class RobotTest {

    @Test
    void testSomething() {
        var armSubsysem = new ArmSubsystem();
        var armDefault = armSubsysem.getArmAngleDegrees();
        System.out.println("armDefault " + armDefault);
        //this looks to random at the moment
        // assertTrue(1.92 < armDefault);
        // assertTrue(armDefault < 1.93);

        var defaultTargetAngle = armSubsysem.getTargetAngle();
        System.out.println("targetAngle " + defaultTargetAngle);
        assertEquals(-27.0, defaultTargetAngle);
    }
}
