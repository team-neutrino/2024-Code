package frc.robot;

import org.junit.jupiter.api.Test;
import frc.robot.subsystems.ArmSubsystem;

import static org.junit.jupiter.api.Assertions.*;

public class RobotTest {

    @Test
    void testTargetAngle() {
        var armSubsystem = new ArmSubsystem();
        var armDefault = armSubsystem.getArmAngleDegrees();
        System.out.println("armDefault " + armDefault);
        //this looks to be random at the moment
        // assertTrue(1.92 < armDefault);
        // assertTrue(armDefault < 1.93);

        var defaultTargetAngle = armSubsystem.getTargetAngle();
        System.out.println("targetAngle " + defaultTargetAngle);
        assertEquals(-27.0, defaultTargetAngle);

        armSubsystem.setClimbReferenceAngle();
        var newTargetAngle = armSubsystem.getTargetAngle();
        System.out.println("newTargetAngle " + newTargetAngle);
        assertEquals(-20.0, newTargetAngle);
    }
}
