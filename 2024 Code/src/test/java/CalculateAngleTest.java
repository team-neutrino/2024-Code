
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import frc.robot.util.CalculateAngle;

public class CalculateAngleTest {

    static final double DELTA = 1e-4;
    CalculateAngle m_calculate_angle;

    @BeforeEach
    void setup() {
        m_calculate_angle = new CalculateAngle();
    }

    @AfterEach
    void shutdown() throws Exception {
    }

    @Test
    void defaultInterpolation() {
        assertEquals(
                1.22654, m_calculate_angle.InterpolateAngle(), DELTA);
    }
}
