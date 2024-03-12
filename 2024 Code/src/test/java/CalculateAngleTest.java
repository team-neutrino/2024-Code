
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.text.DecimalFormat;

import frc.robot.util.CalculateAngle;

public class CalculateAngleTest {

    static final double DELTA = 1e-4;
    private static final DecimalFormat two_points = new DecimalFormat("0.00");

    // the thing under test
    CalculateAngle m_calculate_angle;

    @BeforeEach
    void setup() {
        m_calculate_angle = new CalculateAngle();
    }

    @AfterEach
    void shutdown() throws Exception {
    }

    @Test
    void interpolateZeroPoint() {
        assertEquals(1.22654, m_calculate_angle.InterpolateAngle(new Point2D.Double(0.0, 0.0)), DELTA);
    }

    @Test
    void interpolatePointClosestToSpeaker() {
        assertEquals(1.22654, m_calculate_angle.InterpolateAngle(new Point2D.Double(1.625, 0.0)), DELTA);
    }

    public class PointAnglePair {
        private Point2D pt;
        private double angle;

        PointAnglePair(double x, double y, CalculateAngle calc) {
            pt = new Point2D.Double(x, y);
            angle = calc.InterpolateAngle(pt);
        }

        public double Angle() {
            return angle;
        }

        public String toString() {
            return "x: " + two_points.format(pt.getX()) + "  y: " + two_points.format(pt.getY()) + "  angle: "
                    + two_points.format(angle);
        }
    };

    @Test
    void interpolateAlongXAndVerifyAngleIncreases() {

        final double START_POINT = 1.6;
        final double END_POINT = 4.0;
        final double INTERVAL = 0.05;

        // populate points and calculate angles
        ArrayList<PointAnglePair> points = new ArrayList<PointAnglePair>();
        for (double x = START_POINT; x < END_POINT; x = x + INTERVAL) {
            points.add(new PointAnglePair(x, 0.0, m_calculate_angle));
        }

        // verify that angle increases
        for (int i = 0; i < points.size() - 1; i++) {
            PointAnglePair p_small = points.get(i);
            PointAnglePair p_big = points.get(i + 1);

            String SUFFIX = p_small.Angle() > p_big.Angle() ? " x" : "";
            System.out.println(p_small + SUFFIX);

            assertTrue(p_big.Angle() > p_small.Angle(),
                    "big point:" + p_big.Angle() + " small point:" + p_small.Angle());
        }
    }
}
