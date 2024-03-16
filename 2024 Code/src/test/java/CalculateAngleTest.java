
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.jzy3d.analysis.AWTAbstractAnalysis;
import org.jzy3d.analysis.AnalysisLauncher;
import org.jzy3d.chart.Chart;
import org.jzy3d.chart.ChartLauncher;
import org.jzy3d.chart.factories.AWTChartFactory;
import org.jzy3d.chart.factories.AWTPainterFactory;
import org.jzy3d.chart.factories.IChartFactory;
import org.jzy3d.chart.factories.IPainterFactory;
import org.jzy3d.colors.Color;
import org.jzy3d.colors.ColorMapper;
import org.jzy3d.colors.colormaps.ColorMapRainbow;
import org.jzy3d.maths.Range;
import org.jzy3d.plot3d.builder.Func3D;
import org.jzy3d.plot3d.builder.SurfaceBuilder;
import org.jzy3d.plot3d.builder.concrete.OrthonormalGrid;
import org.jzy3d.plot3d.primitives.Shape;
import org.jzy3d.plot3d.rendering.canvas.Quality;

import com.jogamp.opengl.GLCapabilities;
import com.jogamp.opengl.GLProfile;
import com.jogamp.opengl.awt.GLCanvas;

import java.util.ArrayList;
import java.io.File;
import java.io.IOException;
import java.text.DecimalFormat;

import frc.robot.util.CalculateAngle;
import frc.robot.util.PolarCoord;

public class CalculateAngleTest {

    static final double DELTA = 1e-4;
    private static final DecimalFormat two_points = new DecimalFormat("0.00");
    protected Chart chart;

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
        System.out.println("**************THIS " + m_calculate_angle.InterpolateAngle(new PolarCoord(0.0, 0.0)));
        assertEquals(1.22654, m_calculate_angle.InterpolateAngle(new PolarCoord(0.0, 0.0)), DELTA);
    }

    @Test
    void interpolatePointClosestToSpeaker() {
        assertEquals(1.22654, m_calculate_angle.InterpolateAngle(new PolarCoord(1.625, 0.0)), DELTA);
    }

    public class PointAnglePair {
        private PolarCoord pt;
        private double angle;

        PointAnglePair(double x, double y, CalculateAngle calc) {
            pt = new PolarCoord(x, y);
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
            points.add(new PointAnglePair(x, Math.PI / 6, m_calculate_angle));
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

    @Test
    void visualizeInterpolation() throws IOException {

        // Define a function to plot
        Func3D func = new Func3D((x, y) -> x * Math.sin(x * y));
        Range range = new Range(-3, 3);
        int steps = 80;

        // Create the object to represent the function over the given range.
        final Shape surface = new SurfaceBuilder().orthonormal(new OrthonormalGrid(range, steps), func);
        surface
                .setColorMapper(new ColorMapper(new ColorMapRainbow(), surface, new Color(1, 1, 1, .5f)));
        surface.setFaceDisplayed(true);
        surface.setWireframeDisplayed(true);
        surface.setWireframeColor(Color.BLACK);

        // Create a chart
        // GLCapabilities c = new GLCapabilities(GLProfile.get(GLProfile.GL4));
        // IPainterFactory p = new AWTPainterFactory(c);
        // IChartFactory f = new AWTChartFactory(p);
        IChartFactory f = new AWTChartFactory();

        chart = f.newChart(Quality.Advanced().setHiDPIEnabled(true));
        chart.getScene().getGraph().add(surface);

        ChartLauncher.instructions();
        // ChartLauncher.openChart(chart);
        // ChartLauncher.openStaticChart(chart);
        String FILENAME = "/home/andrew/visualizeInterpolation.png";
        ChartLauncher.screenshot(chart, FILENAME);
    }
}
