package frc.robot.subsystems.wrapper;

import java.lang.Math;

public class CanSparkMaxPidSim {
    public double accumulatedError = 0.0;
    public double prevError = 0.0;

    public CanSparkMaxPidSim() {
    }

    public double runPid(double P, double I, double D, double feedForward, double target, double actual,
            double iZone, double minOutput, double maxOutput) {
        double error = target - actual;
        double calculatedP = P * error;

        double calculatedI = 0.0;
        if (Math.abs(error) <= iZone || Double.compare(iZone, 0.0) == 0) {
            accumulatedError += error;
            calculatedI = I * accumulatedError;
        } else {
            accumulatedError = 0.0;
        }

        double calculatedD = D * (error - prevError);
        prevError = error;

        double calculatedFeedForward = feedForward * target;

        return Math.min(
                Math.max(calculatedP + calculatedI + calculatedD + calculatedFeedForward, minOutput),
                maxOutput);
    }
}
