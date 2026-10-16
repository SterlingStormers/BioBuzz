package com.slipstream;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Curve;

public class VelocityProfile {

    private final SlipstreamConfig config;

    public VelocityProfile(SlipstreamConfig config) {
        this.config = config;
    }

    /**
     * Walks backward from endpoint (t=1.0) to currentT.
     * At each sample, applies 3-axis friction ellipse to find max feasible speed.
     * Returns max speed at currentT.
     */
    public double compute(Path path, double currentT) {

        Curve curve = path.curve;
        double t = currentT;

        int spaces = (int) Math.round((1.0 - t) * 100);
        double[] samples = new double[spaces + 1];

        for (int i = spaces; i >= 0; i--) {
            samples[i] = 1.0 - (i / 100.0); //need decimal .0
            double aheadT = samples[i];
            double solveT = samples[i-1];
            Pose poseAtAheadT = activePath.getPath(0).getPose(aheadT);
            double aheadX = poseAtAheadT.getX();
            double aheadY = poseAtAheadT.getY();
            Pose poseAtSolveT = activePath.getPath(0).getPose(solveT);
            double solveX = poseAtSolveT.getX();
            double solveY = poseAtSolveT.getY();
            double distance = Math.hypot(aheadX - solveX, aheadY - solveY);



        }


        // Figure out how many samples to walk through
        // Based on parameter span from currentT to 1.0


        // Initialize at endpoint (t = 1.0)
        // Robot stops at endpoint, so nextSpeed = 0


        // Walk backward: from t = 1.0 toward currentT
        // At each sample, solve ellipse to get max speed here
        // Roll state so this sample becomes "next" for the following iteration


        return 0;  // return max speed at currentT once loop finishes
    }

    /**
     * Solves the friction ellipse quadratic for max feasible speed at this sample.
     *
     * Formula: A*W² + B*W + C = 0 where W = speed²
     * A = 4 * distance² * curvature² + 1
     * B = 4 * distance² * rotationRate² * robotSize² - 2 * nextSpeed²
     * C = nextSpeed⁴ - 4 * distance² * gripBudget²
     *
     * Returns speed = sqrt(W). Returns 0 if infeasible.
     */
    private double solveEllipse(double distance, double curvature, double rotationRate,
                                double nextSpeed, double gripBudget, double robotSize) {

        // Compute A, B, C coefficients


        // Compute discriminant


        // If discriminant negative, return 0 (infeasible)


        // Solve quadratic for W


        // Return sqrt(W), guarding against negative W


        return 0;
    }
}