package com.slipstream;


import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathSegment;
import com.pedropathing.utils.Angle;

public class VelocityProfile {

    private final SlipstreamConfig config;

    // Arrays for the backward pass results
    public double[] tValues;
    public double[] maxSpeeds;

    // Output values (what gets passed to ADRC)
    public double vx;
    public double vy;
    public double omega;

    public VelocityProfile(SlipstreamConfig config) {
        this.config = config;
    }

    public void compute(PathSegment segment, double currentT) {

        int samples = 101;
        tValues = new double[samples];
        maxSpeeds = new double[samples];

        // T values match indices: tValues[i] = i / 100.0
        for (int i = 0; i < samples; i++) {
            tValues[i] = i / 100.0;
        }

        // Endpoint: robot stops at t=1.0 (index 100)
        maxSpeeds[100] = 0;

        // Find robot's current index
        int currentIndex = (int) Math.round(currentT * 100);

        // Walk backward from 99 down to currentIndex
        for (int i = 99; i >= currentIndex; i--) {
            double hereT = tValues[i];
            double prevT = tValues[i + 1];       // previously computed T (one iteration ago)
            double prevSpeed = maxSpeeds[i + 1]; // previously computed speed (one iteration ago)
            // Distance
            Vector2D herePos = segment.curve.get(hereT);
            Vector2D prevPos = segment.curve.get(prevT);
            double distance = Math.hypot(prevPos.x() - herePos.x(), prevPos.y() - herePos.y());
            // Curvature
            double curvature = segment.curve.curvature(hereT);
            // Rotation Rate
            double hereHeading = segment.heading(hereT);
            double prevHeading = segment.heading(prevT);
            double headingChange = Math.abs(Angle.normalizeSigned(prevHeading - hereHeading));
            double rotationRate;
            if (distance == 0) {
                rotationRate = headingChange / 0.001;
            } else {
                rotationRate = headingChange / distance;
            }


            maxSpeeds[i] = solveEllipse(distance, curvature, rotationRate, prevSpeed, config.gripBudget.get(), config.robotSize.get());
        }

    }

    private double solveEllipse(double distance, double curvature, double rotationRate, double prevSpeed, double gripBudget, double robotSize) {
        double distanceSquare = distance * distance;
        double rotationRateQuad = rotationRate * rotationRate * rotationRate * rotationRate;
        double prevSpeedQuad = prevSpeed * prevSpeed * prevSpeed * prevSpeed;
        double A = (4 * distanceSquare * (curvature * curvature)) + 1 + (4 * distanceSquare * rotationRateQuad * (robotSize * robotSize));
        double B = -(2 * (prevSpeed * prevSpeed));
        double C = prevSpeedQuad - (4 * distanceSquare * (gripBudget * gripBudget));
        double discrim = (B * B) - (4 * A * C);
        if (discrim < 0) return 0;
        double W = (-B + Math.sqrt(discrim)) / (2 * A);
        return Math.sqrt(W);
    }
}