package com.slipstream;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathTracker;

import java.util.HashMap;
import java.util.Map;

public class RewindAlgorithm implements Algorithm {

    private final SlipstreamConfig config;
    private final VelocityProfile velocityProfile;
    // TODO: Add ADRC instances (one per axis) when you build ADRC class
    // TODO: Add Kinematics instance when you build Kinematics class

    // State tracked between loops
    private double currentT = 0;
    private boolean busy = false;

    // State updated each loop (for getters)
    private Pose closestPose;
    private Vector2D closestTangent;
    private Vector2D closestNormal;
    private double curvature;
    private double remainingDistance;

    // Debug info
    private Map<String, Object> debug = new HashMap<>();

    public RewindAlgorithm(SlipstreamConfig config) {
        this.config = config;
        this.velocityProfile = new VelocityProfile(config);
        // TODO: Initialize ADRC instances
        // TODO: Initialize Kinematics
    }

    @Override
    public DrivePowers calculatePath(Drivetrain drivetrain, PathTracker pathTracker,
                                     MotionState state, double deltaTime) {

        // 1. Update currentT (where robot is on path)
        currentT = pathTracker.current().curve.closestParameter(
                state.pose().toVector2D(),
                currentT
        );

        // 2. Update state for getter methods
        closestPose = pathTracker.current().curve.get(currentT)
                .toPose(pathTracker.current().heading(currentT));
        closestTangent = pathTracker.current().curve.tangent(currentT);
        closestNormal = pathTracker.current().curve.leftNormal(currentT);
        curvature = pathTracker.current().curve.curvature(currentT);
        remainingDistance = pathTracker.current().curve.remainingDistance(currentT);

        // 3. VelocityProfile computes vx, vy, omega
        velocityProfile.compute(pathTracker.current(), currentT);

        // 4. ADRC processes velocity commands into motor efforts
        // TODO: Replace with actual ADRC calls
        double effortVx = velocityProfile.vx;
        double effortVy = velocityProfile.vy;
        double effortOmega = velocityProfile.omega;

        // 5. Convert efforts to per-wheel motor powers via kinematics
        // TODO: Replace with actual Kinematics class call
        DrivePowers drivePowers = DrivePowers.zero();

        // 6. Update debug info
        updateDebugInfo();

        return drivePowers;
    }

    @Override
    public DrivePowers calculateHold(Drivetrain drivetrain, Pose target, MotionState state,
                                     boolean useScaling, double deltaTime) {
        // TODO: Implement hold-at-target logic
        return DrivePowers.zero();
    }

    @Override
    public double completion() {
        return currentT;
    }

    @Override
    public Pose closestPose() {
        return closestPose;
    }

    @Override
    public Vector2D closestTangent() {
        return closestTangent;
    }

    @Override
    public Vector2D closestNormal() {
        return closestNormal;
    }

    @Override
    public double curvature() {
        return curvature;
    }

    @Override
    public double remainingDistance() {
        return remainingDistance;
    }

    @Override
    public boolean atParametricEnd() {
        return currentT >= 0.95;  // threshold, could be config value
    }

    @Override
    public void reset() {
        currentT = 0;
        busy = true;
        // TODO: Reset ADRC instances
        debug.clear();
    }

    @Override
    public boolean isBusy() {
        return busy;
    }

    @Override
    public Map<String, Object> debug() {
        return debug;
    }

    private void updateDebugInfo() {
        debug.put("currentT", currentT);
        debug.put("closestPose", closestPose);
        debug.put("curvature", curvature);
        debug.put("remainingDistance", remainingDistance);
        debug.put("targetVx", velocityProfile.vx);
        debug.put("targetVy", velocityProfile.vy);
        debug.put("targetOmega", velocityProfile.omega);
        // Add more as useful
    }
}