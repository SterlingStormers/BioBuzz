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

    @Override
    public DrivePowers calculatePath(Drivetrain drivetrain, PathTracker pathTracker, MotionState state, double deltaTime) {
        return DrivePowers.zero();  // placeholder
    }

    @Override
    public DrivePowers calculateHold(Drivetrain drivetrain, Pose target, MotionState state, boolean useScaling, double deltaTime) {
        return DrivePowers.zero();  // placeholder
    }

    @Override
    public double completion() {
        return 0;
    }

    @Override
    public Pose closestPose() {
        return null;
    }

    @Override
    public Vector2D closestTangent() {
        return null;
    }

    @Override
    public Vector2D closestNormal() {
        return null;
    }

    @Override
    public double curvature() {
        return 0;
    }

    @Override
    public double remainingDistance() {
        return 0;
    }

    @Override
    public boolean atParametricEnd() {
        return false;
    }

    @Override
    public void reset() {
        // placeholder
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public Map<String, Object> debug() {
        return new HashMap<>();
    }
}