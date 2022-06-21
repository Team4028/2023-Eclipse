// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.utilities.drive.BeakDrivetrain;

/** Get auton trajectories from paths. */
public class Trajectories {
    public static PathPlannerTrajectory TestPath1(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("Test Path 1",
                drivetrain.getPhysics().maxVelocity * 0.3,
                drivetrain.getPhysics().maxVelocity * 0.3);
    }
    // public static Trajectory getTrajectory(String path) {
    // Trajectory traj = new Trajectory();
    // try {
    // Path trajPath = Filesystem.getDeployDirectory().toPath().resolve("output/" +
    // path + ".wpilib.json");
    // traj = TrajectoryUtil.fromPathweaverJson(trajPath);
    // } catch (IOException e) {
    // System.out.println("Failed to load path " + path);
    // }

    // return traj;
    // }
}
