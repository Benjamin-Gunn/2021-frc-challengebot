package org.team1619.state;

import org.uacr.robot.AbstractRobotStatus;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.LimitedSizeQueue;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import org.uacr.utilities.purepursuit.Point;
import org.uacr.utilities.purepursuit.Vector;

import java.util.Map;
import java.util.Queue;

/**
 * Sets flags and does global math and logic for competition bot
 */

public class RobotStatus extends AbstractRobotStatus {

	private static final Logger sLogger = LogManager.getLogger(RobotStatus.class);

	public RobotStatus(InputValues inputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, robotConfiguration);
	}

	@Override
	public void initialize() {
		// Zero
		if (!fSharedInputValues.getBoolean("ipb_robot_has_been_zeroed")) {
			fSharedInputValues.setBoolean("ipb_drivetrain_has_been_zeroed", false);
		}
	}

	@Override
	public void update() {
		if (!fSharedInputValues.getBoolean("ipb_robot_has_been_zeroed") &&
				fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed")) {
			fSharedInputValues.setBoolean("ipb_robot_has_been_zeroed", true);
		}

		Map<String, Double> swerveOdometry = fSharedInputValues.getVector("ipv_swerve_odometry");
		Map<String, Double> limelightOdometry = fSharedInputValues.getVector("ipv_limelight_odometry");

		double limelightX = limelightOdometry.getOrDefault("x", 0.0);
		double limelightY = limelightOdometry.getOrDefault("y", 0.0);

		if(!Double.isFinite(limelightX)) {
			limelightX = 0.0;
		}

		if(!Double.isFinite(limelightY)) {
			limelightY = 0.0;
		}

		Vector positionDelta = new Vector(new Point(limelightOdometry.getOrDefault("x", 0.0), limelightOdometry.getOrDefault("y", 0.0)).subtract(new Point(swerveOdometry.getOrDefault("x", 0.0), swerveOdometry.getOrDefault("y", 0.0))));

		fSharedInputValues.setVector("ipv_odometry_delta", Map.of("distance", positionDelta.magnitude(), "x", positionDelta.getX(), "y", positionDelta.getY()));
	}

	@Override
	public void dispose() {

	}
}
