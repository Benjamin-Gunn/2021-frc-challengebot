package org.team1619.state;

import org.uacr.robot.AbstractRobotStatus;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.LimitedSizeQueue;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.Map;
import java.util.Queue;

/**
 * Sets flags and does global math and logic for competition bot
 */

public class RobotStatus extends AbstractRobotStatus {

	private static final Logger sLogger = LogManager.getLogger(RobotStatus.class);

	private String mLimelight;

	public RobotStatus(InputValues inputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, robotConfiguration);
		mLimelight = "";
	}

	@Override
	public void initialize() {
		// Zero
		if (!fSharedInputValues.getBoolean("ipb_robot_has_been_zeroed")) {
			fSharedInputValues.setBoolean("ipb_drivetrain_has_been_zeroed", false);
		}

		mLimelight = fRobotConfiguration.getString("global_limelight", "limelight");
	}

	@Override
	public void update() {
		if (!fSharedInputValues.getBoolean("ipb_robot_has_been_zeroed") &&
				fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed")) {
			fSharedInputValues.setBoolean("ipb_robot_has_been_zeroed", true);
		}

		// jace and alex path no signaling test


		Map<String, Double> llValues = fSharedInputValues.getVector(mLimelight);
		String mPathName;
		boolean hasTarget = llValues.getOrDefault("tv", 0.0) == 1;
		mPathName = "No Target";
		if (hasTarget) {
			double llTargetX = llValues.getOrDefault("tx", 0.0);
			double llTargetY = llValues.getOrDefault("ty", 0.0);
			mPathName = "none";
			if (llTargetX < 10 && llTargetX > 0) {
				mPathName = "sq_auto_gsc_a_red";
			} else if (llTargetX > 20 && llTargetX < 30) {
				mPathName = "sq_auto_gsc_a_blue";
			} else if (llTargetX < -10 && llTargetX > -30) {
				mPathName = "sq_auto_gsc_b_red";
			} else if (llTargetX < 20 && llTargetX > 10) {
				mPathName = "sq_auto_gsc_b_blue";
			}
		}
		fSharedInputValues.setString("Path", mPathName);

		// calculating distance to target for use by flywheels

		boolean hasTarget = llValues.getOrDefault("tv", 0.0) == 1;
		double limelightTargetX = llValues.getOrDefault("tx", 0.0);
		double limelightTargetY = llValues.getOrDefault("ty", 0.0);
		fSharedInputValues.setNumeric("ipn_target_distance", hasTarget ? Math.hypot(limelightTargetY, limelightTargetX) : -1);

	}

	@Override
	public void dispose() {

	}
}
