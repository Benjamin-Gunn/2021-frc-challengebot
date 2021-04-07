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

	private static final Logger LOGGER = LogManager.getLogger(RobotStatus.class);

	private String mLimelight;

	public RobotStatus(InputValues inputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, robotConfiguration);

		mLimelight = fRobotConfiguration.getString("global_drivetrain_swerve", "limelight");
	}

	@Override
	public void initialize() {
		// Zero
		if (!fSharedInputValues.getBoolean("ipb_robot_has_been_zeroed")) {
			fSharedInputValues.setBoolean("ipb_drivetrain_has_been_zeroed", false);
			fSharedInputValues.setBoolean("ipb_collector_servo_has_been_zeroed", false);
		}
	}

	@Override
	public void update() {
		if (!fSharedInputValues.getBoolean("ipb_robot_has_been_zeroed") &&
				fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed") &&
				fSharedInputValues.getBoolean("ipb_collector_servo_has_been_zeroed")) {
			fSharedInputValues.setBoolean("ipb_robot_has_been_zeroed", true);
		}
	}

	public void disabledUpdate() {
		Map<String, Double> llValues = fSharedInputValues.getVector(mLimelight);
		String mPathName;
		boolean hasTarget = llValues.getOrDefault("tv", 0.0) == 1;

		if (hasTarget) {
			double llTargetX = llValues.getOrDefault("tx", 0.0);
			if (llTargetX > 10) {
				mPathName = "sq_auto_gsc_b_red";
			} else if (llTargetX > 0) {
				mPathName = "sq_auto_gsc_a_red";
			}
			else if (llTargetX > -8) {
				mPathName = "sq_auto_gsc_a_blue";
			}
			else{
				mPathName = "sq_auto_gsc_b_blue";
			}
		} else {
			mPathName = "none";

		}

		fSharedInputValues.setString("gsc_path", llValues.getOrDefault("tv", 0.0) > 0.0 ? "st_drivetrain_gsc_a_red" : "st_drivetrain_gsc_b_red");
	}

	@Override
	public void dispose() {

	}
}
