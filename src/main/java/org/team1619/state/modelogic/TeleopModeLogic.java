package org.team1619.state.modelogic;

import org.uacr.models.state.State;
import org.uacr.robot.AbstractModeLogic;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

/**
 * Handles the isReady and isDone logic for teleop mode on competition bot
 */

public class TeleopModeLogic extends AbstractModeLogic {

	private static final Logger sLogger = LogManager.getLogger(TeleopModeLogic.class);

	public TeleopModeLogic(InputValues inputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, robotConfiguration);
	}

	@Override
	public void initialize() {
		sLogger.info("***** TELEOP *****");
	}

	@Override
	public void update() {
	}

	@Override
	public void dispose() {
		// Prevents the auto from running twice in a row without first switching to teleop and then switching back to auto
		fSharedInputValues.setBoolean("ipb_auto_complete", false);
	}

	@Override
	public boolean isReady(String name) {
		switch (name) {
			case "st_drivetrain_zero":
				return !fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed");
			case "st_drivetrain_swerve_align":
				return fSharedInputValues.getBoolean("ipb_driver_left_bumper");
			case "st_drivetrain_oval":
				return fSharedInputValues.getBoolean("ipb_driver_y");
			case "st_drivetrain_straightline":
				return fSharedInputValues.getBoolean("ipb_driver_x");
			case "st_collector_servo_zero":
				return !fSharedInputValues.getBoolean("ipb_collector_servo_has_been_zeroed");
			default:
				return false;
		}
	}

	@Override
	public boolean isDone(String name, State state) {
		switch (name) {
			case "st_drivetrain_swerve_align":
				return !fSharedInputValues.getBoolean("ipb_driver_left_bumper");
			default:
				return state.isDone();
		}
	}
}