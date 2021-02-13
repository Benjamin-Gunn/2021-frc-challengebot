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
	private int mode;

	private boolean isPriming;

	public TeleopModeLogic(InputValues inputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, robotConfiguration);


	}



	@Override
	public void initialize() {
		sLogger.info("***** TELEOP *****");
		mode = 0;
		isPriming = false;
	}

	@Override
	public void update() {
//		if (fSharedInputValues.getBooleanFallingEdge("ipb_driver_back")){
//			mode = (mode < 2)?(mode + 1): 0;
//		}
		if (fSharedInputValues.getBooleanRisingEdge("ipb_driver_right_bumper")) {
			isPriming = !isPriming;
		}

	}

	@Override
	public void dispose() {

	}



	@Override
	public boolean isReady(String name) {
		switch (name) {
			case "st_drivetrain_zero":
				return !fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed");
			case "st_drivetrain_swerve":
				return mode == 0;
			case "st_drivetrain_swerve_math":
				return mode == 1;
			case "st_drivetrain_swerve_align_test":
				return fSharedInputValues.getBoolean("ipb_driver_b");
			case "st_flywheel_zero":
				return !fSharedInputValues.getBoolean("ipb_flywheel_has_been_zeroed");
			case "pl_prime_to_shoot":
				return isPriming;
			case "pl_shoot":
				return fSharedInputValues.getBoolean("ipb_driver_right_trigger") && fSharedInputValues.getBoolean("ipb_flywheel_primed");
			default:
				return false;
		}
	}

	@Override
	public boolean isDone(String name, State state) {
		switch (name) {
			case "st_drivetrain_swerve_align_test":
				return !fSharedInputValues.getBoolean("ipb_driver_b");
			case "pl_prime_to_shoot":
				return !isPriming;
			case "pl_shoot":
				return fSharedInputValues.getBooleanFallingEdge("ipb_driver_right_trigger");
			default:
				return state.isDone();
		}

	}




}
