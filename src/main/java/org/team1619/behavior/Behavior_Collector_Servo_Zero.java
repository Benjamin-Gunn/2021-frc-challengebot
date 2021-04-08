package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.RobotTimer;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.Set;

/**
 * Zeros the servo that controls the release of the collector
 */

public class Behavior_Collector_Servo_Zero implements Behavior {

	private static final Logger sLogger = LogManager.getLogger(Behavior_Collector_Servo_Zero.class);
	private static final Set<String> sSubsystems = Set.of("ss_collector_servo");

	private final InputValues fSharedInputValues;
	private final OutputValues fSharedOutputValues;
	private RobotTimer mTimer;


	private double servoZeroOffset;

	public Behavior_Collector_Servo_Zero(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
		fSharedInputValues = inputValues;
		fSharedOutputValues = outputValues;

		servoZeroOffset = 0.0;
		mTimer = new RobotTimer(inputValues);
	}

	@Override
	public void initialize(String stateName, Config config) {
		sLogger.debug("Entering state {}", stateName);

		servoZeroOffset = config.getDouble("servo_zero_offset", 0.0);
		long timerTimeout = config.getInt("timeout_time",500);
		mTimer.start(timerTimeout);
	}

	@Override
	public void update() {
		if (!fSharedInputValues.getBoolean("ipb_collector_servo_has_been_zeroed")){
			fSharedOutputValues.setNumeric("opn_collector_servo", "", servoZeroOffset);
		}

		if(mTimer.isDone()) {
			fSharedInputValues.setBoolean("ipb_collector_servo_has_been_zeroed", true);
		}
	}

	@Override
	public void dispose() {
	}

	@Override
	public boolean isDone() {
		return fSharedInputValues.getBoolean("ipb_collector_servo_has_been_zeroed");
	}

	@Override
	public Set<String> getSubsystems() {
		return sSubsystems;
	}
}