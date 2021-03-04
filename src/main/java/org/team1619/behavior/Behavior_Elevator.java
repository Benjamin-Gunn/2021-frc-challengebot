package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.Timer;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.Set;

/**
 * Runs elevator at specified speed
 */

public class Behavior_Elevator implements Behavior {

	private static final Logger sLogger = LogManager.getLogger(Behavior_Elevator.class);
	private static final Set<String> sSubsystems = Set.of("ss_elevator");

	private final InputValues fSharedInputValues;
	private final OutputValues fSharedOutputValues;

	private double speed;
	private long cycleOnTime;
	private long cycleOffTime;
	private boolean isCyclingEnabled;
	private final Timer cycleTimer;
	private Boolean isCycleOn;
	private double currentSpeed;


	public Behavior_Elevator(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
		fSharedInputValues = inputValues;
		fSharedOutputValues = outputValues;

		cycleOnTime =  0;
		cycleOffTime = 0;
		speed = 0.0;
		cycleTimer = new Timer();
		isCycleOn = true;
	}

	@Override
	public void initialize(String stateName, Config config) {
		sLogger.debug("Entering state {}", stateName);

		cycleOnTime = config.getInt("cycle_time_on", 0);
		cycleOffTime = config.getInt("cycle_time_off", 0);
		speed = config.getDouble("speed", 0.0);
		isCyclingEnabled = config.getBoolean("enable_cycling", false);
		cycleTimer.start(cycleOnTime);
		currentSpeed = speed;

	}

	@Override
	public void update() {

		if (isCyclingEnabled) {
			if (cycleTimer.isDone()) {
				isCycleOn = !isCycleOn;
				if (isCycleOn) {
					currentSpeed = speed;
				cycleTimer.start(cycleOnTime);
				} else {
					currentSpeed = 0.0;
					cycleTimer.start(cycleOffTime);
				}
			}
		}

		fSharedOutputValues.setNumeric("opn_elevator","percent", currentSpeed );
	}


	@Override
	public void dispose() {
		fSharedOutputValues.setNumeric("opn_elevator", "percent", 0);
	}

	@Override
	public boolean isDone() {
		return true;
	}

	@Override
	public Set<String> getSubsystems() {
		return sSubsystems;
	}
}