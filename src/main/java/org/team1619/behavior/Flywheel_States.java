package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.*;

/**
 * Controls the flywheel, using the state interrupt system
 */

public class Flywheel_States implements Behavior {

	private static final Logger sLogger = LogManager.getLogger(Flywheel_States.class);
	private static final Set<String> sSubsystems = Set.of("ss_flywheel");

	private final InputValues fSharedInputValues;
	private final OutputValues fSharedOutputValues;
	private final ArrayList<Double> fDataDistances;
	private final Map<Double, Integer> fSpeedProfile;
	private final double fIncrement;

	private boolean mAllowAdjust;
	private boolean mCoast;
	private boolean mHasReachedTurboVelocity;
	private double mInitialVelocity;
	private double mVelocity;
	private double mFinalVelocityError;
	private double mPercentOutput;
	private double mTurboVelocityCutoff;
	private double fVelocityAdjustment;
	private String mLimelight;
	private String adjustUpButton;
	private String adjustDownButton;
	private String mVelocityProfile;

	public Flywheel_States(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
		fSharedInputValues = inputValues;
		fSharedOutputValues = outputValues;
		adjustUpButton = robotConfiguration.getString("global_flywheel", "adjust_up_button");
		adjustDownButton = robotConfiguration.getString("global_flywheel", "adjust_down_button");
		fIncrement = robotConfiguration.getDouble("global_flywheel", "increment");
		fDataDistances = new ArrayList<>();
		fSpeedProfile = new HashMap<>();
		for (Object p : robotConfiguration.getMap("global_flywheel", "speed_profile").entrySet()) {
			Map.Entry<Double, Integer> point = (Map.Entry<Double, Integer>) p;
			fDataDistances.add(point.getKey());
			fSpeedProfile.put(point.getKey(), point.getValue());
		}

		Collections.sort(fDataDistances);

		mAllowAdjust = false;
		mCoast = false;
		mInitialVelocity = 0.0;
		mVelocity = 0.0;
		mFinalVelocityError = 0.0;
		mPercentOutput = 0.0;
		mTurboVelocityCutoff = 0.0;
		mVelocityProfile = "none";
		fVelocityAdjustment = 0.0;
	}

	@Override
	public void initialize(String stateName, Config config) {
		sLogger.debug("Entering state {}", stateName);

		mAllowAdjust = config.getBoolean("allow_adjust", false);
		mCoast = config.getBoolean("coast", false);
		mHasReachedTurboVelocity = false;


		mInitialVelocity = config.getDouble("velocity", 0.0);
		mVelocity = mInitialVelocity;
		mFinalVelocityError = config.getDouble("final_velocity_error", 500.0);
		mPercentOutput = config.getDouble("percent_output", 0.9);
		mTurboVelocityCutoff = config.getInt("turbo_velocity_cutoff", 0);
		mLimelight = config.getString("limelight", "none");
		mVelocityProfile = config.getString("velocity_profile", "none");
	}

	@Override
	public void update() {

		String outputType = "velocity";

		if (mTurboVelocityCutoff != 0 && mPercentOutput != 0.0 && fSharedInputValues.getNumeric("ipn_flywheel_primary_velocity") < mTurboVelocityCutoff && !mHasReachedTurboVelocity) {

			// If we haven't reached the turbo prime speed yet set the motor to the turbo prime output in percent mode
			outputType = "percent";
			mVelocity = mPercentOutput;
		} else if (!mLimelight.equals("none")) {

			// If the state should use the limelight use the distance to set the output
			mHasReachedTurboVelocity = true;

			double targetDistance = fSharedInputValues.getNumeric("ipn_odometry_distance");

			if (targetDistance > 0) {
				// ((top of target height - limelight height) / tan(limelight angle - (1/2 limelight FOV) + ty))

				if (fDataDistances.size() < 1) {
					// If there are no data points set velocity to 0
					mVelocity = 0;
				} else if (fDataDistances.size() == 1) {
					// If there is one data point set velocity to the value at the point
					mVelocity = fSpeedProfile.get(fDataDistances.get(0));
				} else if (targetDistance < fDataDistances.get(0)) {
					// If we are closer than the closest data point set the velocity to the value at the closest point
					mVelocity = fSpeedProfile.get(fDataDistances.get(0));
				} else if (targetDistance > fDataDistances.get(fDataDistances.size() - 1)) {
					// If we are farther than the farthest data point set the velocity to the value at the farthest point
					mVelocity = fSpeedProfile.get(fDataDistances.get(fDataDistances.size() - 1));
				} else {
					// We have at least one data point closer and one data point farther than our current position
					// interpolate speed between the points

					// Find the first distance value that is further than where we are
					int index = 1;
					for (; index < fDataDistances.size(); index++) {
						if (fDataDistances.get(index) > targetDistance) {
							break;
						}
					}

					// Get the slope between the previous distance and the next distance
					double slope = (fSpeedProfile.get(fDataDistances.get(index)) - fSpeedProfile.get(fDataDistances.get(index - 1))) / (fDataDistances.get(index) - fDataDistances.get(index - 1));

					// Calculate flywheel distance based on previous point and slope
					mVelocity = fSpeedProfile.get(fDataDistances.get(index - 1)) + (slope * (targetDistance - fDataDistances.get(index - 1)));
				}
			} else {
				// Set the velocity to the initial velocity if we don't have a limelight target
				mVelocity = mInitialVelocity;
			}
		}else if (mTurboVelocityCutoff != 0) {

			// If turbo prime is up to speed but we don't h
			mVelocity = mInitialVelocity;
			mHasReachedTurboVelocity = true;
		} else if (mCoast) {

			// If the state is coast set the output to 0 and the motor to coast so the flywheels spin down slowly
			mVelocity = 0.0;
			outputType = "percent";
		} else {

			// If we are just running percent set the velocity to the initial velocity
			mVelocity = mInitialVelocity;
		}

		if (mVelocityProfile.equals("none")) {

			// If there isn't a velocity profile set put the motor in percent output
			outputType = "percent";
		}


		if (fSharedInputValues.getBoolean(adjustUpButton)) {
			fVelocityAdjustment += fIncrement;
		}
		if (fSharedInputValues.getBoolean(adjustDownButton)) {
			fVelocityAdjustment -= fIncrement;
		}

		if(mAllowAdjust && mHasReachedTurboVelocity){
				mVelocity += fVelocityAdjustment;
		}

		if (mVelocity > 8000) {
			mVelocity = 8000;
		}

		if (mVelocity < -1000) {
			mVelocity = -1000;
		}


		fSharedInputValues.setBoolean("ipb_flywheel_primed", Math.abs(mVelocity - fSharedInputValues.getNumeric("ipn_flywheel_primary_velocity")) <= mFinalVelocityError && mVelocity != 0.0 && mHasReachedTurboVelocity);

		fSharedOutputValues.setNumeric("opn_flywheel", outputType, mVelocity, mVelocityProfile);
		fSharedInputValues.setNumeric("ipn_flywheel_velocity_adjustment", fVelocityAdjustment);
	}

	@Override
	public void dispose() {
		fSharedInputValues.setBoolean("ipb_flywheel_primed", false);
		sLogger.debug("Flywheel velocity = {}, Is Done {}", fSharedInputValues.getNumeric("ipn_flywheel_primary_velocity"), isDone());
	}

	@Override
	public boolean isDone() {
		if (mTurboVelocityCutoff != 0) {
			return Math.abs(mVelocity - fSharedInputValues.getNumeric("ipn_flywheel_primary_velocity")) <= mFinalVelocityError && mHasReachedTurboVelocity;
		} else {
			return Math.abs(mVelocity - fSharedInputValues.getNumeric("ipn_flywheel_primary_velocity")) <= mFinalVelocityError;
		}
	}

	@Override
	public Set<String> getSubsystems() {
		return sSubsystems;
	}
}