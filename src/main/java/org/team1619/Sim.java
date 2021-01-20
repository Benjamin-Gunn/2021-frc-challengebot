package org.team1619;

import org.team1619.modelfactory.SimModelFactory;
import org.team1619.robot.FrcSimRobot;
import org.team1619.state.StateControls;
import org.uacr.robot.AbstractModelFactory;
import org.uacr.robot.AbstractStateControls;

public class Sim {

	public static void main(String[] args) {
		new FrcSimRobot() {
			@Override
			protected AbstractStateControls createStateControls() {
				return new StateControls(fInputValues, fRobotConfiguration);
			}

			@Override
			protected AbstractModelFactory createModelFactory() {
				return new SimModelFactory(fHardwareFactory, fEventBus, fInputValues, fOutputValues, fRobotConfiguration, fObjectsDirectory);
			}
		}.start();
	}
}
