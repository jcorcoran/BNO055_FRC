
package org.team2168;

import java.text.DecimalFormat;

import org.team2168.utils.BNO055;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
public class Robot_SampleRobot extends SampleRobot {

	private static BNO055 imu;
	private double[] pos = new double[3]; // [x,y,z] position data
	private BNO055.CalData cal;
	private DecimalFormat f = new DecimalFormat("+000.000;-000.000");

	public Robot_SampleRobot() {
		imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
				BNO055.vector_type_t.VECTOR_EULER);
	}


	public void disabled() {
		while (isDisabled()) {
			System.out.println("COMMS: " + imu.isSensorPresent()
					+ ", INITIALIZED: " + imu.isInitialized()
					+ ", CALIBRATED: " + imu.isCalibrated());
			if(imu.isInitialized()){
				pos = imu.getVector();
	
				/* Display the floating point data */
				System.out.println("\tX: " + f.format(pos[0])
						+ " Y: " + f.format(pos[1]) + " Z: " + f.format(pos[2])
						+ "  H: " + imu.getHeading());
	
				/* Display calibration status for each sensor. */
				cal = imu.getCalibration();
				System.out.println("\tCALIBRATION: Sys=" + cal.sys
						+ " Gyro=" + cal.gyro + " Accel=" + cal.accel
						+ " Mag=" + cal.mag);
			}

			Timer.delay(0.2); // seconds
		}
	}

	public void autonomous() {
	}

	/**
	 * Runs the motors with arcade steering.
	 */
	public void operatorControl() {
		while (isOperatorControl() && isEnabled()) {
			Timer.delay(0.005);		// wait for a motor update time
		}
	}

	/**
	 * Runs during test mode
	 */
	public void test() {
	}
}
