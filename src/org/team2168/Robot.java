
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
public class Robot extends SampleRobot {

	private static final double BNO055_SAMPLERATE_DELAY_S = 0.100;

	private static BNO055 imu;
	private double[] pos = new double[3]; // [x,y,z] position data
	private BNO055.CalData cal;
	private DecimalFormat f = new DecimalFormat("0000.000");

	public Robot() {
		imu = BNO055.getInstance();

		System.out.println("Orientation Sensor Raw Data Test\n");

		/* Initialize the sensor */
		if(!imu.begin(BNO055.opmode_t.OPERATION_MODE_NDOF)) {
			/* There was a problem detecting the BNO055 ... check your connections */
			while(true){
				System.out.println("Ooops, no BNO055 detected ..."
						+ "\n   Check your wiring or I2C address and reboot roboRIO."
						+ "\n   See http://git.io/vEAh3 for wiring instructions.");
				Timer.delay(1);
			}
		}

		Timer.delay(1);

		/* Display the current temperature */
		System.out.println("Current Temperature: " + imu.getTemp() + " C");

		imu.setExtCrystalUse(true);

		System.out.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
	}


	public void disabled() {
		while (isDisabled()) {
			pos = imu.getVector(BNO055.vector_type_t.VECTOR_EULER);

			/* Display the floating point data */
			System.out.print("X: " + f.format(pos[0])
					+ " Y: " + f.format(pos[1]) + " Z: " + f.format(pos[2]));

			/* Display calibration status for each sensor. */
			cal = imu.getCalibration();
			System.out.println("\t\tCALIBRATION: Sys=" + cal.sys
					+ " Gyro=" + cal.gyro + " Accel=" + cal.accel
					+ " Mag=" + cal.mag);

			Timer.delay(BNO055_SAMPLERATE_DELAY_S);
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
