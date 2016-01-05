package org.team2168.subsystems;

import org.team2168.RobotMap;
import org.team2168.utils.BNO055;
import org.team2168.utils.BNO055.CalData;
import org.team2168.utils.BNO055.reg_t;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Drivetrain extends Subsystem {
    private static Drivetrain instance = null; 
	private static Talon left, right;
	private static BNO055 imu;
    
    /**
     * Private constructor. Singleton pattern, so it can only be
     *   initialized once!
     */
    private Drivetrain() {
    	left = new Talon(RobotMap.leftMotor);
    	right = new Talon(RobotMap.rightMotor);
    	
    	imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
				BNO055.vector_type_t.VECTOR_EULER);
    }
    
	/**
	 * @return an instance of the subsystem.
	 */
	public static Drivetrain getInstance() {
		if(instance == null) {
			instance = new Drivetrain();
		}

		return instance;
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    /**
     * Set the speed of the drivetrain motor controllers.
     * 
     * @param leftSpeed speed to set the left side of the drivetrain (1.0 to -1.0)
     * @param rightSpeed speed to set the right side of the drivetrain (1.0 to -1.0)
     */
    public void set(double leftSpeed, double rightSpeed) {
    	left.set(leftSpeed);
    	right.set(rightSpeed);
    }
    
    /**
	 * The heading of the sensor (x axis) in continuous format. Eg rotating the
	 *   sensor clockwise two full rotations will return a value of 720 degrees.
	 *
	 * @return heading in degrees
     */
    public double getHeading() {
    	return imu.getHeading();
    }
    
    /**
     * Gets a vector representing the sensors position (heading, roll, pitch).
	 * heading:    0 to 360 degrees
	 * roll:     -90 to +90 degrees
	 * pitch:   -180 to +180 degrees
	 *
	 * For continuous rotation heading (doesn't roll over between 360/0) see
	 *   the getHeading() method.
	 *
	 * @return a vector [heading, roll, pitch]
	 */
    public double[] getVector() {
    	return imu.getVector();
    }
    
	/**
	 * @return true if the IMU is found on the I2C bus
	 */
	public boolean isSensorPresent() {
		return imu.isSensorPresent();
	}

	/** 
	 * @return true when the IMU is initialized.
	 */
	public boolean isInitialized() {
		return imu.isInitialized();
	}
	
	/**
	 * Gets current IMU calibration state.
	 * @return each value will be set to 0 if not calibrated, 3 if fully
	 *   calibrated.
	 */
	public BNO055.CalData getCalibration() {
		return imu.getCalibration();
	}
	
	/**
	 * Returns true if all required sensors (accelerometer, magnetometer,
	 *   gyroscope) in the IMU have completed their respective calibration
	 *   sequence.
	 * @return true if calibration is complete for all sensors required for the
	 *   mode the sensor is currently operating in. 
	 */
	public boolean isCalibrated() {
		return imu.isCalibrated();
	}
}

