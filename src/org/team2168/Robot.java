
package org.team2168;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import java.text.DecimalFormat;

import org.team2168.commands.ExampleCommand;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.utils.BNO055;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;
	private DecimalFormat f = new DecimalFormat("+000.000;-000.000");
	private double[] pos = new double[3]; // [x,y,z] position data
	private BNO055.CalData cal;
	
	public static Drivetrain drivetrain;
	
    Command autonomousCommand;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
		oi = new OI();
		
		drivetrain = Drivetrain.getInstance();
		
        // instantiate the command used for the autonomous period
        autonomousCommand = new ExampleCommand();
    }
	
	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
    public void disabledInit(){

    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		
		/*
		 * To see data printed, wire the sensor in to the I2C port on your
		 *   robot per the instructions in the github repository. Connect a
		 *   driver station to your robot.
		 * Above the text box on the right hand side of the Driver Station,
		 *   click the white gear, then select "+ Prints" to enable
		 *   displaying the below print statements.
		 */
		System.out.println("COMMS: " + drivetrain.isSensorPresent()
				+ ", INITIALIZED: " + drivetrain.isInitialized()
				+ ", CALIBRATED: " + drivetrain.isCalibrated());
		if(drivetrain.isInitialized()){
			pos = drivetrain.getVector();
		
			/* Display the floating point data */
			System.out.println("\tX: " + f.format(pos[0])
					+ " Y: " + f.format(pos[1]) + " Z: " + f.format(pos[2])
					+ "  H: " + drivetrain.getHeading());
		
			/* Display calibration status for each sensor. */
			cal = drivetrain.getCalibration();
			System.out.println("\tCALIBRATION: Sys=" + cal.sys
					+ " Gyro=" + cal.gyro + " Accel=" + cal.accel
					+ " Mag=" + cal.mag);
		}
	}

    public void autonomousInit() {
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}
