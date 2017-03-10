package org.usfirst.frc.team97.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;

import java.sql.Time;
import java.util.ArrayList;
import java.util.Date;
import java.util.concurrent.TimeUnit;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Thread visionThread;
	Thread tread;

	// Outputs (motors)
	Victor[] driveMotors;
	Talon shooter;
	Talon climber;
	Talon indexer;
	Victor gear;
	Servo gimbalX;
	Servo gimbalY;

	// Inputs
	Joystick driveStick;
	AnalogGyro gyro;
	DigitalInput indexLim;
	DigitalInput gearLim;

	double thresh; // Drive threshold to minimize insignificant movements

	// Variables for the gimbal
	double trimX;
	double trimY;
	double gimSpd;
	double gimRateX;
	double gimRateY;
	long gimAngX;
	long gimAngY;
	Date dateF;
	Date dateI;
	long deltaDate;
	private int tarX;
	double timeAtStart;
	int autoMode;
	double timeElapsed;

	@Override
	public void robotInit() {
		driveStick = new Joystick(0);
		gyro = new AnalogGyro(0);
		gyro.initGyro();
		gyro.calibrate();
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Gyro Rate", gyro.getRate());

		// Initialize drive motors
		driveMotors = new Victor[4]; // Array of the 4 drive motors initialized
		for (int i = 0; i < 4; i++) {
			driveMotors[i] = new Victor(i); // Set to PWM ports 1-4
		}
		myRobot = new RobotDrive(driveMotors[3], driveMotors[1], driveMotors[2], driveMotors[0]);

		// Initialization of other actuators
		climber = new Talon(4);
		shooter = new Talon(5);
		indexer = new Talon(6);
		gear = new Victor(7);

		indexLim = new DigitalInput(0); // Index limit switch set to IO port 0
		gearLim = new DigitalInput(1); // Gear limit switch set to IO port 1

//		tread = new Thread(() -> {
//			 while (true) {
//			// for (int i = 0; i < 10; i++) {
//			// GripPipelineAndOutput.hslThresholdLuminance[0] = 44-2*i;
//			// GripPipelineAndOutput.hslThresholdLuminance[1] = 255;
//			// SmartDashboard.putNumber("Luminance", i);
//			// try {
//			// TimeUnit.MILLISECONDS.sleep(250);
//			// } catch (InterruptedException e) {
//			// // TODO Auto-generated catch block
//			// e.printStackTrace();
//			// }
//			// }
//			// GripPipelineAndOutput.hslThresholdLuminance[0] = 44;
//			// GripPipelineAndOutput.hslThresholdLuminance[1] = 255.0;
//			//
//			// for (int i = 0; i < 10; i++) {
//			// GripPipelineAndOutput.hslThresholdHue[0] = 78 - 2 * i;
//			// GripPipelineAndOutput.hslThresholdHue[1] = 92 + 2 * i;
//			// SmartDashboard.putNumber("Hue", i);
//			// try {
//			// TimeUnit.MILLISECONDS.sleep(250);
//			// } catch (InterruptedException e) {
//			// // TODO Auto-generated catch block
//			// e.printStackTrace();
//			// }
//			// }
//			// GripPipelineAndOutput.hslThresholdHue[0] = 78;
//			// GripPipelineAndOutput.hslThresholdHue[1] = 92;
//			//
//			// for (int i = 0; i < 10; i++) {
//			// GripPipelineAndOutput.hslThresholdSaturation[0] = 172 - 2 * i;
//			// GripPipelineAndOutput.hslThresholdSaturation[1] = 255;
//			// SmartDashboard.putNumber("Saturation", i);
//			// try {
//			// TimeUnit.MILLISECONDS.sleep(250);
//			// } catch (InterruptedException e) {
//			// // TODO Auto-generated catch block
//			// e.printStackTrace();
//			// }
//			// }
//			// GripPipelineAndOutput.hslThresholdSaturation[0] = 172;
//			// GripPipelineAndOutput.hslThresholdSaturation[1] = 255;
//			//
//			// }
//			while (true) {
//				GripPipelineAndOutput.hslThresholdHue[0] = SmartDashboard.getNumber("HueI", 78);
//				GripPipelineAndOutput.hslThresholdHue[1] = SmartDashboard.getNumber("HueF", 92);
//				GripPipelineAndOutput.hslThresholdHue[0] = SmartDashboard.getNumber("Saturation", 172);
//				GripPipelineAndOutput.hslThresholdHue[0] = SmartDashboard.getNumber("Luminance", 172);
////				GripPipelineAndOutput.hslThresholdHue[0] = Integer.parseInt(SmartDashboard.getData("HueI").toString());
////				GripPipelineAndOutput.hslThresholdHue[1] = Integer.parseInt(SmartDashboard.getData("HueF").toString());
////				GripPipelineAndOutput.hslThresholdSaturation[0] = Integer
////						.parseInt(SmartDashboard.getData("Saturation").toString());
////				GripPipelineAndOutput.hslThresholdLuminance[0] = Integer
////						.parseInt(SmartDashboard.getData("Luminance").toString());
//			}
//		});
//		tread.setDaemon(true);
//		tread.start();

		SmartDashboard.putNumber("HueI", GripPipelineAndOutput.hslThresholdHue[0]);
		SmartDashboard.putNumber("HueF", GripPipelineAndOutput.hslThresholdHue[1]);
		SmartDashboard.putNumber("SaturationI", GripPipelineAndOutput.hslThresholdSaturation[0]);
		SmartDashboard.putNumber("SaturationF", GripPipelineAndOutput.hslThresholdSaturation[1]);
		SmartDashboard.putNumber("LuminanceI", GripPipelineAndOutput.hslThresholdLuminance[0]);
		SmartDashboard.putNumber("LuminanceF", GripPipelineAndOutput.hslThresholdLuminance[1]);
		SmartDashboard.putNumber("Blur", GripPipelineAndOutput.kBlur);
		SmartDashboard.putNumber("Thresh", GripPipelineAndOutput.kBlur);
		SmartDashboard.putNumber("ThreshRatio", GripPipelineAndOutput.kBlur);
		visionThread = new Thread(() -> {

			// Start the camera stream...
			// Get the UsbCamera from CameraServer
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// Set the resolution
			camera.setVideoMode(VideoMode.getPixelFormatFromInt(2), 1280, 720, 15);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("hsl Output", 1280, 720);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {

				GripPipelineAndOutput.hslThresholdHue[0] = SmartDashboard.getNumber("HueI", 78);
				GripPipelineAndOutput.hslThresholdHue[1] = SmartDashboard.getNumber("HueF", 92);
				GripPipelineAndOutput.hslThresholdSaturation[0] = SmartDashboard.getNumber("SaturationI", 172);
				GripPipelineAndOutput.hslThresholdSaturation[1] = SmartDashboard.getNumber("SaturationF", 255);
				GripPipelineAndOutput.hslThresholdLuminance[0] = SmartDashboard.getNumber("LuminanceI", 44);
				GripPipelineAndOutput.hslThresholdLuminance[1] = SmartDashboard.getNumber("LuminanceF", 255);
				GripPipelineAndOutput.kBlur = SmartDashboard.getNumber("Blur", 25);
				GripPipelineAndOutput.thresh = SmartDashboard.getNumber("Thresh", 10);
				GripPipelineAndOutput.threshRatio = SmartDashboard.getNumber("ThreshRatio", 3);
				
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat. If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());

					// skip the rest of the current iteration
					continue;
				}

				GripPipelineAndOutput.process(mat);

				mat = GripPipelineAndOutput.hslThresholdOutput();

				SmartDashboard.putString("Pipeline Access exception: ", "");
				// Error occurs sometimes here... could be too taxing on
				// processing power and could crash the thread.
				try {
					ArrayList<MatOfPoint> contours = GripPipelineAndOutput.filterContoursOutput();
					for (MatOfPoint i : contours) {
						MatOfPoint cont = i;

						// tarX = (int) cont.toArray()[0].x;
						Rect tar = Imgproc.boundingRect(cont);

						// Imgproc.rectangle(mat, new Point(tar.x, tar.y), new
						// Point(tar.x+tar.width, tar.y+tar.height), (255, 0, 0,
						// 255), 3);
						Imgproc.rectangle(mat, new Point(tar.x, tar.y),
								new Point(tar.x + tar.width, tar.y + tar.height), new Scalar(255, 0, 0), 3);
						// Core.line(mat, new Point(50,50), new Point(10, 100),
						// new
						// Scalar(255,0,0), 1);
					}
					// SmartDashboard.putNumber("Rect Height", tar.height);
					// SmartDashboard.putNumber("Rect Width", tar.width);
					// SmartDashboard.putNumber("Rect X", tar.x);
					// SmartDashboard.putNumber("Rect Y", tar.y);
					SmartDashboard.putNumber("Filtered Contours X", tarX);
				} catch (Exception e) {
					SmartDashboard.putString("Pipeline Access exception: ", e.getMessage());
					SmartDashboard.putNumber("Rect Height", 0);
					SmartDashboard.putNumber("Rect Width", 0);
					SmartDashboard.putNumber("Rect X", 0);
					SmartDashboard.putNumber("Rect Y", 0);
					SmartDashboard.putNumber("Filtered Contours X", 0);

				}

				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
//		visionThread.setDaemon(true);
//		visionThread.start();

		thresh = 0.1; // Threshold that drive axes X and Y must overcome

		// Initialize both axes Servo's for the gimbal and set to .5, stopped
		gimbalX = new Servo(8);
		gimbalY = new Servo(9);
		gimbalX.set(.5);
		gimbalY.set(.5);

		// Initialize variables related to gimbal (rates, angles, trim, and
		// adjustment speed)
		gimSpd = .15;
		gimRateX = .5;
		gimRateY = .5;
		gimAngX = 0;
		gimAngY = 0;
		trimX = 0;
		trimY = .03;

		// Initialize final and initial dates for finding camera angle from
		// rates
		dateF = new Date();
		dateI = new Date();

		SmartDashboard.putNumber("AutoMode", 0);
	}

	@Override
	public void autonomousInit() {
		timeAtStart = System.currentTimeMillis();
		autoMode = (int) SmartDashboard.getNumber("AutoMode", 0);
	}

	@Override
	public void autonomousPeriodic() {
		autoMode = (int) SmartDashboard.getNumber("AutoMode", 0);
		timeElapsed = (((double) (System.currentTimeMillis() - timeAtStart)) / 1000);
		if(autoMode == 0) //if (timeElapsed <= 3){myRobot.mecanumDrive_Cartesian(0, 0.5, 0, 0);} //gear handling
				SmartDashboard.putString("Test", "case 0: " + (timeElapsed <= 3));
		else if(autoMode == 1) //if (timeElapsed <= 10){myRobot.mecanumDrive_Cartesian(0, 0.8, 0, 0);} //pass the line
				SmartDashboard.putString("Test", "case 1: " + (timeElapsed <= 10));
		else//myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
				SmartDashboard.putString("Test", "default: " + autoMode);
	}

	@Override
	public void teleopInit() {
		// Disable all safety's for motors
		for (Victor j : driveMotors) {
			j.setExpiration(3);
		}
		shooter.setSafetyEnabled(false);
		climber.setSafetyEnabled(false);
		indexer.setSafetyEnabled(false);
		gear.setSafetyEnabled(false);

		gimbalX.set(.5);
		gimbalY.set(.5);
	}

	@Override
	public void teleopPeriodic() {
		// Send joystick axes to dashboard
		SmartDashboard.putNumber("Joystick X", driveStick.getX());
		SmartDashboard.putNumber("Joystick Y", -driveStick.getY());

		// Check each periodic system
//		 checkDrive();
//		 checkSquareUp();
//		 checkGear();
//		 checkClimb();
//		 checkGimbal();
//		 checkShoot();
	}

	/**
	 * Gets drive input from joystick for X, Y, and Twist, and sends input to
	 * mecDrive. Uses a threshold that it won't drive below.
	 */
	public void checkDrive() {
		if (driveStick.getMagnitude() > thresh) {
			mecDrive(driveStick.getX(), driveStick.getY(), driveStick.getTwist() / 2, gyro.getAngle());
		} else {
			myRobot.stopMotor();
		}
	}

	/**
	 * If button 3 is pressed, square the robot to the gyro angle.
	 */
	public void checkSquareUp() {
		if (driveStick.getRawButton(3)) {
			squareUp(gyro.getAngle());
		}
	}

	/**
	 * If button 8 is pressed, climb up. If button 7 is pressed, climb down.
	 * Otherwise, don't climb.
	 */
	public void checkClimb() {
		SmartDashboard.putString("climber", "not climbing");
		if (driveStick.getRawButton(8)) {
			climber.set(-1);
			SmartDashboard.putString("climber", "climbing up");
		} else if (driveStick.getRawButton(7)) {
			climber.set(1);
			SmartDashboard.putString("climber", "climbing down");
		} else {
			climber.set(0);
		}
	}

	/**
	 * If button 9 is pressed, bring gear plate up. If button 10 is pressed,
	 * bring gear plate down. Otherwise, don't move the gear plate.
	 */
	public void checkGear() {
		SmartDashboard.putBoolean("gear limit", gearLim.get());
		if (driveStick.getRawButton(9)) {
			SmartDashboard.putString("gear", "up");
			gear.set(.75);
		} else if (driveStick.getRawButton(10) && gearLim.get()) {
			SmartDashboard.putString("gear", "down");
			gear.set(-.75);
		} else {
			SmartDashboard.putString("gear", "stopped");
			gear.set(0);
		}
	}

	/**
	 * System for both shooter and indexer (uses index limit) If button 6 is
	 * pressed, start shooter. If button 5 is pressed, stop shooter. If the
	 * limit is not pressed or if the trigger is pulled, run the indexer. If
	 * button 2 is pressed, reverse the indexer. Otherwise, stop the indexer.
	 */
	public void checkShoot() {
		if (driveStick.getRawButton(5)) {
			shooter.set(0);
			SmartDashboard.putString("shooter", "stopped");
		} else if (driveStick.getRawButton(6)) {
			shooter.set(-1);
			SmartDashboard.putString("shooter", "shooting");
		}

		SmartDashboard.putBoolean("index limit", indexLim.get());

		// If the limit is not pressed or if the trigger is pulled, run the
		// indexer, if button 2 is pressed, reverse the indexer
		if (driveStick.getRawButton(2)) {
			indexer.set(.3);
			SmartDashboard.putString("indexer", "backward");
		} else if (/*!indexLim.get() || */driveStick.getRawButton(1)) {
			indexer.set(-.3);
			SmartDashboard.putString("indexer", "running");
		} else { // Don't run the indexer
			indexer.set(0);
			SmartDashboard.putString("indexer", "stopped");
		}
	}

	/**
	 * Uses the POV input on the joystick to rotate the gimbal for the camera.
	 * Also sets the gimbal angle based on time between cycles.
	 */
	public void checkGimbal() {
		int povInput = driveStick.getPOV();
		SmartDashboard.putNumber("povInput", povInput);

		// Get input from the joystick to adjust the gimbal mount
		// If there is no input, set the rates to zero
		if (povInput > 180) // left
			gimRateX = .5 + trimX - gimSpd;
		else if (povInput < 180 && povInput > 0) // right
			gimRateX = .5 + gimSpd;
		else // neither
			gimRateX = .5;

		if (povInput > 90 && povInput < 270) // down
			gimRateY = .5 + trimY - gimSpd;
		else if (povInput > 270 || (povInput < 90 && povInput > -1)) // up
			gimRateY = .5 + gimSpd;
		else
			gimRateY = .5; // neither

		// Finds the change in time between cycles.
		dateF = new Date();
		deltaDate = dateF.getTime() - dateI.getTime();
		dateI = new Date();
		SmartDashboard.putNumber("deltaDate", deltaDate);

		// Sets the gimbal angle based on deltaDate, the change in time between
		// cycles.
		gimAngX += (deltaDate * (gimRateX - .5));
		gimAngY += (deltaDate * (gimRateY - .5) * 1.511);

		gimbalX.set(1 - gimRateX);
		gimbalY.set(1 - gimRateY);

		SmartDashboard.putNumber("gimAngX", gimAngX);
		SmartDashboard.putNumber("gimAngY", gimAngY);
	}

	/**
	 * Takes X, Y, and Twist velocities and uses mecanumDrive_Cartesian to drive
	 * a mecanum robot left, right, foreward, or backward at a given velocity,
	 * but it won't do anything in between them.
	 * 
	 * @param stickX
	 *            Desired - X velocity
	 * @param stickY
	 *            Desired - Y velocity
	 * @param rotatThatPotat
	 *            - Desired twisting velocity
	 * @param gyroAngle
	 *            - Does nothing for now
	 */
	public void mecDrive(double stickX, double stickY, double twistMagnitude, double gyroAngle) {
		double xVector = 0;
		double yVector = 0;
		if (Math.abs(stickX) > Math.abs(stickY)) {
			xVector = driveStick.getX();
			yVector = 0;
			if (xVector < 0) {
				SmartDashboard.putString("Direction", "Right");
			} else if (xVector > 0) {
				SmartDashboard.putString("Direction", "Left");
			}
		} else if (Math.abs(stickY) > Math.abs(stickX)) {
			xVector = 0;
			yVector = driveStick.getY();
			if (xVector < 0) {
				SmartDashboard.putString("Direction", "Forwards");
			} else if (xVector > 0) {
				SmartDashboard.putString("Direction", "Backwards");
			}
		}

		myRobot.mecanumDrive_Cartesian(-xVector / 2, yVector / 2, twistMagnitude, 0);
	}

	/**
	 * Turns the robot toward the gyroAngle given
	 * 
	 * @param gyroAngle
	 *            - Desired angle to point to, can be any double.
	 */
	public void squareUp(double gyroAngle) {
		if (gyro.getAngle() > gyro.getCenter() + 180) {
			myRobot.mecanumDrive_Cartesian(0, 0, 0.5, gyroAngle);
		} else if (gyro.getAngle() < gyro.getCenter() + 180) {
			myRobot.mecanumDrive_Cartesian(0, 0, -0.5, gyroAngle);
		}
	}
}