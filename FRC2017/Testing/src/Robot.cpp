#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <WPILib.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

class Robot: public frc::IterativeRobot {

	frc::DoubleSolenoid* solenoid;
	frc::Joystick* joystick;
	frc::RobotDrive* robotDrive;

	frc::AnalogInput* ultrasonic;

	static bool actuate;
	static int movement;
public:

	void RobotInit() {
		robotDrive = new RobotDrive(0, 1, 2, 3);

		joystick = new frc::Joystick(0);
		joystick->SetAxisChannel(Joystick::kTwistAxis, 3);

		solenoid = new frc::DoubleSolenoid(0, 1);

		//Start the visionThread function in a different thread.
		std::thread visionThread(VisionThread);
		visionThread.detach();

		ultrasonic = new frc::AnalogInput(1);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {
		//MAKE SURE TO UPDATE ALL MOTORS IN EVERY LOOP!

		if (actuate) {
			solenoid->Set(frc::DoubleSolenoid::Value::kForward);
		} else {
			solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
		}

		//TEST CODE
		if (movement > 0) {
			//go right
			robotDrive->MecanumDrive_Cartesian(0.5, 0.0, 0.0);
		} else if (movement < 0) {
			robotDrive->MecanumDrive_Cartesian(-0.5, 0.0, 0.0);
		} else {
			robotDrive->StopMotor();
		}
	}

	void TeleopInit() {
		solenoid->Set(frc::DoubleSolenoid::Value::kOff);
	}

	void TeleopPeriodic() {
		printf("Distance: %f\n", ultrasonic->GetVoltage()* .000977); //scaling factor

		robotDrive->MecanumDrive_Cartesian(joystick->GetX(), joystick->GetY(), joystick->GetTwist());

		if (joystick->GetTrigger()) {
			solenoid->Set(frc::DoubleSolenoid::Value::kForward);
		} else if (joystick->GetTop()) {
			solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
		} else {
			solenoid->Set(frc::DoubleSolenoid::Value::kOff);
		}
	}

	void TestPeriodic() {

	}

	static void VisionThread()
	{
		//Set up the camera
		int g_exp = 50;
		//frc::SmartDashboard::PutNumber("Exp", g_exp);
		cs::UsbCamera camera = cs::UsbCamera("usb0",0);
		camera.SetBrightness(5);
		camera.SetExposureManual(g_exp);
		//frc::SmartDashboard::PutNumber("Brightness", camera.GetBrightness());

		//Start capture, create outputs
		CameraServer::GetInstance()->StartAutomaticCapture(camera);
		camera.SetResolution(320, 240);
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		cs::CvSource outputStreamStd =  CameraServer::GetInstance()->PutVideo("Processed", 320, 240);
		cv::Mat source;
		cv::Mat hsv;

		cv::Mat threshOutput;
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;

		//main vision loop
		while(true) {
			cvSink.GrabFrame(source);
			cvtColor(source, hsv, cv::COLOR_BGR2HSV);
			cv::GaussianBlur(hsv, hsv, cv::Size(9, 9), 2, 2);

			//find green
			cv::inRange(hsv, cv::Scalar(50,100,70), cv::Scalar(70,255,250), threshOutput);

			//group nearby pixels into contours
			cv::findContours(threshOutput, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

			std::vector<std::vector<cv::Point>> contours_poly (contours.size());
			std::vector<cv::Point2f> center(contours.size());
			std::vector<float> radius(contours.size());

			std::vector<cv::Point2f> centerLarge;
			std::vector<float> radiusLarge;

			//Create a circle around contours
			for( unsigned int i = 0; i < contours.size(); i++ ) {
				cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
				minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]);
			}

			//cv::Mat output = cv::Mat::zeros(threshOutput.size(), CV_8UC3);
			//Draw onto camera input
			for (unsigned int i = 0; i < contours.size(); i++) {
				if (radius[i] > 10.0) {
					cv::Scalar color = cv::Scalar(50, 100, 200);
					//cv::drawContours(source, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
					cv::drawContours(source, contours_poly, i, color, 1, 8, hierarchy, 0, cv::Point());
					//cv::rectangle(source, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
					cv::circle(source, center[i], (int)radius[i], color, 2, 8, 0);

					centerLarge.push_back(center[i]);
					radiusLarge.push_back(radius[i]);
				}
			}

			//Possible: find center, and take midpoint to locate center of both
			//BEGIN TEST CODE

			//Find the average
			cv::Mat mean;
			int width = source.cols;
			int pixelCenter = width / 2;

			if (centerLarge.size() > 0) {
				actuate = true;

				cv::reduce(centerLarge, mean, 1, cv::ReduceTypes::REDUCE_AVG);

				cv::Point2f meanPoint(mean.at<float>(0,0), mean.at<float>(0,1));
				cv::circle(source, meanPoint, 3, cv::Scalar(0, 0, 255), -1, 8, 0);

				if (meanPoint.x > (float)pixelCenter) {
					//go right
					movement = 1;
				} else {
					//go left
					movement = -1;
				}
			} else {
				actuate = false;
			}
			//END TEST CODE

			//Send to driver station
			outputStreamStd.PutFrame(source);
		}
	}
};

bool Robot::actuate = false;
int Robot::movement = 0;

START_ROBOT_CLASS(Robot)
