//Include needed libraries
#include <iostream>
#include <memory>
#include <string>
#include <cmath>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <WPILib.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

const float KP_GYRO = -0.1;
const float KP_MOVEMENT = 0.1;
//5v = 5000mV, analog to digital converter scales to 0-1024.
//then, divide by 9.8, as that is the spec'd mv/in.
const float US_SCALE = 5000 / 1024 / 9.8

//Start the class definition
class Robot: public frc::IterativeRobot {

	//Declare the used variables
	frc::DoubleSolenoid* solenoid;
	frc::Joystick* joystick;
	frc::RobotDrive* robotDrive;
	frc::Joystick* xboxjoystick;
	frc::AnalogInput* ultrasonic;
	frc::AnalogGyro* gyro;
	frc::Encoder* enc;

	frc::Servo* servo;

	frc::DigitalInput* limitSwitch;

	float servoPos;
	int autoState;

	//These ones are static because the VisionThread is static.
	static bool actuate;
	static int movement;
public:

	//Startup function
	void RobotInit() {
		robotDrive = new frc::RobotDrive(0, 1, 2, 3);
		robotDrive->SetInvertedMotor(frc::RobotDrive::MotorType::kFrontRightMotor, true);
		robotDrive->SetInvertedMotor(frc::RobotDrive::MotorType::kRearRightMotor, true);

		robotDrive->SetMaxOutput(0.5);

		joystick = new frc::Joystick(0);
		joystick->SetAxisChannel(Joystick::kTwistAxis, 2);
		xboxjoystick = new frc::Joystick(1);

		solenoid = new frc::DoubleSolenoid(0, 1);

		//Start the visionThread function in a different thread.
		std::thread visionThread(VisionThread);
		visionThread.detach();

		ultrasonic = new frc::AnalogInput(2);
		gyro = new frc::AnalogGyro(0);
		enc = new frc::Encoder(2, 3, false, frc::Encoder::EncodingType::k4X);

		servo = new frc::Servo(4);
		servoPos = 0.0;

		limitSwitch = new frc::DigitalInput(5);

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
		gyro->Reset();
		autoState = 0;
	}

	void AutonomousPeriodic() {
		printf("movement: %i\n", movement);

		//We want to have multiple stages of the auto program, so autoState is used.
		//At the end of each stage, it is set to the next value to move on to the next step.
		if (autoState == 0) {
			//Move forward until a distance is reached, away from wall
			autoState = 1;
		} else if (autoState == 1) {
			//Turn 45-ish degrees
			robotDrive->StopMotor();
			autoState = 2;
		} else if (autoState == 2) {
			//Line up with tape, while moving forward until close to gear
			if (ultrasonic->GetValue() * US_SCALE > 8.0) {
				robotDrive->MecanumDrive_Cartesian(KP_MOVEMENT * movement, 0.2, KP_GYRO * gyro->GetAngle());
			} else {
				robotDrive->StopMotor();
				autoState = 3;
			}
		} else if (autoState == 3) {
			//Move forward to put gear on peg
			robotDrive->StopMotor();
			autoState = 4;
		} else if (autoState == 4) {
			//Wait for gear to be lifted out, and back up a set distance
			robotDrive->StopMotor();
			autoState = 5;
		} else if (autoState == 5) {
			//cross auto line
			robotDrive->StopMotor();
		}


		//TEST CODE

	}

	void TeleopInit() {
		solenoid->Set(frc::DoubleSolenoid::Value::kOff);
	}

	void TeleopPeriodic() {
		robotDrive->SetMaxOutput((joystick->GetRawAxis(3) - 1)/-4); //scale speed, max .5

		//printf("Distance: %i\n", limitSwitch->Get());
		printf("Distance: %f\n", ultrasonic->GetValue() * US_SCALE); //scaling factor
		printf("Encoder: %f\n", enc->GetDistance());

		//Add a dead zone
		float x = fabs(joystick->GetX()) > 0.15 ? joystick->GetX() : 0.0;
		float y = fabs(joystick->GetY()) > 0.1 ? joystick->GetY() : 0.0;
		float twist = fabs(joystick->GetTwist()) > 0.1 ? joystick->GetTwist() / 2 : 0.0;

		robotDrive->MecanumDrive_Cartesian(x, y, twist, gyro->GetAngle());
		printf("Gyro: %f\n", gyro->GetAngle());

	}

	void TestPeriodic() {
		printf("Distance: %f\n", ultrasonic->GetValue() * US_SCALE);
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
			cv::inRange(hsv, cv::Scalar(0,0,250), cv::Scalar(180,25,255), threshOutput);

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

			//cv::Mat output = cv::Mat::zeros(threshOutput.size(), CV_8UC3);
			//Draw onto camera input

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

				movement = meanPoint.x - (float)pixelCenter;

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
