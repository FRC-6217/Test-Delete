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
const float KP_MOVEMENT = 0.01;
const float US_SCALE = 1/4.5;

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
	frc::VictorSP* winch;
	frc::VictorSP* winch2;

	frc::DigitalInput* limitSwitch;

	float servoPos;
	int autoState;
	bool debounce;
	int count;

	//These ones are static because the VisionThread is static.
	static bool actuate;
	static int movement;
	static bool process;
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
		enc->SetDistancePerPulse(-0.0211600227);

		servo = new frc::Servo(4);
		servoPos = 0.0;
		winch = new frc::VictorSP(5);
		winch2 = new frc::VictorSP(6);

		limitSwitch = new frc::DigitalInput(5);

		debounce = true;

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
		enc->Reset();
		autoState = 0;
		process = true;
	}

	void AutonomousPeriodic() {
		printf("movement: %i\n", movement);

		float distance = ultrasonic->GetValue();
		if (distance < 214.0) {
			distance = 0;
		} else {
			distance -= 214;
		}

		distance *= US_SCALE;
		distance += 10.5;


		//We want to have multiple stages of the auto program, so autoState is used.
		//At the end of each stage, it is set to the next value to move on to the next step.
		if (autoState == 0) {
			//Move forward until a distance is reached, across line
			if (enc->GetDistance() < 120.0) {
				robotDrive->MecanumDrive_Cartesian(0.0, -0.3, KP_GYRO * gyro->GetAngle());
			} else {
				robotDrive->StopMotor();
				autoState = 1;
			}
		} else if (autoState == 1) {
			//Turn 45-ish degrees
			if (gyro->GetAngle() < 45.0) {
				robotDrive->MecanumDrive_Cartesian(0.0,0.0,0.4);
			} else {
				robotDrive->StopMotor();
				autoState = 2;
				gyro->Reset();
			}
		} else if (autoState == 2) {
			//Line up with tape, while moving forward until close to gear

			if (distance > 20.0) {
				robotDrive->MecanumDrive_Cartesian(KP_MOVEMENT * movement, -0.2, KP_GYRO * gyro->GetAngle());
			} else {
				robotDrive->StopMotor();
				autoState = 4;
			}
		} else if (autoState == 3) {
			if (distance > 12.0) {
				robotDrive->MecanumDrive_Cartesian(0.0, 0.2, KP_GYRO * gyro->GetAngle());
			} else {
				robotDrive->StopMotor();
				autoState = 4;
			}
		} else if (autoState == 4) {
			//Wait for gear to be lifted out, and back up a set distance
			if (!limitSwitch->Get()) {
				robotDrive->StopMotor();
			} else {
				robotDrive->StopMotor();
				autoState = 5;
			}
		} else if (autoState == 5) {
			//cross auto line
			robotDrive->StopMotor();
		}


		//TEST CODE

	}

	void TeleopInit() {
		solenoid->Set(frc::DoubleSolenoid::Value::kOff);
		count = 0;
		enc->Reset();
	}

	void TeleopPeriodic() {
		robotDrive->SetMaxOutput((joystick->GetRawAxis(3) - 1)/-2); //scale speed
		printf("Encoder: %f\n", enc->GetDistance());

		//printf("Distance: %i\n", limitSwitch->Get());

		//Add a dead zone
		float x = fabs(joystick->GetX()) > 0.15 ? joystick->GetX() : 0.0;
		float y = fabs(joystick->GetY()) > 0.1 ? joystick->GetY() : 0.0;
		float twist = fabs(joystick->GetTwist()) > 0.1 ? joystick->GetTwist() / 2 : 0.0;

		robotDrive->MecanumDrive_Cartesian(x, y, twist, gyro->GetAngle());
		printf("Gyro: %f\n", gyro->GetAngle());

		if (joystick->GetRawButton(7)) {
			gyro->Reset();
		}

		if (joystick->GetRawButton(11) && debounce) {
			process = !process;
			debounce = false;
		} else if (joystick->GetRawButton(11) == false) {
			debounce = true;
		}

		if(joystick->GetRawButton(6)) {
			if (count < 40) {
				winch->Set(-0.3);
				winch2->Set(-0.3);
			} else if (count < 40) {
				winch->Set(0.0);
				winch2->Set(0.0);
			} else {
				count = 0;
			}
			count++;
		} else if (joystick->GetRawButton(5)) {
			if (count < 40) {
				winch->Set(-0.63);
				winch2->Set(-0.63);
			} else if (count < 40) {
					winch->Set(0.0);
					winch2->Set(0.0);
			} else {
					count = 0;
			}
			count++;
		} else if (joystick->GetRawButton(4)) {
			winch->Set(-0.1);
			winch2->Set(-0.1);
		} else {
			winch->Set(0.0);
			winch2->Set(0.0);
		}


	}

	void TestPeriodic() {
		printf("Switch: %d\n", limitSwitch->Get());

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
		cs::CvSource outputStreamStd =  CameraServer::GetInstance()->PutVideo("Output", 320, 240);
		cv::Mat source;
		cv::Mat hsv;

		cv::Mat threshOutput;
		cv::Mat out1;
		cv::Mat out2;
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;

		//main vision loop
		while(true) {
			cvSink.GrabFrame(source);

			if (process) {

				cvtColor(source, hsv, cv::COLOR_BGR2HSV);
				cv::GaussianBlur(hsv, hsv, cv::Size(5, 5), 2, 2);

				//find green
				cv::inRange(hsv, cv::Scalar(30,100,150), cv::Scalar(80,255,255), threshOutput);
				//cv::inRange(hsv, cv::Scalar(160,130,140), cv::Scalar(179,160,255), out2);
				//cv::addWeighted(out1, 1.0, out2, 1.0, 0.0, threshOutput);

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
			}
			//END TEST CODE

			//Send to driver station
			outputStreamStd.PutFrame(source);
		}
	}
};

bool Robot::actuate = false;
int Robot::movement = 0;
bool Robot::process = true;

START_ROBOT_CLASS(Robot)
