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
#include "PIDNumSource.h"
#include "PIDNumOutput.h"

#include "Autonomous.h"

/* Arduino control chart
 * | Outputs   |Effect
 * |p3 |p4 |p5 |
 * ___________________
 * | 0 | 0 | 0 | Off
 * | 0 | 0 | 1 | Autonomous
 * | 0 | 1 | 0 | Lined up with peg
 * | 0 | 1 | 1 | Teleop normal
 * | 1 | 0 | 0 |
 * | 1 | 0 | 1 |
 * | 1 | 1 | 0 |
 * | 1 | 1 | 1 |
 */

//Start the class definition
class Robot: public frc::IterativeRobot {

    //Declare the used variables
    frc::Joystick* joystick;
    frc::RobotDrive* robotDrive;
    frc::Joystick* xboxjoystick;
    frc::AnalogInput* ultrasonic;
    frc::AnalogGyro* gyro;
    frc::Encoder* enc;

    frc::VictorSP* winch;
    frc::Spark* shooter;
    frc::Spark* revolver;

    frc::DigitalInput* limitSwitch;

    frc::SendableChooser<const int*>* autoChooser;

    //for communicating with arduino
    frc::DigitalOutput* arduino[3];
    frc::SerialPort* serial;

    const int CROSS = 0;
    const int GEAR_LEFT = 1;
    const int GEAR_CENTER = 2;
    const int GEAR_RIGHT = 3;
    const int BALLS_RED = 4;
    const int BALLS_BLUE = 5;
    const int NOTHING = -1;

    float servoPos;
    int autoState;
    bool debounce;
    bool debounceTwo;
    bool debouncePower;
    bool lockRot;
    int angleOffset;
    int count;
    bool relative;
    double shooterPower;

    int ballNext;

    //These ones are static because the VisionThread is static.
    static bool actuate;
    static int movement;
    static bool cameraToggle;
    static PIDNumSource* visionSource;

    PIDNumOutput* visionOutput;
    frc::PIDController* visionControl;

    static double values[6];
    public:

    Robot() {
        visionSource = new PIDNumSource(0.0);
    }

    //Startup function
    void RobotInit() {

        frc::SmartDashboard::PutNumber("H_Low",0.0);
        frc::SmartDashboard::PutNumber("S_Low",0.0);
        frc::SmartDashboard::PutNumber("V_Low",0.0);
        frc::SmartDashboard::PutNumber("H_High",0.0);
        frc::SmartDashboard::PutNumber("S_High",0.0);
        frc::SmartDashboard::PutNumber("V_High",0.0);

        visionOutput = new PIDNumOutput();
        visionControl = new frc::PIDController(-0.004, -0.00005, 0.0, visionSource, visionOutput);
        visionControl->SetInputRange(-0.3,0.3);


        //template is broken, need to use pointers
        autoChooser = new frc::SendableChooser<const int*>();
        autoChooser->AddDefault("Cross Line", &CROSS);
        autoChooser->AddObject("Left", &GEAR_LEFT);
        autoChooser->AddObject("Center", &GEAR_CENTER);
        autoChooser->AddObject("Right", &GEAR_RIGHT);
        autoChooser->AddObject("Balls (Red)", &BALLS_RED);
        autoChooser->AddObject("Balls (Blue)", &BALLS_BLUE);
        autoChooser->AddObject("Do Nothing", &NOTHING);
        frc::SmartDashboard::PutData("Auto mode", autoChooser);
        frc::SmartDashboard::PutNumber("Shooter Power", 0.0);
        frc::SmartDashboard::PutString("Drive Mode", "Robot");
        frc::SmartDashboard::PutNumber("Gyro", 0.0);
        frc::SmartDashboard::PutNumber("Encoder", 0.0);
        frc::SmartDashboard::PutNumber("Ultrasonic", 0.0);

        robotDrive = new frc::RobotDrive(0, 1, 2, 3);
        robotDrive->SetInvertedMotor(frc::RobotDrive::MotorType::kFrontRightMotor, true);
        robotDrive->SetInvertedMotor(frc::RobotDrive::MotorType::kRearRightMotor, true);

        robotDrive->SetMaxOutput(0.5);

        joystick = new frc::Joystick(0);
        joystick->SetAxisChannel(Joystick::kTwistAxis, 2);
        xboxjoystick = new frc::Joystick(1);

        arduino[0] = new DigitalOutput(3);
        arduino[1] = new DigitalOutput(4);
        arduino[2] = new DigitalOutput(5);
        serial = new frc::SerialPort(9600, frc::SerialPort::kUSB1);

        //Start the visionThread function in a different thread.
        std::thread visionThread(VisionThread);
        visionThread.detach();

        ultrasonic = new frc::AnalogInput(2);
        gyro = new frc::AnalogGyro(0);
        enc = new frc::Encoder(0, 1, false, frc::Encoder::EncodingType::k4X);
        enc->SetDistancePerPulse(-0.0211600227);

        winch = new frc::VictorSP(5);
        shooter = new frc::Spark(6);
        revolver = new frc::Spark(7);

        limitSwitch = new frc::DigitalInput(2);

        debounce = true;
        debounceTwo = true;
        debouncePower = true;
        lockRot = false;
        angleOffset = 0;
        relative = false;
        shooterPower = 0.6;

        ballNext = 0;

        Autonomous::AutoInit(enc, robotDrive, gyro, limitSwitch, shooter, revolver);

        arduino[0]->Set(false);
        arduino[1]->Set(false);
        arduino[2]->Set(false);
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
        robotDrive->SetMaxOutput(1.0);
        visionControl->Enable();
        gyro->Reset();
        enc->Reset();
        Autonomous::autoState = 0;
        cameraToggle = true;

        arduino[0]->Set(false);
        arduino[1]->Set(false);
        arduino[2]->Set(true);
    }

    void AutonomousPeriodic() {
        visionControl->SetSetpoint(0.0);

        winch->Set(0.0);

        float distance = ultrasonic->GetValue();
        if (distance < 214.0) {
            distance = 0;
        } else {
            distance -= 214;
        }

        distance *= US_SCALE;
        distance += 10.5;

        frc::SmartDashboard::PutNumber("Ultrasonic", distance);
        frc::SmartDashboard::PutNumber("Gyro", gyro->GetAngle());
        frc::SmartDashboard::PutNumber("Encoder", enc->GetDistance());
        frc::SmartDashboard::PutNumber("PID", visionOutput->getValue());

        Autonomous::distance = distance;
        Autonomous::movement = visionOutput->getValue();

        const int result = *autoChooser->GetSelected();
        if (result == CROSS) {
            Autonomous::forward();
        } else if (result == GEAR_LEFT || *ballNext == 1) {
            Autonomous::baseGearLeft();
        } else if (result == GEAR_CENTER) {
            Autonomous::baseGearCenter();
        } else if (result == GEAR_RIGHT || *ballNext == 2) {
            Autonomous::baseGearRight();
        } else if (result == BALLS_RED) {
            Autonomous::ballShooter(&ballNext, false);
        } else if (result == BALLS_BLUE) {
        	Autonomous::ballShooter(&ballNext, true);
        } else {
        	//do nothing.
        	shooter->Set(0.0);
        	revolver->Set(0.0);
        	robotDrive->StopMotor();
        }
    }

    void TeleopInit() {
        visionControl->Disable();
        count = 0;
        enc->Reset();
        lockRot = false;
        relative = true;
    }

    void TeleopPeriodic() {
        visionControl->SetSetpoint(0.0);

        frc::SmartDashboard::PutNumber("Gyro", gyro->GetAngle());
        frc::SmartDashboard::PutNumber("Encoder", enc->GetDistance());
        frc::SmartDashboard::PutNumber("Shooter Power", shooterPower);
        frc::SmartDashboard::PutNumber("PID", visionOutput->getValue());

        robotDrive->SetMaxOutput((joystick->GetRawAxis(3) - 1)/-2); //scale speed

        Autonomous::movement = visionOutput->getValue();
        printf("vision: %f\n", visionOutput->getValue());

        //printf("Distance: %i\n", limitSwitch->Get());

        //Add a dead zone
        float x = fabs(joystick->GetX()) > 0.15 ? joystick->GetX() : 0.0;
        float y = fabs(joystick->GetY()) > 0.1 ? joystick->GetY() : 0.0;
        float twist = fabs(joystick->GetTwist()) > 0.1 ? joystick->GetTwist() / 2 : 0.0;

        //Hold to stay lined up with gear
        if (joystick->GetRawButton(2)) {
            if (!lockRot) {
                lockRot = true;
                angleOffset = gyro->GetAngle() * -1;
                visionControl->Enable();
            }
            robotDrive->MecanumDrive_Cartesian(Autonomous::movement, y, KP_GYRO * (gyro->GetAngle() + angleOffset));

            if (visionControl->GetError() < 3) {
                arduino[0]->Set(false);
                arduino[1]->Set(true);
                arduino[2]->Set(false);
            } else {
                arduino[0]->Set(false);
                arduino[1]->Set(true);
                arduino[2]->Set(true);
            }
        } else {
            arduino[0]->Set(false);
            arduino[1]->Set(true);
            arduino[2]->Set(true);
            visionControl->Disable();
            lockRot = false;
            if (!relative) {
                //Move relative to the field
                robotDrive->MecanumDrive_Cartesian(x, y, twist, gyro->GetAngle());
            } else {
                //Move relative to the robot
                robotDrive->MecanumDrive_Cartesian(x, y, twist);
            }
        }

        //Reset the gyro for field-oriented driving
        if (joystick->GetRawButton(7)) {
            gyro->Reset();
        }

        //toggle camera
        if (xboxjoystick->GetRawButton(8) && debounce) {
            cameraToggle = !cameraToggle;
            debounce = false;
        } else if (xboxjoystick->GetRawButton(8) == false) {
            debounce = true;
        }

        //toggle drive mode
        if (joystick->GetRawButton(11) && debounceTwo) {
            relative = !relative;
            if (relative) {
                frc::SmartDashboard::PutString("Drive Mode", "Robot");
            } else {
                frc::SmartDashboard::PutString("Drive Mode", "Field");
            }
            debounceTwo = false;
        } else if (joystick->GetRawButton(11) == false) {
            debounceTwo = true;
        }

        //Move winch medium (X)
        if(xboxjoystick->GetRawButton(3)) {
            if (count < 40) {
                winch->Set(0.4);
            } else if (count < 40) {
                winch->Set(0.0);
            } else {
                count = 0;
            }
            count++;
            //move winch fast (A)
        } else if (xboxjoystick->GetRawButton(1)) {
            if (count < 40) {
                winch->Set(0.87);
            } else if (count < 40) {
                winch->Set(0.0);
            } else {
                count = 0;
            }
            count++;
            //move winch very slow (B)
        } else if (xboxjoystick->GetRawButton(2)) {
            winch->Set(0.2);
        } else {
            winch->Set(0.0);
        }

        //spin revolver
        if (xboxjoystick->GetRawAxis(2) > 0.8) {
            revolver->Set(-0.7);
        } else {
            revolver->Set(0.0);
        }

        //spin the shooter (right trigger)
        if (xboxjoystick->GetRawAxis(3) > 0.8) {
            shooter->Set(shooterPower);
        } else {
            shooter->Set(0);
        }

        //Change the shooter power (RB - up) (LB - down) (Y - reset)
        if (xboxjoystick->GetRawButton(6) && debouncePower) {
            debouncePower = false;
            shooterPower += 0.1;
            if (shooterPower > 1.0) {
                shooterPower = 1.0;
            }
        } else if (xboxjoystick->GetRawButton(5) && debouncePower) {
            debouncePower = false;
            shooterPower -= 0.1;
            if (shooterPower < 0.0) {
                shooterPower = 0.0;
            }
        } else if (xboxjoystick->GetRawButton(4) && debouncePower) {
            debouncePower = false;
            shooterPower = 0.6;
        } else if (xboxjoystick->GetRawButton(6) == false && xboxjoystick->GetRawButton(5) == false && xboxjoystick->GetRawButton(4) == false) {
            debouncePower = true;
        }

        values[0] = frc::SmartDashboard::GetNumber("H_Low",0.0);
        values[1] = frc::SmartDashboard::GetNumber("S_Low",0.0);
        values[2] = frc::SmartDashboard::GetNumber("V_Low",0.0);
        values[3] = frc::SmartDashboard::GetNumber("H_High",0.0);
        values[4] = frc::SmartDashboard::GetNumber("S_High",0.0);
        values[5] = frc::SmartDashboard::GetNumber("V_High",0.0);
        printf("Values %f\n", values[0]);
    }

    void TestPeriodic() {
        printf("Switch: %d\n", limitSwitch->Get());
        uint8_t data[1];
        char* read = new char[serial->GetBytesReceived()];
        serial->Read(read, serial->GetBytesReceived());
        frc::SmartDashboard::PutString("Serial\n", std::string(read));
    }

    void DisabledInit() {
        arduino[0]->Set(false);
        arduino[1]->Set(false);
        arduino[2]->Set(false);
    }

    static void VisionThread()
    {
        //Set up the camera
        int g_exp = 50;

        cs::UsbCamera camera = cs::UsbCamera("usb0",1);
        camera.SetBrightness(5);
        camera.SetExposureManual(g_exp);

        cs::UsbCamera backCamera = cs::UsbCamera("usb1", 0);
        backCamera.SetBrightness(5);
        backCamera.SetExposureManual(g_exp);
        //frc::SmartDashboard::PutNumber("Brightness", camera.GetBrightness());

        //Start capture, create outputs
        CameraServer::GetInstance()->StartAutomaticCapture(camera);
        CameraServer::GetInstance()->StartAutomaticCapture(backCamera);

        camera.SetResolution(320, 240);
        backCamera.SetResolution(320, 240);

        cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo(camera);
        cs::CvSink backSink = CameraServer::GetInstance()->GetVideo(backCamera);

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
            if (cameraToggle) {
                cvSink.GrabFrame(source);

                cvtColor(source, hsv, cv::COLOR_BGR2HSV);
                //cv::GaussianBlur(hsv, hsv, cv::Size(5, 5), 2, 2);


                //find green
                //cv::inRange(hsv, cv::Scalar(values[0],values[1],values[2]), cv::Scalar(values[3],values[4],values[5]), threshOutput);
                cv::inRange(hsv, cv::Scalar(70,160,220), cv::Scalar(90,250,255), threshOutput);
                //cv::inRange(hsv, cv::Scalar(160,130,140), cv::Scalar(179,160,255), out2);
                //cv::addWeighted(out1, 1.0, out2, 1.0, 0.0, threshOutput);

                //group nearby pixels into contours
                cv::findContours(threshOutput, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

                std::vector<std::vector<cv::Point>> contours_poly (contours.size());
                std::vector<cv::Point2f> center(contours.size());
                std::vector<float> radius(contours.size());

                std::vector<cv::Point2f> centerLarge;
                std::vector<float> radiusLarge;

                int maxY = source.cols;
                int secY = source.cols;
                int maxIndex = -1;
                int secIndex = -1;
                //Create a circle around contours
                for( unsigned int i = 0; i < contours.size(); i++ ) {
                    cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
                    minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]);

                    if (radius[i] > 10.0) {
                        if (center[i].y < maxY) {
                            secY = maxY;
                            secIndex = maxIndex;
                            maxY = center[i].y;
                            maxIndex = i;
                        } else if (center[i].y < secY) {
                            secY = center[i].y;
                            secIndex = i;
                        }
                        cv::Scalar color = cv::Scalar(50, 100, 200);
                        //cv::drawContours(source, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
                        cv::drawContours(source, contours_poly, i, color, 1, 8, hierarchy, 0, cv::Point());
                        //cv::rectangle(source, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
                    }
                }

                //draw a line
                cv::line(source, cv::Point(source.cols/2,0), cv::Point(source.cols/2,source.rows), cv::Scalar(0,0,255), 1);

                //cv::Mat output = cv::Mat::zeros(threshOutput.size(), CV_8UC3);
                //Draw onto camera input

                //Possible: find center, and take midpoint to locate center of both
                //BEGIN TEST CODE

                //Find the average
                cv::Mat mean;
                int width = source.cols;
                int pixelCenter = width / 2;

                if (maxIndex > -1 && secIndex > -1) {
                    centerLarge.push_back(center[maxIndex]);
                    centerLarge.push_back(center[secIndex]);
                    cv::circle(source, center[maxIndex], (int)radius[maxIndex], cv::Scalar(50, 100, 200), 2, 8, 0);
                    cv::circle(source, center[secIndex], (int)radius[secIndex], cv::Scalar(50, 100, 200), 2, 8, 0);
                    actuate = true;

                    cv::reduce(centerLarge, mean, 1, cv::ReduceTypes::REDUCE_AVG);

                    cv::Point2f meanPoint(mean.at<float>(0,0), mean.at<float>(0,1));
                    cv::circle(source, meanPoint, 3, cv::Scalar(0, 0, 255), -1, 8, 0);

                    visionSource->setInput((double)meanPoint.x - (double)pixelCenter);

                } else {
                    actuate = false;
                    Autonomous::movement = 0.0;
                    visionSource->setInput(0.0);
                }
            } else {
                backSink.GrabFrame(source);
            }

            //Send to driver station
            outputStreamStd.PutFrame(source);
        }
    }
};

bool Robot::actuate = false;
int Robot::movement = 0;
bool Robot::cameraToggle = true;
PIDNumSource* Robot::visionSource = nullptr;
double Robot::values[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

START_ROBOT_CLASS(Robot)
