// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//ROBOT
import edu.wpi.first.wpilibj.TimedRobot;

//CAMERA
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.cscore.VideoMode.PixelFormat;

//PID + Encoder
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

//VISION
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

//AUTO
import edu.wpi.first.wpilibj.Timer;

//CONTROLLER
import edu.wpi.first.wpilibj.XboxController;

//DRIVE TRAIN
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//MOTOR CONTROLLERS
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//SMART DASHBOARD
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//CAN LIGHTS

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //create our variables and options for auton
  private static final String kScoreAuto = "Score Auto";
  private static final String kScoreAndDriveAuto = "Score + Drive Auto";
  private static final String kScoreAndBalanceAuto = "Score + Balance Auto";
  private static final String kBalanceAuto = "Balance Auto";
  private String m_autoSelected;
  private SendableChooser<String> m_chooser;
  private autoBalance mAutoBalance;

  //create motors for drivetrain
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  //create our drive and speed options
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private static final String kSlowSpeed = "Slow Speed";
  private static final String kFastSpeed = "Fast Speed";
  private static final double fastSpeed = 1.0;
  private static final double slowSpeed = 0.75;
  private String m_speedSelected;
  private SendableChooser<String> m_speedChooser;
  private double speedMultiplier = slowSpeed;

  //shoulder
  private CANSparkMax m_shoulder;
  private RelativeEncoder m_encoder;
  private final double kP = 0.016;
  private final double kI = 0.002;
  private final double kD = 0.0;
  private final PIDController pid = new PIDController(kP, kI, kD);
  private double setpoint = 0;
  private double start = -1;
  private double floor = 10;
  private double mid = 41;
  private double high = 48;
  private double player_station = 45;
  private double feedForward = 0.01;

  //claw
  private CANSparkMax m_claw;
  private final double closeClawSpeed = -0.3;
  private final double openClawSpeed = 0.35;

  //CREATE CONTROLLER :)
  private final XboxController m_driveController = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);

  //Cameras
  private UsbCamera camera1;
  //private UsbCamera camera2;

  //Limelight networking variables
  private final NetworkTable kLimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableValue kLimelightTXvalue = kLimelightTable.getValue("tx");
  private final NetworkTableValue kLimelightTYvalue = kLimelightTable.getValue("ty");
  private final NetworkTableValue kLimelightTAvalue = kLimelightTable.getValue("ta");
  private final NetworkTableValue kLimelightTVvalue = kLimelightTable.getValue("tv");

  //Setup lights
  CANLight lights;
  private static final String kRedTeam = "Red Alliance";
  private static final String kBlueTeam = "Blue Alliance";
  private String m_colorSelected;
  private SendableChooser<String> m_colorChooser;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //Add autobalance options to dashboard
    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption(kScoreAuto, kScoreAuto);
    m_chooser.addOption(kScoreAndDriveAuto, kScoreAndDriveAuto);
    m_chooser.addOption(kScoreAndBalanceAuto, kScoreAndBalanceAuto);
    m_chooser.addOption(kBalanceAuto, kBalanceAuto);
    SmartDashboard.putData("Auto mode:", m_chooser);

    //initialize autoBalance code
    mAutoBalance = new autoBalance();

    //Add speed options to dashboard
    m_speedChooser = new SendableChooser<>();
    m_speedChooser.setDefaultOption("Slow", kSlowSpeed);
    m_speedChooser.addOption("Fast", kFastSpeed);
    SmartDashboard.putData("Speed:", m_speedChooser);

    //Setup lights and color options
    lights = new CANLight(0);
    //red
    lights.writeRegister(0, 30, 255, 0, 0);
    //blue
    lights.writeRegister(1, 30, 0, 0, 255);

    m_colorChooser = new SendableChooser<>();
    m_colorChooser.setDefaultOption(kBlueTeam, kBlueTeam);
    m_colorChooser.addOption(kRedTeam, kRedTeam);
    SmartDashboard.putData("Color:", m_colorChooser);

    //setup our front camera
    camera1 = CameraServer.startAutomaticCapture("Front Camera", 0);
    camera1.setResolution(320, 240);
    camera1.setFPS(15);

    //setup our second camera
    // camera2 = CameraServer.startAutomaticCapture("Back camera", 1);
    // camera2.setResolution(240, 180);
    // camera2.setPixelFormat(PixelFormat.kMJPEG);
    // camera2.setFPS(15);
    // camera2.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    //invert voltages of one of our motors
    m_rightMotor.setInverted(true);

    //set a current limit for the claw
    m_claw = new CANSparkMax(4, MotorType.kBrushed);
    m_claw.setSmartCurrentLimit(20);

    //initialize arm spark max so the encoder value gets reset
    m_shoulder = new CANSparkMax(5, MotorType.kBrushless);
    m_shoulder.restoreFactoryDefaults();
    m_encoder = m_shoulder.getEncoder();
    m_encoder.setPosition(0);
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Tilt: ", mAutoBalance.getTilt());

    m_colorSelected = m_colorChooser.getSelected();
    switch(m_colorSelected){
      case kRedTeam:
        lights.showRegister(0);
        break;
      case kBlueTeam:
      default:
        lights.showRegister(1);
        break;
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_autoSelected = m_chooser.getSelected();
    double speed;

    switch (m_autoSelected) {
      case kScoreAndBalanceAuto:
        speed = mAutoBalance.scoreAndBalance();
        SmartDashboard.putNumber("Auto speed: ", speed);
        m_robotDrive.arcadeDrive(speed, 0);
        break;
      case kBalanceAuto:
        speed = mAutoBalance.autoBalanceRoutine();
        SmartDashboard.putNumber("Auto speed: ", speed);
        m_robotDrive.arcadeDrive(speed, 0);
        break;
      case kScoreAndDriveAuto:
        speed = mAutoBalance.scoreAndDrive();
        SmartDashboard.putNumber("Auto speed: ", speed);
        m_robotDrive.arcadeDrive(speed, 0);
        break;
      case kScoreAuto:
      default:
        speed = mAutoBalance.score();
        SmartDashboard.putNumber("Auto speed: ", speed);
        m_robotDrive.arcadeDrive(speed, 0);
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_speedSelected = m_speedChooser.getSelected();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //set drive speed
    m_speedSelected = m_speedChooser.getSelected();
    switch (m_speedSelected) {
      case kFastSpeed:
        speedMultiplier = fastSpeed;
        break;
      case kSlowSpeed:
      default:
        // Put default auto code here
        speedMultiplier = slowSpeed;
        break;
    }

    //Drive with split arcade drive
    double leftStick = -m_driveController.getLeftY() * speedMultiplier;
    double rightStick = -m_driveController.getRightX() * speedMultiplier;
    m_robotDrive.arcadeDrive(leftStick, rightStick);

    double leftTrigger = m_operatorController.getLeftTriggerAxis();
    double rightTrigger = m_operatorController.getRightTriggerAxis();
    boolean rightBumperPressed = m_operatorController.getRightBumperPressed();
    boolean leftBumperPressed = m_operatorController.getLeftBumperPressed();
    boolean aButtonPressed = m_operatorController.getAButtonPressed();
    boolean bButtonPressed = m_operatorController.getBButtonPressed();
    boolean yButtonPressed = m_operatorController.getYButtonPressed();
    boolean xButtonPressed = m_operatorController.getXButtonPressed();
    boolean backButtonPressed = m_operatorController.getBackButtonPressed();

    // intentionally putting this in separate if/else to ensure accidental back button presses are overridden

    if(backButtonPressed){
      setpoint = start;
    }

    if(aButtonPressed){
      //a button set the arm to floor
      setpoint = floor;
    } else if(bButtonPressed) {
      //b button set the arm to mid
      setpoint = mid;
    } else if(yButtonPressed) {
      //y button set the arm to high
      setpoint = high;
    } else if(xButtonPressed) {
      //x button set the arm to retract
      setpoint = player_station;
    }

    double pidValue = pid.calculate(m_encoder.getPosition(), setpoint);
    m_shoulder.set(feedForward + pidValue);


    if(rightTrigger > 0){
      //right trigger closes the claw
      m_claw.set(closeClawSpeed);
    } else if(leftTrigger > 0) {
      //left trigger opens the claw
      m_claw.set(openClawSpeed);
    } else {
      m_claw.set(0.00);
    }

    SmartDashboard.putNumber("Arm Position: ", m_encoder.getPosition());
    SmartDashboard.putNumber("PID", pidValue);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {mAutoBalance = new autoBalance();}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {System.out.println("Tilt: "+ mAutoBalance.getTilt());}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
