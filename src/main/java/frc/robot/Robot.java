// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.RelativeEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //create our variables and options for auton
  private static final String kDefaultAuto = "Basic Auto";
  private static final String kBalanceAuto = "Balance Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private double autoSpeed = 0;
  private double autoStart = 0;
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
  private final SendableChooser<String> m_speedChooser = new SendableChooser<>();
  private double speedMultiplier = slowSpeed;

  //shoulder
  private final CANSparkMax m_shoulder = new CANSparkMax(5, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_shoulder.getEncoder();
  private final double kP = 0.016;
  private final double kI = 0.0;
  private final double kD = 0.0;
  //double kI = 0.00002;
  //double kD = 0.0001;
  private final PIDController pid = new PIDController(kP, kI, kD);
  private double setpoint = 0;
  private double start = -5;
  private double floor = 10;
  private double mid = 30;
  private double high = 42;

  //claw
  private final CANSparkMax m_claw = new CANSparkMax(4, MotorType.kBrushed);

  //CREATE CONTROLLER :)
  private final XboxController m_driveController = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);

  //Cameras
  private UsbCamera camera1;
  private UsbCamera camera2;

  //Limelight networking variables
  private final NetworkTable kLimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableValue kLimelightTXvalue = kLimelightTable.getValue("tx");
  private final NetworkTableValue kLimelightTYvalue = kLimelightTable.getValue("ty");
  private final NetworkTableValue kLimelightTAvalue = kLimelightTable.getValue("ta");
  private final NetworkTableValue kLimelightTVvalue = kLimelightTable.getValue("tv");
  
  //Limelight target variables
  private boolean limelightTargetDetected = false;
  private double limelightTargetX = 0.0;
  private double limelightTargetY = 0.0;
  private double limelightTargetArea = 0.0;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Basic Auto", kDefaultAuto);
    m_chooser.addOption("Balance Auto", kBalanceAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_speedChooser.setDefaultOption("Slow", kSlowSpeed);
    m_speedChooser.addOption("Fast", kFastSpeed);
    SmartDashboard.putData("Speed choices", m_speedChooser);

    //camera
    camera1 = CameraServer.startAutomaticCapture("Front Camera", 0);
    camera1.setResolution(480, 320);
    camera1.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    camera2 = CameraServer.startAutomaticCapture("Back camera", 1);
    camera2.setResolution(480, 320);
    camera2.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    //invert voltages of one of our motors
    m_rightMotor.setInverted(true);

    //initialize autoBalance code
    mAutoBalance = new autoBalance();
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
    //updateLimelight();
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

    //get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kBalanceAuto:
        double speed = mAutoBalance.scoreAndBalance();
        m_robotDrive.arcadeDrive(speed, 0);

        break;
      case kDefaultAuto:
      default:
        //get time since start of auto
        double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
        //series of timed events making up the flow of auto
        if(autoTimeElapsed < 2){
          //raise the arm and drive forward slowly for three seconds
          autoSpeed = .65;
          setpoint = high;
        }else if(autoTimeElapsed < 6){
          //stop and drop the game piece
          autoSpeed = 0;
        }else if(autoTimeElapsed < 15){
          //back up for four seconds to leave the community
          autoSpeed = -0.75;
        }else{
          //stop
          autoSpeed = 0;
        }

        m_robotDrive.arcadeDrive(autoSpeed, 0);
        m_shoulder.set(pid.calculate(m_encoder.getPosition(), setpoint));
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
      setpoint = start;
    }

    double pidValue = pid.calculate(m_encoder.getPosition(), setpoint);
    m_shoulder.set(pidValue);

    if(rightTrigger > 0){
      //right trigger closes the claw
      m_claw.set(-0.35);
    } else if(leftTrigger > 0) {
      //left trigger opens the claw
      m_claw.set(0.2);
    } else {
      m_claw.set(0.00);
    }

    SmartDashboard.putNumber("Arm Position: ", m_encoder.getPosition());

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /*
   * This function periodically checks the NetworkTable for updates from 
   * the limelight and updates the target variables if a valid target is found.
  */
  public void updateLimelight() {

    try {
      limelightTargetDetected = kLimelightTVvalue.getBoolean();
      limelightTargetX = kLimelightTXvalue.getDouble();
      limelightTargetY = kLimelightTYvalue.getDouble();
      limelightTargetArea = kLimelightTAvalue.getDouble();
    } catch (ClassCastException ex)
    {
      // This exception will occur if the Limelight NetworkTable can't be found,
      // which could be caused by a bad ethernet cable or something similar
    }
    

    /* You could add additional vision processing code here, but it's best 
     to keep it as inexpensive as possible and leave more computationally
     intensive tasks to a vision coprocessor */
  }
}
