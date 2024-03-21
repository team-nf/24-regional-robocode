// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.File;
// import java.lang.reflect.Array;
import java.util.Arrays;
// import java.util.List;
// import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  int connListenerHandle;

  private final SendableChooser<Command> autoChooser;
  
  NetworkTable vision_nt = ntInst.getTable("vision"); 
  // Arrays from nf-vision always hold 4 double values. 
  // The first three values are co-ordinates, distance, heading whatever; while the last value can be either -1, 0 or 1
  // -1 means the subscriber has returned its default value, 0 means nothing is detected and 1 means something is detected.
  final DoubleArraySubscriber apriltag_sub = vision_nt.getDoubleArrayTopic("apriltag").subscribe(new double[]{0.0, 0.0, 0.0, -1.0});
  final DoubleArraySubscriber stereovision_sub = vision_nt.getDoubleArrayTopic("stereo-vision").subscribe(new double[]{0.0, 0.0, 0.0, -1.0});
  final DoubleArraySubscriber objectdetection_sub = vision_nt.getDoubleArrayTopic("darknet").subscribe(new double[]{0.0, 0.0, 0.0, -1.0});

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/teamnf"));
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  // private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  //XboxController driverXbox = new XboxController(0);
  //CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandGenericHID m_controller = new CommandGenericHID(OperatorConstants.CONTROLLER_PORT);
  CommandGenericHID m_testController = new CommandGenericHID(4);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    NamedCommands.registerCommand("intake", m_intake.runIntakeCommand());
    NamedCommands.registerCommand("shooter", m_shooter.runThrowerCommand());
    
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // add a connection listener; the first parameter will cause the
    // callback to be called immediately for any current connections
    connListenerHandle = ntInst.addConnectionListener(true, event -> {
      if (event.is(NetworkTableEvent.Kind.kConnected)) {
        System.out.println("Connected to " + event.connInfo.remote_id);
        if ( SmartDashboard.getStringArray("Connections", new String[]{"None"})[0] != "None") {
          String[] connected = SmartDashboard.getStringArray("Connections", new String[]{});
          String[] connections =  Arrays.copyOf(connected, connected.length + 1);
          connections[connected.length + 1] = event.connInfo.remote_id;
          SmartDashboard.putStringArray("Connections", connections);
        } else {
          SmartDashboard.putStringArray("Connections", new String[] {event.connInfo.remote_id});
        }
      } else if (event.is(NetworkTableEvent.Kind.kDisconnected)) {
        System.out.println("Disconnected from " + event.connInfo.remote_id);
        if ( Arrays.toString(SmartDashboard.getStringArray("Connections", new String[]{})).contains(event.connInfo.remote_id)) {
          String[] connected = SmartDashboard.getStringArray("Connections", new String[]{});
          String[] connections = new String[connected.length];
          for (String c : connected) { int i = 0; if (!c.equals(event.connInfo.remote_id)){ connections[i++] = c;}};
          SmartDashboard.putStringArray("Connections", connections);
        }
      }
    });

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // VISION
    SmartDashboard.putNumberArray("Apriltag Detection Output", apriltag_sub.get());
    SmartDashboard.putNumberArray("Stereo Vision Output", stereovision_sub.get());
    SmartDashboard.putNumberArray("Object Detection Output", objectdetection_sub.get());
    
    SmartDashboard.putNumberArray(
      "Latest updates from nf-vision: [ado, svo, odo]", 
    new double[]{apriltag_sub.getLastChange(), stereovision_sub.getLastChange(), objectdetection_sub.getLastChange()}
    );

    // Configure the trigger bindings
    configureBindings();
    // configureTestingBindings();
    
    // Generic HID Controller axis numbers and button numbers must be arranged.
    // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
    //                                                                () -> MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_Y_AXIS),
    //                                                                                             OperatorConstants.LEFT_Y_DEADBAND),
    //                                                                () -> MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_X_AXIS),
    //                                                                                             OperatorConstants.LEFT_X_DEADBAND),
    //                                                                () -> MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.RIGHT_X_AXIS),
    //                                                                                             OperatorConstants.RIGHT_X_DEADBAND),
    //                                                                m_controller.button(OperatorConstants.BUTTON_Y)::getAsBoolean,
    //                                                                m_controller.button(OperatorConstants.BUTTON_A)::getAsBoolean,
    //                                                                m_controller.button(OperatorConstants.BUTTON_X)::getAsBoolean,
    //                                                                m_controller.button(OperatorConstants.BUTTON_B)::getAsBoolean);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_Y_AXIS), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_X_AXIS), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> m_controller.getRawAxis(OperatorConstants.RIGHT_X_AXIS),
    //     () -> m_controller.getRawAxis(OperatorConstants.LEFT_X_AXIS));

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_Y_AXIS), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_X_AXIS), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> m_controller.getRawAxis(OperatorConstants.RIGHT_X_AXIS));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_Y_AXIS), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_X_AXIS), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_controller.getRawAxis(OperatorConstants.RIGHT_X_AXIS));


    double driveK = 1;
    double angleK = 0.85;
    Command driveRobotOrientedAngularVelocity = drivebase.robotCentricDriveCommand(
        () -> (MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_Y_AXIS), OperatorConstants.LEFT_Y_DEADBAND) * driveK),
        () -> MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_X_AXIS), OperatorConstants.LEFT_X_DEADBAND) * driveK,
        () -> m_controller.getRawAxis(OperatorConstants.RIGHT_X_AXIS) * angleK);

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveRobotOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);

    //drivebase.registerVisionReading(0, apriltag_sub::get);
    //drivebase.registerVisionReading(1, stereovision_sub::get);
    //drivebase.registerVisionReading(2, objectdetection_sub::get);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    // These are binding configurations that were inherited from YAGSL by circumstance, they are rewritten below.   
    //new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    //new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    //new JoystickButton(driverXbox, 2).whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));

    m_controller.button(OperatorConstants.BUTTON_A).whileTrue(
      new StartEndCommand(
        () -> m_intake.setVoltage(12),
        () -> m_intake.setVoltage(0)
      )
    );

    m_controller.button(OperatorConstants.BUTTON_X).whileTrue(
      new StartEndCommand(
        () -> m_shooter.setFeederVoltage(12),
        () -> m_shooter.setFeederVoltage(0)
      )
    );

    m_controller.button(OperatorConstants.BUTTON_Y).whileTrue(
      m_shooter.runThrowerCommand()
    );
    m_controller.button(OperatorConstants.BUTTON_Y).whileFalse(
      m_shooter.stopThrowerCommand()
    );

    m_controller.button(OperatorConstants.BUTTON_B).whileTrue(
      m_shooter.setAngleCommand(connListenerHandle)
    );

    // m_controller.button(OperatorConstants.ZERO_GYRO_BUTTON).onTrue(new InstantCommand(drivebase::zeroGyro));
    // m_controller.button(OperatorConstants.FAKE_VISION_TRIGGER).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // /** Bu napıyor?? */
    // m_controller.button(OperatorConstants.ACTION_TRIGGER).whileTrue(
    //   Commands.deferredProxy(
    //       () -> drivebase.driveToPose(
    //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))
    //       )
    //     ));

    // m_controller.button(OperatorConstants.LEFT_BUMPER).whileTrue(m_intake.runIntakeCommand()).onFalse(m_intake.stopIntakeCommand());
    // m_controller.button(OperatorConstants.RIGHT_BUMPER).whileTrue(m_shooter.throwWithVoltageCommand());
    
    //m_controller.button(OperatorConstants.LOCK_DRIVEBASE_TRIGGER).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Return whichever autonomous route is selected from SmartDashboard.
    return autoChooser.getSelected();
  }

  /**
   * Configures button bindings /specific/ to the test mode.
   * Test mode uses a controller on usb 4 instead of the default.
   * Each button runs a different System Identification Routine while held.
   */
   public void configureTestingBindings() {
     // Shooter SysId Routines
     m_testController.button(OperatorConstants.LEFT_BUMPER).whileTrue(m_shooter.sysIdDynamic(Direction.kForward));
     m_testController.button(OperatorConstants.RIGHT_BUMPER).whileTrue(m_shooter.sysIdDynamic(Direction.kReverse));
     m_testController.axisGreaterThan(OperatorConstants.LEFT_TRIGGER, 0.5).whileTrue(m_shooter.sysIdQuasistatic(Direction.kForward));
     m_testController.axisGreaterThan(OperatorConstants.RIGHT_TRIGGER, 0.5).whileTrue(m_shooter.sysIdQuasistatic(Direction.kReverse));


    m_testController.button(OperatorConstants.LOCK_DRIVEBASE_TRIGGER).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

     // Swerve SysId Routines
     m_testController.button(OperatorConstants.BUTTON_A).whileTrue(drivebase.sysIdDriveMotorCommand());
     m_testController.button(OperatorConstants.BUTTON_B).whileTrue(drivebase.sysIdAngleMotorCommand());
   }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void closeNT() {
    ntInst.removeListener(connListenerHandle);

    apriltag_sub.close();
    stereovision_sub.close();
    objectdetection_sub.close();
  }
}
