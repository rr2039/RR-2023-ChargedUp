// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.DefaultAuto;
import frc.robot.autonomous.TestAuto;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.ArmExtension;
import frc.robot.commands.GripperRoll;
import frc.robot.commands.TransportPosition;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperPitchSubsystem;
import frc.robot.subsystems.GripperRollSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utilities.LimelightInterface;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LimelightInterface m_limelight = new LimelightInterface(m_robotDrive);
  private final GripperSubsystem m_gripper = new GripperSubsystem();
  private final ShoulderSubsystem m_shoulder = new ShoulderSubsystem();
  private final GripperPitchSubsystem m_gripperPitch = new GripperPitchSubsystem();
  private final GripperRollSubsystem m_gripperRoll = new GripperRollSubsystem();
  private final ArmExtensionSubsystem m_armExtension = new ArmExtensionSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  GenericHID m_operatorController = new GenericHID(OIConstants.kOperatorControllerPort);

  // Auto Chooser for Dashboard
  SendableChooser<Command> auto_chooser = new SendableChooser<>();
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(m_driverController.getRawAxis(0), 0.06),
                MathUtil.applyDeadband(-m_driverController.getRawAxis(1), 0.06),
                MathUtil.applyDeadband(-m_driverController.getRawAxis(4), 0.06),
                true),
            m_robotDrive)
    );
    m_shoulder.setDefaultCommand(
      new RunCommand(() -> m_shoulder.moveShoudler(m_operatorController.getRawAxis(1) * 0.25), m_shoulder).andThen(() -> m_shoulder.moveShoudler(0.0))
    );
    m_gripperPitch.setDefaultCommand(
      new RunCommand(() -> m_gripperPitch.moveWristPitch(m_operatorController.getRawAxis(3) * 0.25), m_gripperPitch).andThen(() -> m_gripperPitch.moveWristPitch(0.0))
    );

    // Auto Options
    auto_chooser.setDefaultOption("TestAuto", new TestAuto(m_robotDrive));
    auto_chooser.addOption("DefaultAuto", new DefaultAuto(m_robotDrive));
    SmartDashboard.putData("Auto Chooser", auto_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(new AlignToAprilTag(m_robotDrive, m_limelight).andThen(new AlignToAprilTag(m_robotDrive, m_limelight)));
    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading()));
    new JoystickButton(m_operatorController, 7)
        .whileTrue(new RunCommand(()-> m_shoulder.moveShoulderToSetpoint(), m_shoulder));

    new JoystickButton(m_operatorController, 1)
        .whileTrue(new RunCommand(() -> m_gripper.open())).onFalse(new RunCommand(() -> m_gripper.hardClose()));
    new JoystickButton(m_operatorController, 4)
        .whileTrue(new RunCommand(() -> m_gripperPitch.moveWristPitchToSetPoint(), m_gripperPitch));
    new JoystickButton(m_operatorController, 8)
        .onTrue(new TransportPosition(m_shoulder, m_gripperPitch));
    Trigger DpadRight = new POVButton(m_operatorController, 90);
    DpadRight.onTrue(new GripperRoll(m_gripperRoll, 0.1)).onFalse(new GripperRoll(m_gripperRoll, 0));
    Trigger DpadLeft = new POVButton(m_operatorController, 270);
    DpadLeft.onTrue(new GripperRoll(m_gripperRoll, -0.1)).onFalse(new GripperRoll(m_gripperRoll, 0));
    //Trigger DpadUp = new POVButton(m_operatorController, 0);
    //DpadUp.onTrue(new GripperPitch(m_gripperPitch, 0.1)).onFalse(new GripperPitch(m_gripperPitch, 0));
    //Trigger DpadDown = new POVButton(m_operatorController, 180);
    //DpadDown.onTrue(new GripperPitch(m_gripperPitch, -0.1)).onFalse(new GripperPitch(m_gripperPitch, 0));
    Trigger LeftBumper = new JoystickButton(m_operatorController, 5);
    LeftBumper.onTrue(new ArmExtension(m_armExtension, -0.25)).onFalse(new ArmExtension(m_armExtension, 0));
    Trigger RightBumper = new JoystickButton(m_operatorController, 6);
    RightBumper.onTrue(new ArmExtension(m_armExtension, 0.25)).onFalse(new ArmExtension(m_armExtension, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto_chooser.getSelected();
  }
}
