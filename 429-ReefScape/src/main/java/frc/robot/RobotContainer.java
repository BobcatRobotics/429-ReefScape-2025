// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import BobcatLib.Hardware.Controllers.EightBitDo;
import BobcatLib.Hardware.Controllers.OI;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Containers.SwerveBase;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.PIDConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.Alliance;
import BobcatLib.Subsystems.Swerve.Utility.LoadablePathPlannerAuto;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.RollerSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends SwerveBase {
        public final ArmSubsystem m_arm = new ArmSubsystem();
        public final RollerSubsystem m_roller = new RollerSubsystem();
        public final Climber m_climber = new Climber();

        public Command armUpCommand = new InstantCommand(()->m_arm.runArm(Constants.ArmConstants.ARM_SPEED_UP));
        public Command armDownCommand = new InstantCommand(()->m_arm.runArm(Constants.ArmConstants.ARM_SPEED_DOWN));
        public Command rollerInCommand = new InstantCommand(()->m_roller.runRoller(Constants.RollerConstants.ROLLER_SPEED_IN));
        public Command rollerOutCommand = new InstantCommand(()->m_roller.runRoller(Constants.RollerConstants.ROLLER_SPEED_OUT));
        public Command climberWinchIn = new InstantCommand(()->m_climber.runClimber(Constants.ClimberConstants.CLIMBER_SPEED_IN));
        public Command climberWinchOut = new InstantCommand(()->m_climber.runClimber(Constants.ClimberConstants.CLIMBER_SPEED_OUT));

        public Command climberStopCommand =new InstantCommand(()->m_climber.stopClimber());
        public Command armStopCommand = new InstantCommand(()->m_arm.stopArm());
        public Command rollerStopCommand = new InstantCommand(()->m_roller.stopRoller());


        public EightBitDo m_operatorController;
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer(OI driver_controller,
                        List<LoadablePathPlannerAuto> autos,
                        String robotName,
                        boolean isSim,
                        Alliance alliance,
                        PIDConstants tranPidPathPlanner,
                        PIDConstants rotPidPathPlanner) {

                super(driver_controller, autos, robotName, isSim, alliance, tranPidPathPlanner, rotPidPathPlanner);
                
                configureOperatorButtonBindings();

        }

        public void periodic() {
                s_Swerve.periodic();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        public void configureOperatorButtonBindings() {
                m_operatorController = new EightBitDo(Constants.Controllers.operator_controller_port);

                m_operatorController.getLeftBumper().whileTrue(armUpCommand).onFalse(armStopCommand);
                m_operatorController.getRightBumper().whileTrue(armDownCommand).onFalse(armStopCommand);

                m_operatorController.getAorCross().whileTrue(rollerInCommand).onFalse(rollerStopCommand);
                m_operatorController.getBorCircle().whileTrue(rollerOutCommand).onFalse(rollerStopCommand);

                m_operatorController.getXorSquare().whileTrue(climberWinchIn).onFalse(climberStopCommand);
                m_operatorController.getYorTriangle().whileTrue(climberWinchOut).onFalse(climberStopCommand);
                
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        @Override
        public Command getAutonomousCommand(String name) {
                // This method loads the auto when it is called, however, it is recommended
                // to first load your paths/autos when code starts, then return the
                // pre-loaded auto/path
                return super.getAutonomousCommand(name);
        }

        /**
         * Use this to pass the test command to the main {@link Robot} class.
         * Control pattern is forward, right , backwards, left, rotate in place
         * clockwise, rotate in place counterclowise, forward while rotating Clockwise,
         * forward while rotating counter clockwise
         *
         * @return the command to run in autonomous
         */
        @Override
        public Command getTestCommand() {
                return super.getTestCommand();
        }
}