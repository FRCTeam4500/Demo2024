package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DemoShooter;
import frc.robot.subsystems.DemoSwerve;

public class Robot extends TimedRobot {
    private DemoSwerve swerve;
    private DemoShooter shooter;
    
    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        swerve = new DemoSwerve();
        shooter = new DemoShooter();

        CommandXboxController drive = new CommandXboxController(2);
        CommandJoystick operator = new CommandJoystick(1);

        swerve.setDefaultCommand(
            swerve.angleCentricDrive(drive)
        );
        drive.a().onTrue(swerve.resetHeading());
        drive.rightTrigger().onTrue(shooter.shoot());

        operator.button(2).onTrue(shooter.startIntake());
        operator.button(2).onFalse(shooter.finishIntake());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
