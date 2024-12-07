package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.hardware.NavX;
import frc.robot.subsystems.swerve.SwerveModule;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class DemoSwerve extends SubsystemBase {
    private NavX gyro;
    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private PIDController anglePID;
    private Rotation2d targetAngle;

    public DemoSwerve() {
        gyro = new NavX(Port.kMXP);
        modules = new SwerveModule[] {
            new SwerveModule(
				FRONT_LEFT_DRIVE_MOTOR,
				FRONT_LEFT_ANGLE_MOTOR,
				FRONT_LEFT_MODULE_CONFIG
			),
			new SwerveModule(
				FRONT_RIGHT_DRIVE_MOTOR,
				FRONT_RIGHT_ANGLE_MOTOR,
				FRONT_RIGHT_MODULE_CONFIG
			),
			new SwerveModule(
				BACK_LEFT_DRIVE_MOTOR,
				BACK_LEFT_ANGLE_MOTOR,
				BACK_LEFT_MODULE_CONFIG
			),
			new SwerveModule(
				BACK_RIGHT_DRIVE_MOTOR,
				BACK_RIGHT_ANGLE_MOTOR,
				BACK_RIGHT_MODULE_CONFIG
			)
        };
        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        targetAngle = gyro.getOffsetedAngle();
        anglePID = new PIDController(5, 0, 0);
		anglePID.enableContinuousInput(-Math.PI, Math.PI);
		anglePID.setTolerance(Math.PI / 32, Math.PI / 32);
		anglePID.setSetpoint(0);
    }

    public Command angleCentricDrive(CommandXboxController xbox) {
        return run(
            () -> {
                double delta = xbox.getRightX() * MAX_SPEEDS.omegaRadiansPerSecond;
                targetAngle = Rotation2d.fromRadians(targetAngle.getRadians() - delta);
                drive(
                    -xbox.getLeftY() *  MAX_SPEEDS.vxMetersPerSecond,
                    -xbox.getLeftX() * MAX_SPEEDS.vyMetersPerSecond
                );
            }
        ).beforeStarting(() -> targetAngle = gyro.getOffsetedAngle());
    }

    public Command resetHeading() {
        return Commands.runOnce(
            () -> {
				if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
					targetAngle = Rotation2d.fromDegrees(180);
				} else {
					targetAngle = new Rotation2d();
				}
                gyro.zeroWithOffset(targetAngle);
            }
        );
    }


    private void drive(
        double forwardVelocity,
        double sidewaysVelocity
    ) {
        Rotation2d addition = new Rotation2d();
		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			addition = Rotation2d.fromDegrees(180);
		}
        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardVelocity,
            sidewaysVelocity,
            calculateRotationalVelocityToTarget(targetAngle),
            gyro.getOffsetedAngle().plus(addition)

        );
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(
			ChassisSpeeds.discretize(targetChassisSpeeds, 0.02)
		);
		SwerveDriveKinematics.desaturateWheelSpeeds(
			states,
			MAX_LINEAR_SPEED_MPS
		);
		for (int i = 0; i < modules.length; i++) {
			modules[i].drive(states[i]);
		}
	}

    private double calculateRotationalVelocityToTarget(Rotation2d targetRotation) {
		double rotationalVelocity = anglePID.calculate(
			gyro.getOffsetedAngle().getRadians(),
			targetRotation.getRadians()
		);
		if (anglePID.atSetpoint()) {
			rotationalVelocity = 0;
		}
		return rotationalVelocity;
	}

	private Translation2d[] getModuleTranslations() {
		Translation2d[] translations = new Translation2d[modules.length];
		for (int i = 0; i < modules.length; i++) {
			translations[i] = modules[i].getTranslationFromCenter();
		}
		return translations;
	}
}
