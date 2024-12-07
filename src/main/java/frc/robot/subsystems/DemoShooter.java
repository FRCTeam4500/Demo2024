package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.CANConstants.*;

public class DemoShooter extends SubsystemBase {
    private SparkMax tiltMotor;
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private SparkMax loaderMotor;

    public DemoShooter() {
        rightMotor = new SparkMax(SHOOTER_ONE_ID, MotorType.kBrushless);
        leftMotor = new SparkMax(SHOOTER_TWO_ID, MotorType.kBrushless);
        loaderMotor = new SparkMax(LOADER_ID, MotorType.kBrushless);

        tiltMotor = new SparkMax(SHOOTER_PIVOT_ID, MotorType.kBrushless);
        SparkMaxConfig tiltConfig = new SparkMaxConfig();
        tiltConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(30)
            .apply(new ClosedLoopConfig()
                .p(3)
                .outputRange(-0.5, 0.3)
            );
        tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        loaderMotor = new SparkMax(LOADER_ID, MotorType.kBrushless);
        SparkMaxConfig loaderConfig = new SparkMaxConfig();
        loaderConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        loaderMotor.configure(loaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightMotor = new SparkMax(SHOOTER_ONE_ID, MotorType.kBrushless);
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.smartCurrentLimit(30);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftMotor = new SparkMax(SHOOTER_TWO_ID, MotorType.kBrushless);
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.smartCurrentLimit(30);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command startIntake() {
        return Commands.runOnce(
            () -> {
                rightMotor.set(-0.5);
                leftMotor.set(-0.5);
                tiltMotor.set(0);
                loaderMotor.set(0);
            }
        );
    }

    public Command finishIntake() {
        return Commands.runOnce(
            () -> {
                rightMotor.set(-0.15);
                leftMotor.set(-0.15);
                loaderMotor.set(0);
                tiltMotor.getClosedLoopController().setReference(-7, ControlType.kPosition);
            }
        ).andThen(
            Commands.waitSeconds(0.15)
        ).andThen(
            Commands.runOnce(
                () -> {
                    loaderMotor.set(0.25);
                }    
            )
        ).andThen(
            Commands.waitSeconds(0.11)
        ).andThen(
            () -> {
                rightMotor.set(0);
                leftMotor.set(0);
                loaderMotor.set(0);
                tiltMotor.getClosedLoopController().setReference(-1, ControlType.kPosition);
            }    
        ).andThen(
            Commands.waitSeconds(0.15)
        ).andThen(
            Commands.runOnce(() -> tiltMotor.set(0))
        );

    }

    public Command shoot() {
        return Commands.runOnce(() -> {
            rightMotor.set(0.8);
            leftMotor.set(0.8);
            tiltMotor.set(0);
            loaderMotor.set(0);
        }).andThen(
            Commands.waitSeconds(1)
        ).andThen(
            Commands.runOnce(() -> loaderMotor.set(-1))
        ).andThen(
            Commands.waitSeconds(1)
        ).andThen(
            Commands.runOnce(() -> {
                leftMotor.set(0);
                rightMotor.set(0);
                loaderMotor.set(0);
            })
        );
    }
}
