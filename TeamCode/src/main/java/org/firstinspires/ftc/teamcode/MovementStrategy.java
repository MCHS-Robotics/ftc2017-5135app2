package org.firstinspires.ftc.teamcode;

/**
 * Created by student on 1/17/18.
 */

public interface MovementStrategy {

    public void forward(float amount);
    public void backward(float amount);

    public void pivotLeft(float amount);
    public void pivotRight(float amount);
}
