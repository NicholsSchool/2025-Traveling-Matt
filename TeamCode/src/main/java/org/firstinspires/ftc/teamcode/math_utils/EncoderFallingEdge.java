package org.firstinspires.ftc.teamcode.math_utils;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class EncoderFallingEdge
{
    IntSupplier position;
    IntSupplier velocity;

    public int numFullRotations = 0;

    public double prevPosition;
    public EncoderFallingEdge(IntSupplier position, IntSupplier velocity ) {
        this.position = position;
        this.velocity = velocity;
        prevPosition = 0.0;
    }

    public int calculatePosition()
    {
        if( velocity.getAsInt() > 0 && prevPosition > position.getAsInt() )
            numFullRotations++;

        if( velocity.getAsInt() < 0 && prevPosition < position.getAsInt() )
            numFullRotations--;

        prevPosition = position.getAsInt();
        assert numFullRotations >= 0;

        return 1024 * numFullRotations + position.getAsInt();
    }

    public void reset()
    {
        numFullRotations = 0;
    }
}
