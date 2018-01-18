package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

/**
 * Created by seb7 on 1/12/2018.
 */
/*
* This is a VuforiaLocalizer replacement, as it can do everything it does, as well as detach from the camera.
        *     To use, one can replace statements like:
        *
        *         VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        *
        *     with
        *
        *         ClosableVuforiaLocalizer vuforia = new ClosableVuforiaLocalizer(parameters);
        *
        *     To close vuforia, simply call vuforia.close();
        */

public class ClosableVuforiaLocalizer extends VuforiaLocalizerImpl {
    boolean closed = false;
    public ClosableVuforiaLocalizer(Parameters parameters) {
        super(parameters);
    }
    @Override
    public void close() {
        if (!closed) super.close();
        closed = true;
    }
}
