package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

public class InstantAction extends EventAction{

    public interface InstantLambda {
        public void run();
    }

    private InstantLambda instantAction;


    public InstantAction(InstantLambda instantAction) {
        this.instantAction = instantAction;
    }

    @Override
    public boolean run(CombinedTelemetry t) {

        instantAction.run();

        return false;
    }

    @Override
    public void init() {

    }

    @Override
    public void stop(boolean interrupted) {

    }
}
