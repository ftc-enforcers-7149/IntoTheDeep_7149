package org.firstinspires.ftc.teamcode.ActionUtils.ActionStructure;

//TODO: Create multiple ActionInfo classes to be used by EventActions
public class ActionInfo<T> {

    T infoObj;

    public ActionInfo(T info) {
        infoObj = info;
    }

    public T getInfo() {
        return infoObj;
    }

}
