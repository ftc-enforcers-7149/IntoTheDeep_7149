package org.firstinspires.ftc.teamcode.PathingSystems.PurePursuit;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

public class PurePursuitController {


    public static double p2pDistance(NavPoint p1, NavPoint p2) {
        return Math.hypot(p2.getX() - p1.getX(), p2.getY() - p1.getY());
    }

    public static double p2pDistance(Pose2d p1, NavPoint p2) {
        return Math.hypot(p2.getX() - p1.position.x, p2.getY() - p1.position.y);
    }


    public static PurePursuitReturn findGoalPoint(
            ArrayList<NavPoint> path, Pose2d position,
            double lookAheadDist, int lastFoundIndex, Canvas canvas) {

        NavPoint goalPoint = null;

        double currentX = position.position.x;
        double currentY = position.position.y;

        int start = lastFoundIndex;

        for (int i = start; i < path.size() - 1; i++) {

            double x1 = path.get(i).getX();
            double y1 = path.get(i).getY();
            double x2 = path.get(i+1).getX();
            double y2 = path.get(i+1).getY();

            double x1Offset = x1 - currentX;
            double y1Offset = y1 - currentY;
            double x2Offset = x2 - currentX;
            double y2Offset = y2 - currentY;

            double dx = x2Offset - x1Offset;
            double dy = y2Offset - y1Offset;
            double dr = Math.hypot(dx, dy);

            double determinant = x1Offset * y2Offset - x2Offset * y1Offset;
            double discriminant = Math.pow(lookAheadDist, 2) * Math.pow(dr, 2) - Math.pow(determinant, 2);

            if (discriminant >= 0){

                double sol1x = ((determinant * dy) + Math.signum(dy) * dx * Math.sqrt(discriminant)) / Math.pow(dr, 2);
                double sol1y = ((-determinant * dy) + Math.abs(dy) * Math.sqrt(discriminant)) / Math.pow(dr, 2);
                double sol2x = ((determinant * dy) - Math.signum(dy) * dx * Math.sqrt(discriminant)) / Math.pow(dr, 2);
                double sol2y = ((-determinant * dy) - Math.abs(dy) * Math.sqrt(discriminant)) / Math.pow(dr, 2);

                NavPoint sol1 = new NavPoint(sol1x + currentX, sol1y + currentY);

                NavPoint sol2 = new NavPoint(sol2x + currentX, sol2y + currentY);

                canvas.setFill("orange");
                canvas.fillCircle(sol1.getX(), sol1.getY(), 2);
                canvas.fillCircle(sol2.getX(), sol2.getY(), 2);

                double minX = Math.min(x1, x2);
                double maxX = Math.max(x1, x2);

                boolean int1Found = (minX <= sol1.getX() && sol1.getX() <= maxX);
                boolean int2Found = (minX <= sol2.getX() && sol2.getX() <= maxX);

                if (int1Found && int2Found) {
                    if (p2pDistance(sol1, path.get(i+1)) < p2pDistance(sol2, path.get(i+1))){
                        goalPoint = sol1;
                    } else {
                        goalPoint = sol2;
                    }
                } else if(int1Found){
                    goalPoint = sol1;
                } else if(int2Found){
                    goalPoint = sol2;
                } else {
                    goalPoint = path.get(lastFoundIndex);
                }

                if ((lastFoundIndex == path.size() - 2) && (p2pDistance(goalPoint, path.get(i+1)) > p2pDistance(position, path.get(i+1)))) {
                    goalPoint = path.get(path.size() - 1);
                }

                if (p2pDistance(goalPoint, path.get(i+1)) < p2pDistance(position, path.get(i+1)) && goalPoint != path.get(i)) {
                    lastFoundIndex = i;
                    break;
                } else {
                    lastFoundIndex = i+1;
                }

            }

        }

        int pathSize = path.size();

        if (goalPoint == null){
            //make sure we do not get an index out of bounds error
            int clipLastIndex = Range.clip(lastFoundIndex, 0, pathSize - 1);
            return new PurePursuitReturn(path.get(clipLastIndex), lastFoundIndex);
        }

        int clipNextIndex = Range.clip(lastFoundIndex + 1, 0, pathSize - 1);

        return new PurePursuitReturn(goalPoint.setHeading(path.get(clipNextIndex).getHeading()), lastFoundIndex);
    }


}
