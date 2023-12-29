package RoboRaiders.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class StevesPipeline extends OpenCvPipeline {

    private boolean findCountoursExternalOnly = false;

    private Mat hsvThresholdOutput = new Mat();
    private Mat hsvThresholdInput;
    private Mat findCountoursInput;

    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> filterContoursContours;
    private ArrayList<MatOfPoint> foundCountours;

    //HSV values for blue and red we want
    private double[] blueHSVThresholdHue = {56.38682136602451, 180.0};
    private double[] blueHSVThresholdSaturation = {99.609375, 255.0};
    private double[] blueHSVThresholdValue = {100.71614583333334, 255.0};

    private double[] redHSVThresholdHue = {0.0, 20.473209195046536};
    private double[] redHSVThresholdSaturation = {109.79460707021325, 255.0};
    private double[] redHSVThresholdValue = {0.0, 255.0};

    // Contour values/parameters we want - want a give size
    double filterContoursMinArea = 7000.0;
    double filterContoursMinPerimeter = 0.0;
    double filterContoursMinWidth = 0.0;
    double filterContoursMaxWidth = 1000.0;
    double filterContoursMinHeight = 1.0;
    double filterContoursMaxHeight = 1000.0;
    double[] filterContoursSolidity = {0, 100};
    double filterContoursMaxVertices = 1000000.0;
    double filterContoursMinVertices = 0.0;
    double filterContoursMinRatio = 0.0;
    double filterContoursMaxRatio = 1000.0;



    @Override
    public Mat processFrame(Mat input) {

        hsvThresholdInput = input;     // save the original image

        // find the blue objects in frame, to find red, just change up the hue saturation and value
        hsvThreshold(hsvThresholdInput,
                     blueHSVThresholdHue,
                     blueHSVThresholdSaturation,
                     blueHSVThresholdValue,
                     hsvThresholdOutput);

        // find the contours
        findCountoursInput = hsvThresholdOutput;
        findContours(findCountoursInput,
                      findCountoursExternalOnly,
                      findContoursOutput);

        filterContoursContours = findContoursOutput;
        filterContours(filterContoursContours,
                filterContoursMinArea,
                filterContoursMinPerimeter,
                filterContoursMinWidth,
                filterContoursMaxWidth,
                filterContoursMinHeight,
                filterContoursMaxHeight,
                filterContoursSolidity,
                filterContoursMaxVertices,
                filterContoursMinVertices,
                filterContoursMinRatio,
                filterContoursMaxRatio,
                filterContoursOutput);

        return input;
    }

    /**
     * Segment an image based on hue, saturation, and value ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue
     * @param sat The min and max saturation
     * @param val The min and max value
     * @param out The image in which to store the output.
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    /**
     * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
     * @param input The image on which to perform the Distance Transform.
     * @param externalOnly The Transform. used to be "type"
     * @param contours the size of the mask. used to be "maskSize"
     */

    private void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

    /**
     * Filters out contours that do not meet certain criteria.
     * @param inputContours is the input list of contours
     * @param output is the the output list of contours
     * @param minArea is the minimum area of a contour that will be kept
     * @param minPerimeter is the minimum perimeter of a contour that will be kept
     * @param minWidth minimum width of a contour
     * @param maxWidth maximum width
     * @param minHeight minimum height
     * @param maxHeight maximimum height
     * @param solidity the minimum and maximum solidity of a contour
     * @param minVertexCount minimum vertex Count of the contours
     * @param maxVertexCount maximum vertex Count
     * @param minRatio minimum ratio of width to height
     * @param maxRatio maximum ratio of width to height
     */
    private void filterContours(List<MatOfPoint> inputContours, double minArea,
                                double minPerimeter, double minWidth, double maxWidth, double minHeight, double
                                        maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
                                        minRatio, double maxRatio, List<MatOfPoint> output) {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        //operation
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if (bb.width < minWidth || bb.width > maxWidth) continue;
            if (bb.height < minHeight || bb.height > maxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int)hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < solidity[0] || solid > solidity[1]) continue;
            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
            final double ratio = bb.width / (double)bb.height;
            if (ratio < minRatio || ratio > maxRatio) continue;
            output.add(contour);
        }
    }

    // Return the contours that were found in filterCountours
    public ArrayList<MatOfPoint> getFoundContours() { return filterContoursOutput; }

}
