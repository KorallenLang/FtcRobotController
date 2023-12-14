package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

public class DrawRectangleProcessor implements VisionProcessor {
    public Rect rect = new Rect(20, 20, 50, 50);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), rectPaint);
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvas) {
        int left = Math.round(rect.x * scaleBmpPxToCanvas);
        int top = Math.round(rect.y * scaleBmpPxToCanvas);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvas);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvas);

        return new android.graphics.Rect(left, top, right, bottom);
    }
}
