package com.github.pmtischler.vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

/**
 * Tests correctness of FtcColor utilities.
 */
public class FtcColorTest {
    private static final double diffThresh = 0.00001;

    @Before
    public void loadOpenCV() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public Mat loadImage(String filepath) throws Exception {
        return Imgcodecs.imread(
                "SharedCode/src/test/java/com/github/pmtischler/vision/testdata/" + filepath,
                Imgcodecs.CV_LOAD_IMAGE_COLOR);
    }

    public void writeImage(Mat img, String filepath) throws Exception {
        Imgcodecs.imwrite("SharedCode/src/test/java/com/github/pmtischler/vision/testoutput/" +
                filepath, img);
    }

    @Test
    public void testSimplifyImgSmoke() throws Exception{
        Mat img;

        img = loadImage("cryptobox_blue.jpg");
        FtcColor.simplifyImg(img);
        writeImage(img, "cryptobox_blue.jpg");

        img = loadImage("cryptobox_red.jpg");
        FtcColor.simplifyImg(img);
        writeImage(img, "cryptobox_red.jpg");
    }

    public void assertSimplifyHsv(int in_h, int in_s, int in_v,
                                  int out_h, int out_s, int out_v) {
        byte[] pixel = new byte[3];
        pixel[0] = (byte)in_h;
        pixel[1] = (byte)in_s;
        pixel[2] = (byte)in_v;

        FtcColor.simplifyHsv(pixel);

        assertEquals(out_h, pixel[0] & 0xFF);
        assertEquals(out_s, pixel[1] & 0xFF);
        assertEquals(out_v, pixel[2] & 0xFF);
    }

    @Test
    public void testSimplifyHsv() throws Exception {
        // Red stays red.
        assertSimplifyHsv(0, 255, 255,
                          0, 255, 255);
        assertSimplifyHsv(0, 200, 200,
                          0, 255, 255);
        // Blue stays blue.
        assertSimplifyHsv(120, 255, 255,
                          120, 255, 255);
        assertSimplifyHsv(120, 200, 200,
                          120, 255, 255);
        // Black becomes black.
        assertSimplifyHsv(0, 255, 0,
                          0, 255, 0);
        // White becomes black.
        assertSimplifyHsv(0, 0, 255,
                          0, 0, 0);
        // Green becomes black.
        assertSimplifyHsv(60, 255, 255,
                          60, 255, 0);
    }
}
