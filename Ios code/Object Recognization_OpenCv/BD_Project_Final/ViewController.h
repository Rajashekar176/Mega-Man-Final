//
//  ViewController.h
//  BD_Project_Final
//
//  Created by Vamshi Reddy on 7/25/15.
//  Copyright (c) 2015 UMKC. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
#import <CoreMotion/CoreMotion.h>
#include <stdlib.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include <vector>

@interface ViewController : UIViewController<AVCaptureVideoDataOutputSampleBufferDelegate>{
    
    AVCaptureSession *_session;
    AVCaptureDevice *_captureDevice;
    BOOL _useBackCamera;
    
}

- (void)setupCamera;
- (void)turnCameraOn;
- (void)turnCameraOff;
- (void)didCaptureIplImage:(IplImage *)iplImage;
- (void)didFinishProcessingImage:(IplImage *)iplImage;
- (void)getUIImageFromIplImage:(IplImage *)iplImage;
@end

