//
//  ViewController.m
//  BD_Project_Final
//
//  Created by Vamshi Reddy on 7/25/15.
//  Copyright (c) 2015 UMKC. All rights reserved.
//

#import "ViewController.h"


@interface ViewController ()

@end

using namespace std;
using namespace cv;
IplImage *imageipl = 0;

double compareSURFDescriptors( const float* d1, const float* d2, double best, int length )
{
    double total_cost = 0;
    assert( length % 4 == 0 );
    for( int i = 0; i < length; i += 4 )
    {
        double t0 = d1[i] - d2[i];
        double t1 = d1[i+1] - d2[i+1];
        double t2 = d1[i+2] - d2[i+2];
        double t3 = d1[i+3] - d2[i+3];
        total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
        if( total_cost > best )
            break;
    }
    return total_cost;
}
int naiveNearestNeighbor( const float* vec, int laplacian,
                         const CvSeq* model_keypoints,
                         const CvSeq* model_descriptors )
{
    int length = (int)(model_descriptors->elem_size/sizeof(float));
    int i, neighbor = -1;
    double d, dist1 = 1e6, dist2 = 1e6;
    CvSeqReader reader, kreader;
    cvStartReadSeq( model_keypoints, &kreader, 0 );
    cvStartReadSeq( model_descriptors, &reader, 0 );
    
    for( i = 0; i < model_descriptors->total; i++ )
    {
        const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* mvec = (const float*)reader.ptr;
        CV_NEXT_SEQ_ELEM( kreader.seq->elem_size,kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size,reader );
        if( laplacian != kp->laplacian )
            continue;
        d = compareSURFDescriptors( vec, mvec, dist2, length );
        if( d < dist1 )
        {
            dist2 = dist1;
            dist1 = d;
            neighbor = i;
        }
        else if ( d < dist2 )
            dist2 = d;
    }
    if ( dist1 < 0.6*dist2 )
        return neighbor;
    return -1;
}

void findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
               const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
    int i;
    CvSeqReader reader, kreader;
    cvStartReadSeq( objectKeypoints, &kreader );
    cvStartReadSeq( objectDescriptors, &reader );
    ptpairs.clear();
    
    for( i = 0; i < objectDescriptors->total; i++ )
    {
        const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* descriptor = (const float*)reader.ptr;
        CV_NEXT_SEQ_ELEM( kreader.seq->elem_size,kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size,reader );
        int nearest_neighbor = naiveNearestNeighbor( descriptor, kp->laplacian, imageKeypoints, imageDescriptors );
        if( nearest_neighbor >= 0 )
        {
            ptpairs.push_back(i);
            ptpairs.push_back(nearest_neighbor);
        }
    }
}

/* a rough implementation for object location */
int locatePlanarObject( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
                          const CvSeq* imageKeypoints, const CvSeq* imageDescriptors,
                          const CvPoint src_corners[4], CvPoint dst_corners[4] )
{
    double h[9];
    CvMat _h = cvMat(3, 3, CV_64F, h);
    vector<int> ptpairs;
    vector<CvPoint2D32f> pt1, pt2;
    CvMat _pt1, _pt2;
    int i, n;
    findPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
    n = (int)(ptpairs.size()/2);
    if( n < 4 )
        return 0;
    
    pt1.resize(n);
    pt2.resize(n);
    for( i = 0; i < n; i++ )
    {
        pt1[i] = ((CvSURFPoint*)cvGetSeqElem(objectKeypoints,ptpairs[i*2]))->pt;
        pt2[i] = ((CvSURFPoint*)cvGetSeqElem(imageKeypoints,ptpairs[i*2+1]))->pt;
    }
    
    _pt1 = cvMat(1, n, CV_32FC2, &pt1[0] );
    _pt2 = cvMat(1, n, CV_32FC2, &pt2[0] );
    if( !cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, 5 ))
        return 0;
    
    for( i = 0; i < 4; i++ )
    {
        double x = src_corners[i].x, y = src_corners[i].y;
        double Z = 1./(h[6]*x + h[7]*y + h[8]);
        double X = (h[0]*x + h[1]*y + h[2])*Z;
        double Y = (h[3]*x + h[4]*y + h[5])*Z;
        dst_corners[i] = cvPoint(cvRound(X), cvRound(Y));
    }
    
    return 1;
}
@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}



- (void)captureOutput:(AVCaptureFileOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection
{
    IplImage *imageipl = 0;
    
    //int main(int argc, char** argv)
    //{
        const char* object_filename = "image.bmp"; // input image to be detected
    
        int key;
    
        //int sy;
        //sy=system("stty -F /dev/ttyUSB0 cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoe -noflsh -ixon");
        CvMemStorage* storage = cvCreateMemStorage(0);
        //help();
        // cvNamedWindow("Object", 1);
        cvNamedWindow("Object Correspond", 1);
        
        static CvScalar colors[] =
        {
            {{0,0,255}},
            {{0,128,255}},
            {{0,255,255}},
            {{0,255,0}},
            {{255,128,0}},
            {{255,255,0}},
            {{255,0,0}},
            {{255,0,255}},
            {{255,255,255}}
        };
        
        CvCapture* capture = cvCreateCameraCapture(3);
        CvMat* prevgray = 0, *image = 0, *gray =0;
        while( key != 'q' )
        {
            int firstFrame = gray == 0;
            IplImage* frame = cvQueryFrame(capture);
            if(!frame)
                break;
            if(!gray)
            {
                image = cvCreateMat(frame->height, frame->width, CV_8UC1);
            }
            cvCvtColor(frame, image, CV_BGR2GRAY);
            CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
            int i;
            
            //Extract SURF points by initializing parameters
            CvSURFParams params = cvSURFParams(500, 1);
            cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params );
            IplImage* object = cvLoadImage( object_filename, CV_LOAD_IMAGE_GRAYSCALE );
            
            IplImage* object_color = cvCreateImage(cvGetSize(object), 8, 3);
            cvCvtColor( object, object_color, CV_GRAY2BGR );
            
            CvSeq *objectKeypoints = 0, *objectDescriptors = 0;
            cvExtractSURF( object, 0, &objectKeypoints, &objectDescriptors, storage, params );
            printf("Object Descriptors: %d\n", objectDescriptors->total);
            printf("Image Descriptors: %d\n", imageDescriptors->total);
            CvPoint src_corners[4] = {{0,0}, {object->width,0}, {object->width, object->height}, {0, object->height}};
            CvPoint dst_corners[4];
            IplImage* correspond = cvCreateImage( cvSize(image->width, object->height+image->height), 8, 1 );
            cvSetImageROI( correspond, cvRect( 0, 0, object->width, object->height ) );
            cvCopy( object, correspond );
            cvSetImageROI( correspond, cvRect( 0, object->height, correspond->width, correspond->height ) );
            cvCopy( image, correspond );
            cvResetImageROI( correspond );
            
            if( locatePlanarObject( objectKeypoints, objectDescriptors, imageKeypoints,
                                   imageDescriptors, src_corners, dst_corners ))
            {
                printf("object found\n");
                for( i = 0; i < 1; i++ )
                {
                    CvPoint r1 = dst_corners[i%4];
                    CvPoint r2 = dst_corners[(i+1)%4];
                    cvLine( correspond, cvPoint(r1.x, r1.y+object->height ),
                           cvPoint(r2.x, r2.y+object->height ), colors[8] );
                    printf("%d,%d\n", r1.x, r1.y);
                    
                    if(r1.x<290)
                    {
                        printf("MOVE RIGHT\n");
                        //sy=system("echo -n '3' > /dev/ttyUSB0");
                    }
                    if(r1.x>340)
                    {
                        printf("MOVE LEFT\n");
                        //sy=system("echo -n '2' > /dev/ttyUSB0");
                    }
                    if((r1.x>290)&&(r1.x<340))
                    {
                        printf("MOVE FORWARD\n");
                        //sy=system("echo -n '1' > /dev/ttyUSB0");
                        
                    }
                }
            }
            else
            {
                printf("searching.....\n");
                //sy=system("echo -n '7' > /dev/ttyUSB0");
                printf("searching..nnnnggggg...\n");
            }
            vector<int> ptpairs;
            findPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
            for( i = 0; i < (int)ptpairs.size(); i += 2 )
            {
                CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( objectKeypoints, ptpairs[i] );
                CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, ptpairs[i+1] );
                cvLine( correspond, cvPointFrom32f(r1->pt),
                       cvPoint(cvRound(r2->pt.x), cvRound(r2->pt.y+object->height)), colors[8] );
            }
            
            cvShowImage( "Object Correspond", correspond );
            for( i = 0; i < objectKeypoints->total; i++ )
            {
                CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( objectKeypoints, i );
                CvPoint center;
                int radius;
                center.x = cvRound(r->pt.x);
                center.y = cvRound(r->pt.y);
                radius = cvRound(r->size*1.2/9.*2);
                cvCircle( object_color, center, radius, colors[0], 1, 8, 0 );
            }
            cvWaitKey(30);
        }
    
    [self didCaptureIplImage:workingCopy];
}


- (void)didFinishProcessingImage:(IplImage *)iplImage
{
    dispatch_async(dispatch_get_main_queue(), ^{
        UIImage *uiImage = [self getUIImageFromIplImage:iplImage];
        _imageView.image = uiImage;
    });
}


- (UIImage*)getUIImageFromIplImage:(IplImage*)iplImage
{
    CGImageRef cgImage = [self getCGImageFromIplImage:iplImage];
    UIImage *uiImage = [[UIImage alloc] initWithCGImage:cgImage
                                                  scale:1.0
                                            orientation:UIImageOrientationUp];
    
    CGImageRelease(cgImage);
    return uiImage;
}



- (void)didCaptureIplImage:(IplImage *)iplImage
{
    //ipl image is in BGR format, it needs to be converted to RGB for display in UIImageView
    IplImage *imgRGB = cvCreateImage(cvGetSize(iplImage), IPL_DEPTH_8U, 3);
    cvCvtColor(iplImage, imgRGB, CV_BGR2RGB);
    Mat matRGB = Mat(imgRGB);
    
    //ipl imaeg is also converted to HSV; hue is used to find certain color
    IplImage *imgHSV = cvCreateImage(cvGetSize(iplImage), 8, 3);
    cvCvtColor(iplImage, imgHSV, CV_BGR2HSV);
    
    IplImage *imgThreshed = cvCreateImage(cvGetSize(iplImage), 8, 1);
    IplImage *yelowImgThreshed = cvCreateImage(cvGetSize(iplImage), 8, 1);
    IplImage *greenThreshed = cvCreateImage(cvGetSize(iplImage), 8, 1);
    
    //it is important to release all images EXCEPT the one that is going to be passed to
    //the didFinishProcessingImage: method and displayed in the UIImageView
    cvReleaseImage(&iplImage);
    
    //filter all pixels in defined range, everything in range will be white, everything else
    //is going to be black
    cvInRangeS(imgHSV, cvScalar(160, 100, 100), cvScalar(179, 255, 255), imgThreshed);//red
    cvInRangeS(imgHSV, cvScalar(22, 100, 100), cvScalar(38, 255, 255), yelowImgThreshed);//yellow
    cvInRangeS(imgHSV, cvScalar(38,100,100), cvScalar(75,255,255), greenThreshed);//green
    
    cvReleaseImage(&imgHSV);
    
    Mat matThreshed = Mat(imgThreshed);
    Mat yelomatThreshed = Mat(yelowImgThreshed);
    Mat greenmatThreshed = Mat(greenThreshed);
    
    //smooths edges
    cv::GaussianBlur(matThreshed,
                     matThreshed,
                     cv::Size(9, 9),
                     2,
                     2);
    
    cv::GaussianBlur(yelomatThreshed,
                     yelomatThreshed,
                     cv::Size(9, 9),
                     2,
                     2);
    
    cv::GaussianBlur(greenmatThreshed,
                     greenmatThreshed,
                     cv::Size(9, 9),
                     2,
                     2);
    
    //debug shows threshold image, otherwise the circles are detected in the
    //threshold image and shown in the RGB image
    if (_debug)
    {
        cvReleaseImage(&imgRGB);
        [self didFinishProcessingImage:imgThreshed];
        [self didFinishProcessingImage:yelowImgThreshed];
        [self didFinishProcessingImage:greenThreshed];
    }
    else
    {
        vector<Vec3f> circles;
        vector<Vec3f> greenCircles;
        vector<Vec3f> yeloCircles;
        
        //get circles
        HoughCircles(matThreshed,
                     circles,
                     CV_HOUGH_GRADIENT,
                     2,
                     matThreshed.rows / 4,
                     150,
                     75,
                     10,
                     150);
        //get circles
        HoughCircles(yelomatThreshed,
                     yeloCircles,
                     CV_HOUGH_GRADIENT,
                     2,
                     yelomatThreshed.rows / 4,
                     150,
                     75,
                     10,
                     150);
        //get circles
        HoughCircles(greenmatThreshed,
                     greenCircles,
                     CV_HOUGH_GRADIENT,
                     2,
                     greenmatThreshed.rows / 4,
                     150,
                     75,
                     10,
                     150);
        
        for (size_t i = 0; i < circles.size(); i++)
        {
            cout << "Red Circle position x = " << (int)circles[i][0] << ", y = " << (int)circles[i][1] << ", radius = " << (int)circles[i][2] << "\n";
            
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            
            int radius = cvRound(circles[i][2]);
            
            circle(matRGB, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            circle(matRGB, center, radius, Scalar(0, 0, 255), 3, 8, 0);
        }
        
        for (size_t i = 0; i < greenCircles.size(); i++)
        {
            cout << "Green Circle position x = " << (int)greenCircles[i][0] << ", y = " << (int)greenCircles[i][1] << ", radius = " << (int)greenCircles[i][2] << "\n";
            
            cv::Point center(cvRound(greenCircles[i][0]), cvRound(greenCircles[i][1]));
            
            int radius = cvRound(greenCircles[i][2]);
            
            circle(matRGB, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            circle(matRGB, center, radius, Scalar(0, 0, 255), 3, 8, 0);
        }
        
        for (size_t i = 0; i < yeloCircles.size(); i++)
        {
            cout << "Yellow Circle position x = " << (int)yeloCircles[i][0] << ", y = " << (int)yeloCircles[i][1] << ", radius = " << (int)yeloCircles[i][2] << "\n";
            
            cv::Point center(cvRound(yeloCircles[i][0]), cvRound(yeloCircles[i][1]));
            
            int radius = cvRound(yeloCircles[i][2]);
            
            circle(matRGB, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            circle(matRGB, center, radius, Scalar(0, 0, 255), 3, 8, 0);
        }
        BOOL yellow = NO;
        BOOL green = NO;
        BOOL red = NO;
        if (circles.size() > greenCircles.size()) {
            if (circles.size()<yeloCircles.size()) {
                //slow down & turn right;
                yellow = YES;
                //                dispatch_async(dispatch_get_main_queue(), ^{
                //                    [[[UIAlertView alloc] initWithTitle:@"YELLOW" message:@"Turn Right" delegate:nil cancelButtonTitle:nil otherButtonTitles:@"Ok", nil] show];
                //                });
                [self.Romo3 driveForwardWithSpeed:0.15];
                [self.Romo3 turnByAngle:90 withRadius:0.25 completion:^(BOOL success, float heading) {
                    //
                }];
                [self stopUpdates];
                //[super turnCameraOff];
                
            }else{
                red = YES;
                [self.Romo3 stopDriving];
                [self stopUpdates];
                //[super turnCameraOff];
                //                dispatch_async(dispatch_get_main_queue(), ^{
                //                    [[[UIAlertView alloc] initWithTitle:@"Red" message:@"Stop" delegate:nil cancelButtonTitle:nil otherButtonTitles:@"Ok", nil] show];
                //                });
                
            }
        }else{
            
            if (greenCircles.size()<yeloCircles.size()) {
                yellow = YES;
                [self.Romo3 driveForwardWithSpeed:0.15];
                [self.Romo3 turnByAngle:90 withRadius:0.25 completion:^(BOOL success, float heading) {
                    //
                }];
                [self stopUpdates];
                //[super turnCameraOff];
                //                dispatch_async(dispatch_get_main_queue(), ^{
                //                    [[[UIAlertView alloc] initWithTitle:@"YELLOW" message:@"Turn Right" delegate:nil cancelButtonTitle:nil otherButtonTitles:@"Ok", nil] show];
                //                });
                
            }else{
                green = YES;
                [self.Romo3 driveForwardWithSpeed:0.55];
                [self stopUpdates];
                //[super turnCameraOff];
                //                dispatch_async(dispatch_get_main_queue(), ^{
                //                    [[[UIAlertView alloc] initWithTitle:@"GREEN" message:@"Do not stop" delegate:nil cancelButtonTitle:nil otherButtonTitles:@"Ok", nil] show];
                //                });
                
            }
        }
        
        //threshed image is not needed any more and needs to be released
        cvReleaseImage(&imgThreshed);
        cvReleaseImage(&yelowImgThreshed);
        cvReleaseImage(&greenThreshed);
        
        //imgRGB will be released once it is not needed, the didFinishProcessingImage:
        //method will take care of that
        [self didFinishProcessingImage:imgRGB];
    }
}






@end
