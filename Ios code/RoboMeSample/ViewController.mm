//
//  ViewController.m
//  RoboMeBasicSample
//
//  Copyright (c) 2015 Vamshi Reddy. All rights reserved.
//

#import "ViewController.h"
#import "GCDAsyncSocket.h"
#include <ifaddrs.h>
#include <arpa/inet.h>
#import "AFHTTPRequestOperationManager.h"
#import "AFHTTPRequestOperationManager.h"


#import <AVFoundation/AVFoundation.h>

#define TTP @"http://tts-api.com/tts.mp3?"


//opencv framework
#import <opencv2/objdetect/objdetect.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#import "opencv2/opencv.hpp"
#import "opencv2/nonfree/nonfree.hpp"
#import <opencv2/highgui/cap_ios.h>
#import <math.h>


using namespace std;
using namespace cv;

static const NSTimeInterval accelerometerMin = 0.1;

#define WELCOME_MSG  0
#define ECHO_MSG     1
#define WARNING_MSG  2

#define READ_TIMEOUT 15.0
#define READ_TIMEOUT_EXTENSION 10.0

#define FORMAT(format, ...) [NSString stringWithFormat:(format), ##__VA_ARGS__]
#define PORT 1234


@interface ViewController () {
    dispatch_queue_t socketQueue;
    NSMutableArray *connectedSockets;
    BOOL isRunning;
    
    GCDAsyncSocket *listenSocket;
}

@property (nonatomic,strong) AVPlayer *player;
@property (nonatomic, strong) RoboMe *roboMe;
@property(nonatomic, strong) CommandPlayer *commandPlayer;
@property (strong, nonatomic) AVAudioPlayer *audioPlayer;



@end
float gravity[] = {0,0,0};
float xPoint=0;
float yPoint=0;
float zPoint=0;
float xPoint1;
float yPoint1;
float zPoint1;
Boolean slope_traversal;


double a;
double b;
double c;

double red_min = 160;
double red_max = 179;


@implementation ViewController

@synthesize player = _player;
@synthesize playButton=_playButton;
@synthesize pickerView=_pickerView;

#pragma mark - View Management
double speed=0.3;
CMMotionManager *mManager ;

- (void)viewDidLoad {
    [super viewDidLoad];
    
    // create RoboMe object
    self.roboMe = [[RoboMe alloc] initWithDelegate: self];
    
    self.commandPlayer = [[CommandPlayer alloc]init];
    
    //[self addGestureRecognizers];
    
    // start listening for events from RoboMe
    [self.roboMe startListening];
    
    // Do any additional setup after loading the view, typically from a nib.
    socketQueue = dispatch_queue_create("socketQueue", NULL);
    
    listenSocket = [[GCDAsyncSocket alloc] initWithDelegate:self delegateQueue:socketQueue];
    
    // Setup an array to store all accepted client connections
    connectedSockets = [[NSMutableArray alloc] initWithCapacity:1];
    
    isRunning = NO;
    
    
    
    NSLog(@"Ip:%@", [self getIPAddress]);
    
     //[self greetPeople];
    
    [self toggleSocketState];
    
    //Starting the Socket
    
  
   
    [self perform:@"alert"];
    
    //[self flipAction];
    
}

//- (void)viewWillAppear:(BOOL)animated
//{
//    // Add Romo's face to self.view whenever the view will appear
//    [self.Romo addToSuperview:self.view];
//    //[self setupCamera];
//    //[self turnCameraOn];
//}

-(void) greetPeople{
    [self.commandPlayer playCommand:@"GooDAfternoon.mp3"];
    [self.commandPlayer playCommand:@"MynameisRoboC.mp3"];
}


#pragma mark - RoboMeConnectionDelegate

// Event commands received from RoboMe
- (void)commandReceived:(IncomingRobotCommand)command {
    // Display incoming robot command in text view
    //[self displayText: [NSString stringWithFormat: @"Received: %@" ,[RoboMeCommandHelper incomingRobotCommandToString: command]]];
    
    // To check the type of command from RoboMe is a sensor status use the RoboMeCommandHelper class
    if([RoboMeCommandHelper isSensorStatus: command]){
        // Read the sensor status
      //  SensorStatus *sensors = [RoboMeCommandHelper readSensorStatus: command];
        
        // Update labels
//        [self.edgeLabel setText: (sensors.edge ? @"ON" : @"OFF")];
//        [self.chest20cmLabel setText: (sensors.chest_20cm ? @"ON" : @"OFF")];
//        [self.chest50cmLabel setText: (sensors.chest_50cm ? @"ON" : @"OFF")];
//        [self.cheat100cmLabel setText: (sensors.chest_100cm ? @"ON" : @"OFF")];
    }
}

#pragma mark -
#pragma mark User-Defined Robo Movement

- (NSString *)direction:(NSString *)message {
    
    return @"";
}

- (void)perform:(NSString *)command {
    
    NSString *cmd = [command uppercaseString];
    if ([cmd isEqualToString:@"HELLO MEGA MAN"]) {
        NSLog(@"Inside hello block");
        [self play:@"hello how are you"];
    }
    
    if ([cmd isEqualToString:@"LEFT"]) {
        [self.roboMe sendCommand:kRobot_TurnLeft90Degrees];
        
        [self play:@"I am moving left"];
    } else if ([cmd isEqualToString:@"RIGHT"]) {
        [self.roboMe sendCommand: kRobot_TurnRight90Degrees];
        [self play:@"I am moving right"];
    } else if ([cmd isEqualToString:@"DOWN"]) {
        [self.roboMe sendCommand: kRobot_MoveBackwardFastest];
        [self play:@"I am moving backward"];
    } else if ([cmd isEqualToString:@"UP"]) {
        [self.roboMe sendCommand: kRobot_MoveForwardFastest];
        [self play:@"I am moving forward"];
    } else if([cmd isEqualToString:@"STOP"]){
        [self.roboMe sendCommand:kRobot_Stop];
        [self play:@"I am stoping to move forward"];
    }
//    else if([cmd isEqualToString:@"UP"]){
//        NSLog(@"Head tilt up ");
//        [self.roboMe sendCommand:kRobot_HeadTiltUp];
//        [self play:@"I am tilting head up"];
//    }
//    else if ([cmd isEqualToString:@"DOWN"]){
//        NSLog(@"Head tilt down");
//        [self.roboMe sendCommand:kRobot_HeadTiltDown];
//        [self play:@"I am tilting head down"];
//    }
    else if ([cmd isEqualToString:@"DO YOU KNOW ME"]){
        NSLog(@"Hello block");
        [self.roboMe sendCommand:kRobot_HeadTiltDown];
        [self play:@"Yes. you are doctor James"];
    }
    else if ([cmd isEqualToString:@"BYE"]){
        NSLog(@"Inside the bye block");
        //[self.roboMe sendCommand:kRobot_HeadTiltDown];
        [self play:@"Bye. Have a good day"];
    }
    else if ([cmd isEqualToString:@"GOOD MORNING"]){
        NSLog(@"Inside morning block");
        [self play:@"good morning have a nice day"];
    }
    else if ([cmd isEqualToString:@"GOOD AFTERNOON"]){
        NSLog(@"Inside AFTERNOON block");
        [self play:@"good afternoon have a nice day"];
    }
    else if ([cmd isEqualToString:@"GOOD EVENING"]){
        NSLog(@"Inside  block");
        [self play:@"good evening how was the day"];
    }
    else if ([cmd isEqualToString:@"GOOD NIGHT"]){
        NSLog(@"Inside morning block");
        [self play:@"good night. Sweet dreams"];
    }
    else if ([cmd containsString:@"WHAT IS WEATHER IN"]){
        NSString *locationString =cmd;
        
        NSRange range = [locationString rangeOfString:@" "  options:NSBackwardsSearch];
        
        NSString *result = [locationString substringFromIndex:range.location+1];
        
        NSString *locationName = result;
        NSString * countryName  = @"us";
        
        
        NSString *urlString=[NSString stringWithFormat:@"http://api.openweathermap.org/data/2.5/weather?q=%@,%@",locationName,countryName];
        
        NSLog(@"URL String%@", urlString);
        
        AFHTTPRequestOperationManager *manager = [AFHTTPRequestOperationManager manager];
        
        [manager GET:urlString parameters:nil success:^(AFHTTPRequestOperation *operation, id responseObject) {
            
            NSDictionary *dict = [responseObject objectForKey:@"main"];
            
            NSString *temperature= [dict objectForKey:@"temp"];
            
            NSString *grid_level=[dict objectForKey:@"grid_level"];
            
            NSString *humidiity=[dict objectForKey:@"humidity"];
            
            NSString *pressure=[dict objectForKey:@"pressure"];
            NSString *sealevel=[dict objectForKey:@"sea_level"];
            NSString *temp_min=[dict objectForKey:@"temp_min"];
            NSString *temp_max=[dict objectForKey:@"temp_max"];
            
            NSArray *array=[responseObject objectForKey:@"weather"];
            NSDictionary *dict2=[array objectAtIndex:0];
            NSString *description=[dict2 objectForKey:@"description"];
            NSString *main = [dict2 objectForKey:@"main"];
            
            NSString *outputMain = [main uppercaseString];
            if([outputMain isEqualToString:@"CLOUDY"]){
                [self play:@"It is cloudy so it may rain."];
            }
            else if ([outputMain isEqualToString:@"RAIN"]){
                [self play:@"It is raining"];
            }
            else if ([outputMain isEqualToString:@"CLEAR"]){
                NSString *body = [NSString stringWithFormat:@"Sky is clear. Highest temperature is %@ and lowest temperature is %@.",temp_max,temp_min];
                [self play:body];
            }
        }failure:^(AFHTTPRequestOperation *operation, NSError *error) {
            
            NSLog(@"Error: %@",error);
            
            UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"Error"
                                                            message:[error description]
                                                           delegate:nil
                                                  cancelButtonTitle:@"Ok"
                                                  otherButtonTitles:nil, nil];
            [alert show];
        }];
        

        
        
    }
    
        else if ([cmd containsString:@"I AM GOING TO "]){
            NSString *locationString =cmd;
            
            NSRange range = [locationString rangeOfString:@" "  options:NSBackwardsSearch];
            
            NSString *result = [locationString substringFromIndex:range.location+1];
            
            NSString *locationName = result;
            NSString * countryName  = @"us";
            
            
            NSString *urlString=[NSString stringWithFormat:@"http://api.openweathermap.org/data/2.5/weather?q=%@,%@",locationName,countryName];
            
            NSLog(@"URL String%@", urlString);
            
            AFHTTPRequestOperationManager *manager = [AFHTTPRequestOperationManager manager];
            
            [manager GET:urlString parameters:nil success:^(AFHTTPRequestOperation *operation, id responseObject) {
                
                NSDictionary *dict = [responseObject objectForKey:@"main"];
                
                NSString *temperature= [dict objectForKey:@"temp"];
                
                NSString *grid_level=[dict objectForKey:@"grid_level"];
                
                NSString *humidiity=[dict objectForKey:@"humidity"];
                
                NSString *pressure=[dict objectForKey:@"pressure"];
                NSString *sealevel=[dict objectForKey:@"sea_level"];
                NSString *temp_min=[dict objectForKey:@"temp_min"];
                NSString *temp_max=[dict objectForKey:@"temp_max"];
                
                NSArray *array=[responseObject objectForKey:@"weather"];
                NSDictionary *dict2=[array objectAtIndex:0];
                NSString *description=[dict2 objectForKey:@"description"];
                NSString *main = [dict2 objectForKey:@"main"];
                
                NSString *outputMain = [main uppercaseString];
                if([outputMain isEqualToString:@"CLOUDY"]){
                    [self play:@"It is cloudy so it may rain. Carry umberilla. take care bye"];
                }
                else if ([outputMain isEqualToString:@"RAIN"]){
                    [self play:@"It is raining over there so try to avoid to go there"];
                }
                else if ([outputMain isEqualToString:@"CLEAR"]){
                    NSString *body = [NSString stringWithFormat:@"Sky is clear so you can go over there. Highest temperature is %@ and lowest temperature is %@. Bye take care",temp_max,temp_min];
                    [self play:body];
                }
            }failure:^(AFHTTPRequestOperation *operation, NSError *error) {
                
                NSLog(@"Error: %@",error);
                
                UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"Error"
                                                                message:[error description]
                                                               delegate:nil
                                                      cancelButtonTitle:@"Ok"
                                                      otherButtonTitles:nil, nil];
                [alert show];
            }];
        

    }
    else if([cmd isEqualToString:@"SING"]){
        //[self.commandPlayer playCommand:@"robo.mp3"];
        
        
        
        NSLog(@"Inside the sing ");
        
        [self play:@"Playing song"];
        
        [self.commandPlayer playCommand:@"song.mp3"];
    }
    else if([cmd isEqualToString:@"WHAT IS THE TIME NOW"]){
        NSDate *now = [NSDate date];
        
        NSDateFormatter *formatter = [[NSDateFormatter alloc]init];
        
        [formatter setDateFormat:@"HH:mm:ss"];
        
        NSString *newDateString = [formatter stringFromDate:now];
        
        NSLog(@"Current time is %@:",newDateString);
        
        NSString *message = [NSString stringWithFormat:@"Current time is %@",newDateString];
        [self play:message];
    }
    else if([cmd isEqualToString:@"HI"]){
        //[self.commandPlayer playCommand:@"robo.mp3"];
        
        
        
        NSLog(@"Inside the Hi ");
        
        [self play:@"Hi. How are you doing"];
        
        //[self.commandPlayer playCommand:@"robo.mp3"];
    }
    
    else if ([cmd isEqualToString:@"I AM GOING OUT"]){
        NSLog(@"Inside the going out method");
        
        NSString *locationName = @"Kansas";
        NSString * countryName  = @"us";
        
        
        NSString *urlString=[NSString stringWithFormat:@"http://api.openweathermap.org/data/2.5/weather?q=%@,%@",locationName,countryName];
        
        NSLog(@"URL String%@", urlString);
        
        AFHTTPRequestOperationManager *manager = [AFHTTPRequestOperationManager manager];
        
        [manager GET:urlString parameters:nil success:^(AFHTTPRequestOperation *operation, id responseObject) {
            
            NSDictionary *dict = [responseObject objectForKey:@"main"];
            
            NSString *temperature= [dict objectForKey:@"temp"];
            
            NSString *grid_level=[dict objectForKey:@"grid_level"];
            
            NSString *humidiity=[dict objectForKey:@"humidity"];
            
            NSString *pressure=[dict objectForKey:@"pressure"];
            NSString *sealevel=[dict objectForKey:@"sea_level"];
            NSString *temp_min=[dict objectForKey:@"temp_min"];
            NSString *temp_max=[dict objectForKey:@"temp_max"];
            
            NSArray *array=[responseObject objectForKey:@"weather"];
            NSDictionary *dict2=[array objectAtIndex:0];
            NSString *description=[dict2 objectForKey:@"description"];
            NSString *main = [dict2 objectForKey:@"main"];
            
            NSString *outputMain = [main uppercaseString];
            if([outputMain isEqualToString:@"CLOUDY"]){
                [self play:@"It is cloudy so it may rain. Please carry umberilla. Take care bye"];
            }
            else if ([outputMain isEqualToString:@"RAIN"]){
                [self play:@"It is raining outside Carry umberilla. Take care bye"];
            }
            else if ([outputMain isEqualToString:@"CLEAR"]){
                NSString *body = [NSString stringWithFormat:@"Sky is clear so you can go out. Highest temperature is %@ and lowest temperature is %@. Take care bye",temp_max,temp_min];
                [self play:body];
            }
        }failure:^(AFHTTPRequestOperation *operation, NSError *error) {
            
            NSLog(@"Error: %@",error);
            
            UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"Error"
                                                            message:[error description]
                                                           delegate:nil
                                                  cancelButtonTitle:@"Ok"
                                                  otherButtonTitles:nil, nil];
            [alert show];
        }];

        
    }
    
    else if ([cmd isEqualToString:@"SEND"]){
        
        
        NSLog(@"Sending the weather reports");
         
         [self play:@"sending details"];
        [self sendMessageWeather];
    }
    else if ([cmd isEqualToString:@"GREET THE DOCTOR"]){
        NSLog(@"greet");
        [self play:@"Good Afternoon Doctor"];
        //[self.commandPlayer playCommand:@"GooDAfternoon.mp3"];
    }
    
    else if ([cmd isEqualToString:@"WHAT IS YOUR NAME"]){
        NSLog(@"Name");
        [self play:@"My Name is Robot"];
        //[self.commandPlayer playCommand:@"GooDAfternoon.mp3"];
    }
    //else if([cmd isEqualToString:@""])
    
    //accelerometer
    else if ([cmd isEqualToString:@"WILL YOU START MONITORING"]) {
        [self setupCamera];
        [self turnCameraOn];
        
        [self play:@"Yes I am about to start"];
        
        triggerImageURL = [[NSBundle mainBundle] pathForResource:@"stop" ofType:@"bmp"];
        
       // [self flipAction];
        
        [self.roboMe sendCommand:kRobot_HeadTiltDown1];
        
        // triggerImageURL = [[NSBundle mainBundle] pathForResource:@"stop" ofType:@"bmp"];
        
        if(speed <= 0){
            speed = 0.3;
                    [self.roboMe sendCommand: kRobot_MoveForwardSpeed3];;
            NSLog(@"%f",speed);
        }
        else{
            
                    [self.roboMe sendCommand: kRobot_MoveForwardSpeed2];
            NSLog(@"%f",speed);
        }
        NSLog(@"Before Accelerometer");
        [self checkAccelerometer];
        NSLog(@"After Accelreomter");
    }
    
    //taking picture
    
    else if([cmd isEqualToString:@"PICTURE"]){
        [self setupCamera];
        [self turnCameraOn];
       UIImagePickerController *poc = [[UIImagePickerController alloc] init];
        [poc setTitle:@"Take a photo."];
       // [poc setDelegate:self];
        [poc setSourceType:UIImagePickerControllerSourceTypeCamera];
        poc.showsCameraControls = NO;
        NSLog(@"Before taking picture");
        [poc takePicture];
        NSLog(@"Picture is taken");
    }
    
    //camera
    else if ([cmd isEqualToString:@"CAMERA"]) {
        
        NSLog(@"inside>>>>>");
        
        AVCaptureSession *session = [[AVCaptureSession alloc] init];
        NSError *error = nil;
        [session setSessionPreset:AVCaptureSessionPresetLow];
        
        NSArray *devices = [AVCaptureDevice devices];
        for (AVCaptureDevice *device in devices) {
            NSLog(@"Device name: %@", [device localizedName]);
            if([[device localizedName] isEqual:@"Front Camera"]){
                NSLog(@"front camera checked");
                //aquiring the lock
                if ([device isFocusModeSupported:AVCaptureFocusModeLocked]) {
                    
                    if ([device lockForConfiguration:&error]) {
                        device.focusMode = AVCaptureFocusModeLocked;
                        [device unlockForConfiguration];
                    }
                }
                AVCaptureDeviceInput *input =
                [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];
                if (!input) {
                    // Handle the error appropriately.
                    NSLog(@"no imput detected");
                    
                }
                else{
                    NSLog(@"input detected");
                    AVCaptureSession *captureSession = session;
                    AVCaptureDeviceInput *captureDeviceInput = input;
                    if ([captureSession canAddInput:captureDeviceInput]) {
                        NSLog(@"success in adding input");
                        [captureSession addInput:captureDeviceInput];
                        AVCaptureVideoPreviewLayer *previewLayer = [[AVCaptureVideoPreviewLayer alloc] initWithSession:session];
                        [previewLayer setVideoGravity:AVLayerVideoGravityResizeAspectFill];
                        CALayer *rootLayer = [[self view] layer];
                        [rootLayer setMasksToBounds:YES];
                        [previewLayer setFrame:CGRectMake(-70, 0, rootLayer.bounds.size.height, rootLayer.bounds.size.height)];
                        [rootLayer insertSublayer:previewLayer atIndex:0];
                        [captureSession startRunning];
                        
                        
                        
                        //capturing image
                        NSLog(@"before still ");
                        AVCaptureStillImageOutput *stillImageOutput = [[AVCaptureStillImageOutput alloc] init];
                        NSDictionary *outputSettings = @{ AVVideoCodecKey : AVVideoCodecJPEG};
                        [stillImageOutput setOutputSettings:outputSettings];
                        
                        NSLog( @"%@",stillImageOutput.description);
                        NSLog(@"after still ");
                        
                        AVCaptureConnection *videoConnection = nil;
                        for (AVCaptureConnection *connection in stillImageOutput.connections) {
                            for (AVCaptureInputPort *port in [connection inputPorts]) {
                                if ([[port mediaType] isEqual:AVMediaTypeVideo] ) {
                                    videoConnection = connection;
                                    break;
                                }
                            }
                            if (videoConnection) { NSLog(@"Got video connection. breaking from loop");break; }
                        }
                    }
                    else {
                        // Handle the failure.
                        NSLog(@"failure in adding input");
                    }
                }
                
                
            }
        }
    }
   
    
   // else if([cmd isEqualToString:@""])
}



// Play the the input

-(void) play:(NSString *)input {
    
//    NSLog(@"Hello How are you");
//    NSString* escapedUrlString = [input stringByAddingPercentEscapesUsingEncoding: NSASCIIStringEncoding];
//    
//       NSString *_language_tf = @"en";
//    if([escapedUrlString length]>0 && [@"en" length]>0)
//    {
//        [_playButton setEnabled:NO];
//        NSString *_sound =[NSString stringWithFormat:@"http://translate.google.com/translate_tts?tl=%@&q=%@",_language_tf,escapedUrlString];
//        NSURL *_url =[[NSURL alloc] initWithString:_sound];
//        
//        _player = [[AVPlayer alloc] initWithURL:_url];
//        [_player play];
//        
////        _player.actionAtItemEnd = AVPlayerActionAtItemEndNone;
////        [[NSNotificationCenter defaultCenter] addObserver:self
////                                                 selector:@selector(PlayerItemDidReachEnd:)
////                                                     name:AVPlayerItemDidPlayToEndTimeNotification
////                                                   object:[_player currentItem]];
//
    
        
        AFHTTPRequestOperationManager *manager = [AFHTTPRequestOperationManager manager];
        
        NSString *sentence = [input stringByAddingPercentEncodingWithAllowedCharacters:[NSCharacterSet URLQueryAllowedCharacterSet]];
        
        NSString *url = [NSString stringWithFormat:@"%@q=%@",TTP , sentence];
        
        NSLog(@"url is: %@",url);
        
        manager.responseSerializer = [AFHTTPResponseSerializer serializer];
        
        [manager GET:url parameters:nil success:^(AFHTTPRequestOperation *operation, id responseObject) {
            
            operation.responseSerializer.acceptableContentTypes = [NSSet setWithObjects:@"audio/mpeg",nil];
            
            NSData *audioData = responseObject;
            
            [[AVAudioSession sharedInstance] setCategory:AVAudioSessionCategoryPlayback error:nil];
            
            [[AVAudioSession sharedInstance] setActive:YES error:nil];
            
            self.audioPlayer = [[AVAudioPlayer alloc] initWithData:audioData error:nil] ;
            
            [self.audioPlayer prepareToPlay];
            
            [self.audioPlayer play];
            
        } failure:^(AFHTTPRequestOperation *operation, NSError *error) {
            
            NSLog(@"Error: %@", error);
            
        }];
}




//

//TO move camera front


- (void)flipAction
{
    _useBackCamera = !_useBackCamera;
    
    [self turnCameraOff];
    [self setupCamera];
    [self turnCameraOn];
}



#pragma mark -
#pragma mark -Twilio

-(void) sendMessageWeather{
    NSString *twilioSID = @"AC1af95fd1bead34c082aa5a8c842b332f";
    NSString *twilioAuthKey = @"2d95f11dd5aec00d865e64d33563353d";
    NSString *fromNumber = @"+13173163054";
    NSString *ToNumber = @"+14696235966";
    NSString *bodyMessage;
    
    NSLog(@"starting the application");
    NSString *locationName=@"Kansas";
    NSString *countryName=@"us";
    NSLog(@"locationName%@",locationName);
    NSLog(@"countryName%@",countryName);
    NSString *urlString=[NSString stringWithFormat:@"http://api.openweathermap.org/data/2.5/weather?q=%@,%@",locationName,countryName];
    
    NSLog(@"URL String%@", urlString);
    
    AFHTTPRequestOperationManager *manager = [AFHTTPRequestOperationManager manager];
    
    [manager GET:urlString parameters:nil success:^(AFHTTPRequestOperation *operation, id responseObject) {
        
        NSDictionary *dict = [responseObject objectForKey:@"main"];
        
        NSString *temperature= [dict objectForKey:@"temp"];
        
        NSString *grid_level=[dict objectForKey:@"grid_level"];
        
        NSString *humidiity=[dict objectForKey:@"humidity"];
        
        NSString *pressure=[dict objectForKey:@"pressure"];
        NSString *sealevel=[dict objectForKey:@"sea_level"];
        NSString *temp_min=[dict objectForKey:@"temp_min"];
        NSString *temp_max=[dict objectForKey:@"temp_max"];
        
        NSArray *array=[responseObject objectForKey:@"weather"];
        NSDictionary *dict2=[array objectAtIndex:0];
        NSString *description=[dict2 objectForKey:@"description"];
        NSString *main = [dict2 objectForKey:@"main"];
        
        NSLog(@"Temperature: %@", temperature);
        NSLog(@"temp_min: %@",temp_min);
        
        NSLog(@"description: %@",description);
        
        NSString *messageBody = [NSString stringWithFormat:@"Temperature: %@, Min temp: %@, temp max: %@, description: %@, Humidity: %@", temperature,temp_min,temp_max,description,humidiity];
        //bodyMessage = @"Temperature: %@, Min temp: %@, temp max: %@, description: %@, Humidity: %@", temperature
        
        //Starting point to send the messages
        
        NSString *urlString = [NSString stringWithFormat:@"https://%@:%@@api.twilio.com/2010-04-01/Accounts/%@/SMS/Messages", twilioSID, twilioAuthKey, twilioSID];
        
        NSURL *url = [NSURL URLWithString:urlString];
        
        NSMutableURLRequest *request = [[NSMutableURLRequest alloc] init];
        [request setURL:url];
        [request setHTTPMethod:@"POST"];
        
        //Set up the body the request
        
        NSString *bodyString = [NSString stringWithFormat:@"From=%@&To=%@&Body=%@", fromNumber,ToNumber,messageBody];
        
        NSData *data =[bodyString dataUsingEncoding:NSUTF8StringEncoding];
        
        
        [request setHTTPBody:data];
        
        NSError *error;
        
        NSURLResponse *response;
        
        NSData *receivedData = [NSURLConnection sendSynchronousRequest:request returningResponse:&response error:&error];
        
        //Handle the received data
        
        if(error){
            NSLog(@"Error:%@", error);
        }else{
            NSString *receivedString = [[NSString alloc]initWithData:receivedData encoding:NSUTF8StringEncoding];
            NSLog(@"Request sent.%@",receivedString);
        }
    }failure:^(AFHTTPRequestOperation *operation, NSError *error) {
        
        NSLog(@"Error: %@",error);
        
        UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"Error"
                                                        message:[error description]
                                                       delegate:nil
                                              cancelButtonTitle:@"Ok"
                                              otherButtonTitles:nil, nil];
        [alert show];
    }];

}


-(void) sendMessage{
    NSString *twilioSID = @"AC1af95fd1bead34c082aa5a8c842b332f";
    NSString *twilioAuthKey = @"2d95f11dd5aec00d865e64d33563353d";
    NSString *fromNumber = @"+13173163054";
    NSString *ToNumber = @"+13177931773";
    NSString *bodyMessage;
    
    NSLog(@"starting the application");
    NSString *locationName=@"Kansas";
    NSString *countryName=@"us";
    NSLog(@"locationName%@",locationName);
    NSLog(@"countryName%@",countryName);
    NSString *urlString=[NSString stringWithFormat:@"http://api.openweathermap.org/data/2.5/weather?q=%@,%@",locationName,countryName];
    
    NSLog(@"URL String%@", urlString);
    
    AFHTTPRequestOperationManager *manager = [AFHTTPRequestOperationManager manager];
    
    [manager GET:urlString parameters:nil success:^(AFHTTPRequestOperation *operation, id responseObject) {
        
        NSDictionary *dict = [responseObject objectForKey:@"main"];
        
        NSString *temperature= [dict objectForKey:@"temp"];
        
        NSString *grid_level=[dict objectForKey:@"grid_level"];
        
        NSString *humidiity=[dict objectForKey:@"humidity"];
        
        NSString *pressure=[dict objectForKey:@"pressure"];
        NSString *sealevel=[dict objectForKey:@"sea_level"];
        NSString *temp_min=[dict objectForKey:@"temp_min"];
        NSString *temp_max=[dict objectForKey:@"temp_max"];
        
        NSArray *array=[responseObject objectForKey:@"weather"];
        NSDictionary *dict2=[array objectAtIndex:0];
        NSString *description=[dict2 objectForKey:@"description"];
        NSString *main = [dict2 objectForKey:@"main"];
        
        NSLog(@"Temperature: %@", temperature);
        NSLog(@"temp_min: %@",temp_min);
        
        NSLog(@"description: %@",description);
        
        NSString *messageBody = [NSString stringWithFormat:@"Red Mark has been identified please take action"];
        
//        NSString *messageBody = [NSString stringWithFormat:@"Temperature: %@, Min temp: %@, temp max: %@, description: %@, Humidity: %@", temperature,temp_min,temp_max,description,humidiity];
        //bodyMessage = @"Temperature: %@, Min temp: %@, temp max: %@, description: %@, Humidity: %@", temperature
        
        //Starting point to send the messages
        
        NSString *urlString = [NSString stringWithFormat:@"https://%@:%@@api.twilio.com/2010-04-01/Accounts/%@/SMS/Messages", twilioSID, twilioAuthKey, twilioSID];
        
        NSURL *url = [NSURL URLWithString:urlString];
        
        NSMutableURLRequest *request = [[NSMutableURLRequest alloc] init];
        [request setURL:url];
        [request setHTTPMethod:@"POST"];
        
        //Set up the body the request
        
        NSString *bodyString = [NSString stringWithFormat:@"From=%@&To=%@&Body=%@", fromNumber,ToNumber,messageBody];
        
        NSData *data =[bodyString dataUsingEncoding:NSUTF8StringEncoding];
        
        
        [request setHTTPBody:data];
        
        NSError *error;
        
        NSURLResponse *response;
        
        NSData *receivedData = [NSURLConnection sendSynchronousRequest:request returningResponse:&response error:&error];
        
        //Handle the received data
        
        if(error){
            NSLog(@"Error:%@", error);
        }else{
            NSString *receivedString = [[NSString alloc]initWithData:receivedData encoding:NSUTF8StringEncoding];
            NSLog(@"Request sent.%@",receivedString);
        }
    }failure:^(AFHTTPRequestOperation *operation, NSError *error) {
        
        NSLog(@"Error: %@",error);
        
        UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"Error"
                                                        message:[error description]
                                                       delegate:nil
                                              cancelButtonTitle:@"Ok"
                                              otherButtonTitles:nil, nil];
        [alert show];
    }];
    
}

//Ending of the twilip apis
#pragma mark -
#pragma mark Socket

- (void)toggleSocketState
{
    if(!isRunning)
    {
        NSError *error = nil;
        if(![listenSocket acceptOnPort:PORT error:&error])
        {
            [self log:FORMAT(@"Error starting server: %@", error)];
            return;
        }
        
        [self log:FORMAT(@"Echo server started on port %hu", [listenSocket localPort])];
        isRunning = YES;
    }
    else
    {
        // Stop accepting connections
        [listenSocket disconnect];
        
        // Stop any client connections
        @synchronized(connectedSockets)
        {
            NSUInteger i;
            for (i = 0; i < [connectedSockets count]; i++)
            {
                // Call disconnect on the socket,
                // which will invoke the socketDidDisconnect: method,
                // which will remove the socket from the list.
                [[connectedSockets objectAtIndex:i] disconnect];
            }
        }
        
        [self log:@"Stopped Echo server"];
        isRunning = false;
    }
}

- (void)log:(NSString *)msg {
    NSLog(@"%@", msg);
}

- (NSString *)getIPAddress
{
    NSString *address = @"error";
    struct ifaddrs *interfaces = NULL;
    struct ifaddrs *temp_addr = NULL;
    int success = 0;
    
    // retrieve the current interfaces - returns 0 on success
    success = getifaddrs(&interfaces);
    if (success == 0) {
        // Loop through linked list of interfaces
        temp_addr = interfaces;
        while (temp_addr != NULL) {
            if( temp_addr->ifa_addr->sa_family == AF_INET) {
                // Check if interface is en0 which is the wifi connection on the iPhone
                if ([[NSString stringWithUTF8String:temp_addr->ifa_name] isEqualToString:@"en0"]) {
                    // Get NSString from C String
                    address = [NSString stringWithUTF8String:inet_ntoa(((struct sockaddr_in *)temp_addr->ifa_addr)->sin_addr)];
                }
            }
            
            temp_addr = temp_addr->ifa_next;
        }
    }
    // Free memory
    freeifaddrs(interfaces);
    
    return address;
}

#pragma mark -
#pragma mark GCDAsyncSocket Delegate

- (void)socket:(GCDAsyncSocket *)sock didAcceptNewSocket:(GCDAsyncSocket *)newSocket
{
    // This method is executed on the socketQueue (not the main thread)
    
    @synchronized(connectedSockets)
    {
        [connectedSockets addObject:newSocket];
    }
    
    NSString *host = [newSocket connectedHost];
    UInt16 port = [newSocket connectedPort];
    
    dispatch_async(dispatch_get_main_queue(), ^{
        @autoreleasepool {
            
            [self log:FORMAT(@"Accepted client %@:%hu", host, port)];
            
        }
    });
    
    NSString *welcomeMsg = @"Welcome to the AsyncSocket Echo Server\r\n";
    NSData *welcomeData = [welcomeMsg dataUsingEncoding:NSUTF8StringEncoding];
    
    [newSocket writeData:welcomeData withTimeout:-1 tag:WELCOME_MSG];
    
    
    [newSocket readDataWithTimeout:READ_TIMEOUT tag:0];
    newSocket.delegate = self;
    
    //    [newSocket readDataToData:[GCDAsyncSocket CRLFData] withTimeout:READ_TIMEOUT tag:0];
}

- (void)socket:(GCDAsyncSocket *)sock didWriteDataWithTag:(long)tag
{
    // This method is executed on the socketQueue (not the main thread)
    
    if (tag == ECHO_MSG)
    {
        [sock readDataToData:[GCDAsyncSocket CRLFData] withTimeout:100 tag:0];
    }
}

- (void)socket:(GCDAsyncSocket *)sock didReadData:(NSData *)data withTag:(long)tag
{
    
    NSLog(@"== didReadData %@ ==", sock.description);
    
    NSString *msg = [[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding];
    
    [self log:msg];
    [self perform:msg];
    [sock readDataWithTimeout:READ_TIMEOUT tag:0];
}

/**
 * This method is called if a read has timed out.
 * It allows us to optionally extend the timeout.
 * We use this method to issue a warning to the user prior to disconnecting them.
 **/
- (NSTimeInterval)socket:(GCDAsyncSocket *)sock shouldTimeoutReadWithTag:(long)tag
                 elapsed:(NSTimeInterval)elapsed
               bytesDone:(NSUInteger)length
{
    if (elapsed <= READ_TIMEOUT)
    {
        NSString *warningMsg = @"Are you still there?\r\n";
        NSData *warningData = [warningMsg dataUsingEncoding:NSUTF8StringEncoding];
        
        [sock writeData:warningData withTimeout:-1 tag:WARNING_MSG];
        
        return READ_TIMEOUT_EXTENSION;
    }
    
    return 0.0;
}


//accelorometer dat collection
//- (void)startUpdatesWithSliderValue:(int)sliderValue
//{
//    
//    NSLog(@"in startUpdateswithSliderValue Accelerometer");
//    NSTimeInterval delta = 0.05;
//    NSTimeInterval updateInterval = accelerometerMin + delta * sliderValue;
//    
//    CMMotionManager *mManager = [(AppDelegate *) [[UIApplication sharedApplication] delegate] sharedManager];
//    
//    if([mManager isAccelerometerAvailable]) {
//        
//        [mManager setAccelerometerUpdateInterval:updateInterval];
//        
//        [mManager startAccelerometerUpdatesToQueue:[NSOperationQueue mainQueue] withHandler:^(CMAccelerometerData *accelerometerData, NSError *error) {
//            
//            a = accelerometerData.acceleration.x;
//            b = accelerometerData.acceleration.y;
//            c = accelerometerData.acceleration.z;
//            NSLog(@"x %f y %f z %f", a, b, c);
//            
//            if ((a <= 0.35 & a>= -0.35) & (b <= -0.75 & b >= -0.93) & (c<=0.62 & c>= -0.67)) {
//                [self.roboMe sendCommand:kRobot_MoveForwardSpeed3];
//                NSLog(@"Block1");
//            }
//            //declination
//            else if ((a <= 0.35 & a >= -0.35) & (b >= -1.0 & b <= -0.80 )& (c >= -0.55 & c<= -0.11)){
//                [self.roboMe sendCommand: kRobot_MoveForwardSlowest];
//                NSLog(@"Block2");
//            }
//            
//            else if ((a <= 0.35 & a >= -0.35)& ((b <= 1.0 & b >= -0.80) || (b >= -1.0 & b <= -0.80) )& (c >= -0.19 & c<= 0.50)){
//                [self.roboMe sendCommand:kRobot_MoveForwardSpeed3];
//                NSLog(@"Block3");
//            }
//            else if ((a >= 0.03 & a < 0.06) & (b >= -0.96 & b <= -0.85 )& (c >=0.38 & c<= 0.50)){
//                [self.roboMe sendCommand:kRobot_MoveForwardSpeed4];
//                NSLog(@"Block4");
//            }
//            else if ((a <= 0.35 & a >= -0.35)& ((b <= 1.0 & b >= -0.80) || (b >= -1.0 & b <= -0.80) )& (c >= -0.19 & c<= 0.50)){
//                [self.roboMe sendCommand:kRobot_MoveForwardSpeed5];
//                NSLog(@"Block5");
//            }
//            // inclination to high
//            else if ((a <= 0.35 & a >= -0.35) & ((b <= -0.02 & b >= -0.50) || (b >= -0.55 & b <= -0.75 ) )& (c >= -1.10 & c <= -0.76)){
//                [self.roboMe sendCommand:kRobot_MoveForwardFastest];
//                NSLog(@"Block6");
//            }
//            else if ((a <= 0.35 & a >= -0.35) & (b <= -0.55 & b >= -0.75 )& (c >= -0.81 & c<= -0.75)){
//                [self.roboMe sendCommand:kRobot_MoveForwardSpeed2];
//                NSLog(@"Block7");
//            }
//            else if ((a <= 0.35 & a >= -0.35) & ((b <= -0.02 & b >= -0.50) || (b >= -0.55 & b <= -0.75 ) )& (c >= -1.10 & c <= -0.76)){
//                [self.roboMe sendCommand:kRobot_MoveForwardSpeed3];
//                NSLog(@"Block8");
//            }
//            else if (((a <= 0.35 & a >= -0.35) & (b >= -0.02 & b <= 0.7 )& (c <=1.10 & c >= 0.76))){
//                
//                NSLog(@"Block9");
//                [self.roboMe sendCommand:kRobot_Stop];
//                [self.roboMe sendCommand:kRobot_IncreaseMood];
//                
//                
//            }
//            
//            //stopn in excess decination
//            
//            else if ((a >= 0.35 & a <= -0.35)& (b >= 1.0 & b <= -0.80 )& (c >= 0.50 & c<= 0.70)){
//                
//                
//                
//                [self.roboMe sendCommand:kRobot_Stop];
//                
//            }
//            else {
//                [self.roboMe sendCommand:kRobot_MoveForwardSpeed3];
//                
//            }
//            
//        }];
//    }
//    
//}



int count = 0 ;
-(void) checkAccelerometer
{
    NSLog(@"In Accelerometer");
    
    NSTimeInterval delta = 0.005;
    NSTimeInterval updateInterval = 1 + delta * 2;
    
    NSLog(@"Before CMManager");
    mManager = [(AppDelegate *)[[UIApplication sharedApplication] delegate] sharedManager];
    
    NSLog(@"After CMManager");
    // float alpha = (float) 0.2;
    //ViewController * __weak weakSelf = self;
    
    //  while()
    if ([mManager isAccelerometerAvailable] == YES) {
        [mManager setAccelerometerUpdateInterval:updateInterval];
        
        [mManager startAccelerometerUpdatesToQueue:[NSOperationQueue mainQueue] withHandler:^(CMAccelerometerData *accelerometerData, NSError *error) {
            
            xPoint1  = xPoint;
            yPoint1 = yPoint;
            zPoint1 = zPoint;
            xPoint =  accelerometerData.acceleration.x;
            yPoint =  accelerometerData.acceleration.y;
            zPoint =  accelerometerData.acceleration.z;
            
//            
//            if(zPoint - zPoint1 < 0 && yPoint - yPoint1 > 0.01)
//            {
//                slope_traversal = true;
//                if(speed < 0.6){
//                    speed = speed + 0.5;
//                    [self.roboMe sendCommand: kRobot_MoveForwardFastest];}
//                NSLog(@"IN z-z1 <0 and y-y1 > 0.15");
//                NSLog(@"Speed : %f", speed);
//                
//            }
//            else if(yPoint - yPoint1 < 0.01 &&zPoint-zPoint1<0){
//                
//                slope_traversal=true;
//                
//                if(speed > 0.4){
//                    speed = speed - 0.3;
//                    [self.roboMe sendCommand: kRobot_MoveForwardSlowest];
//                }
//                
//                NSLog(@"IN z-z1 <0 and y-y1 < 0.15");
//                NSLog(@"Speed : %f", speed);
//                
//            }
//            
//            else if(yPoint - yPoint1 < 0.005&& yPoint - yPoint1 > -0.001)
//            {
//                if(slope_traversal)
//                {
//                    [self.roboMe sendCommand: kRobot_Stop];
//
//                   
//                    slope_traversal = false;
//                   // self.Romo.expression=RMCharacterExpressionChuckle;
//                    //self.Romo.emotion=RMCharacterEmotionHappy;
//                    
//                    [self.roboMe sendCommand:kRobot_IncreaseMood];
//                    
//                    
//                    
//                    sleep(2);
//                    
//                    [self.roboMe sendCommand: kRobot_MoveForwardSpeed2];
//                }
//                
//            }
//            
//            else if (zPoint-zPoint1==0){
//                
//                if(speed < 0.6){
//                    speed = speed + 0.3;
//                    
//                    [self.roboMe sendCommand: kRobot_MoveForwardSpeed2];
//                    
//                }
//                NSLog(@"Z DIff : %f", zPoint1-zPoint1);
//                NSLog(@"Y Diff : %f", yPoint-yPoint1);
//                
//                NSLog(@"In Else");
//            }
//            else if(zPoint - zPoint1 > 2)
//            {
//                
//                if(speed > 0.4){
//                    speed = speed - 0.3;
//                    [self.roboMe sendCommand: kRobot_MoveForwardSlowest];
//                }
//                NSLog(@"IN z-z1 <0 and y-y1 < 0.15");
//                NSLog(@"Speed : %f", speed);
//                
//            }

            
            a = accelerometerData.acceleration.x;
                      b = accelerometerData.acceleration.y;
                        c = accelerometerData.acceleration.z;
                        NSLog(@"x %f y %f z %f", a, b, c);
            
            
            if ((a <= 0.35 & a>= -0.35) & (b <= -0.80 & b >= -0.93) & (c<=0.56 & c>= -0.67)) {
                
               // speed = speed1; //speed = 0.2
                
                NSLog(@"in loop1");
                
                NSLog(@"speed is : %f",speed);
                
                [self.roboMe sendCommand:kRobot_MoveForwardSpeed3];
                
            }
            
            //inclination
            
            else if ((a <= 0.35 & a >= -0.35) & (b >= -0.78 & b <= -0.50 )& (c >= -0.90 & c<= -0.58)){
                
               
                    
                    NSLog(@"in loop2");
                    
                  //  speed = speed1 + 0.2;
                    
                   // NSLog(@"speed is : %f",speed);
                    
                    [self.roboMe sendCommand:kRobot_MoveForwardSpeed4];
                    
                
                
            }
            
            else if ((a <= 0.35 & a >= -0.35) & (b <= -0.30 & b >= -0.45 )& (c >= -1.0 & c<= -0.85)){
                
                
                    NSLog(@"in loop3");
                    
                [self.roboMe sendCommand:kRobot_MoveForwardSpeed5];
                
            }
            
            else if ((a <= 0.35 & a >= -0.35) & ((b >=-0.25  & b <= -0.001) || (b >= -0.55 & b <= -0.75 ) )& (c >= -1.10 & c <= -0.76)){
                                    NSLog(@"in loop4");
                    
                [self.roboMe sendCommand:kRobot_MoveForwardFastest];
                
            }
            
            
            
            
            
            
            
            // declination
            
            
            
            else if ((a <= 0.35 & a >= -0.35) & (b >= -1.0 & b <= -0.80 )& (c >= -0.55 & c<= -0.35)){
                
                NSLog(@"in loop5");
                
                [self.roboMe sendCommand:kRobot_MoveForwardSpeed3];
            }
            
            
            
            else if ((a <= 0.35 & a >= -0.35)& ((b <= 1.0 & b >= -0.99) || (b >= -1.0 & b <= -0.80) )& (c >= -0.40 & c<= -0.15)){
                
                NSLog(@"in loop6");
                
                [self.roboMe sendCommand:kRobot_MoveForwardSpeed2];
                
            }
            
            
            
            else if ((a <= 0.35 & a >= -0.35)& ((b <= 1.0 & b >= -0.99) || (b >= -1.0 & b <= -0.80) )& (c >= -0.15 & c<= 0.15)){
                
                
                
                NSLog(@"in loop7");
                [self.roboMe sendCommand:kRobot_MoveForwardSpeed1];
                
            }
            
            //stop in excess decination
            
            else if ((a <= 0.35 & a >= -0.35)& ((b <= 1.0 & b >= -0.99) || (b >= -1.0 & b <= -0.80) )& (c >= 0.10 & c<= 1.10)){
                
                [self.roboMe sendCommand:kRobot_Stop];
                
            }
            
            //stop in excess inclination
            
            else if (((a <= 0.35 & a >= -0.35) & ((b >= -0.01 & b <= 0.22) ||(b>= -0.45 & b<= -0.01))& (c >= -1.10 & c <= -0.76))){
                
                [self.roboMe sendCommand:kRobot_Stop];
                
            }
            
//                        if ((a <= 0.35 & a>= -0.35) & (b <= -0.75 & b >= -0.93) & (c<=0.62 & c>= -0.67)) {
//                            [self.roboMe sendCommand:kRobot_MoveForwardSpeed3];
//                           NSLog(@"Block1");
//                        }
//                        //declination
//                        else if ((a <= 0.35 & a >= -0.35) & (b >= -1.0 & b <= -0.80 )& (c >= -0.55 & c<= -0.11)){
//                            [self.roboMe sendCommand: kRobot_MoveForwardSlowest];
//                            NSLog(@"Block2");
//                        }
//            
//                        else if ((a <= 0.35 & a >= -0.35)& ((b <= 1.0 & b >= -0.80) || (b >= -1.0 & b <= -0.80) )& (c >= -0.19 & c<= 0.50)){
//                        [self.roboMe sendCommand:kRobot_MoveForwardSpeed3];
//                            NSLog(@"Block3");
//                        }
//                        else if ((a >= 0.03 & a < 0.06) & (b >= -0.96 & b <= -0.85 )& (c >=0.38 & c<= 0.50)){
//                            [self.roboMe sendCommand:kRobot_MoveForwardSpeed4];
//                            NSLog(@"Block4");
//                        }
//                        else if ((a <= 0.35 & a >= -0.35)& ((b <= 1.0 & b >= -0.80) || (b >= -1.0 & b <= -0.80) )& (c >= -0.19 & c<= 0.50)){
//                            [self.roboMe sendCommand:kRobot_MoveForwardSpeed5];
//                            NSLog(@"Block5");
//                        }
//                        // inclination to high
//                        else if ((a <= 0.35 & a >= -0.35) & ((b <= -0.02 & b >= -0.50) || (b >= -0.55 & b <= -0.75 ) )& (c >= -1.10 & c <= -0.76)){
//                            [self.roboMe sendCommand:kRobot_MoveForwardFastest];
//                            NSLog(@"Block6");
//                        }
//                        else if ((a <= 0.35 & a >= -0.35) & (b <= -0.55 & b >= -0.75 )& (c >= -0.81 & c<= -0.75)){
//                            [self.roboMe sendCommand:kRobot_MoveForwardSpeed2];
//                            NSLog(@"Block7");
//                        }
//                        else if ((a <= 0.35 & a >= -0.35) & ((b <= -0.02 & b >= -0.50) || (b >= -0.55 & b <= -0.75 ) )& (c >= -1.10 & c <= -0.76)){
//                            [self.roboMe sendCommand:kRobot_MoveForwardSpeed3];
//                            NSLog(@"Block8");
//                        }
//                        else if (((a <= 0.35 & a >= -0.35) & (b >= -0.02 & b <= 0.7 )& (c <=1.10 & c >= 0.76))){
//            
//                            NSLog(@"Block9");
//                            [self.roboMe sendCommand:kRobot_Stop];
//                            [self.roboMe sendCommand:kRobot_IncreaseMood];
//                            
//                            
//                        }
//            
//                        //stopn in excess decination
//            
//                        else if ((a >= 0.35 & a <= -0.35)& (b >= 1.0 & b <= -0.80 )& (c >= 0.50 & c<= 0.70)){
//                            
//                            
//                            
//                            [self.roboMe sendCommand:kRobot_Stop];
//                            
//                        }
//                        else {
//                            [self.roboMe sendCommand:kRobot_MoveForwardSpeed3];
//                            
//                       }

            
        }];
        
    }
    else
    {
        NSLog(@"Accelerometer not available");
    }
    
    
}



- (void)setupCamera
{
    _captureDevice = nil;
    
    NSArray *devices = [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo];
    
    for (AVCaptureDevice *device in devices)
    {
        if (device.position == AVCaptureDevicePositionFront && !_useBackCamera)
        {
            _captureDevice = device;
            break;
        }
//        if (device.position == AVCaptureDevicePositionBack && _useBackCamera)
//        {
//            _captureDevice = device;
//            break;
//        }
    }
    
    if (!_captureDevice)
        _captureDevice = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
}


- (void)turnCameraOn
{
    NSError *error;
    
    _session = [[AVCaptureSession alloc] init];
    [_session beginConfiguration];
    [_session setSessionPreset:AVCaptureSessionPresetMedium];
    
    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:_captureDevice error:&error];
    
    if (input == nil)
        NSLog(@"%@", error);
    
    [_session addInput:input];
    
    AVCaptureVideoDataOutput *output = [[AVCaptureVideoDataOutput alloc] init];
    [output setSampleBufferDelegate:self queue:dispatch_queue_create("myQueue", NULL)];
    output.videoSettings = @{(id)kCVPixelBufferPixelFormatTypeKey:@(kCVPixelFormatType_32BGRA)};
    output.alwaysDiscardsLateVideoFrames = YES;
    
    [_session addOutput:output];
    
    [_session commitConfiguration];
    [_session startRunning];
    NSLog(@"Camera turned on");
}


- (void)turnCameraOff
{
    [_session stopRunning];
    _session = nil;
}


- (void)captureOutput:(AVCaptureVideoDataOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection
{
    //NSLog(@"didoutSampleBuffer executed");
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    
    CVPixelBufferLockBaseAddress(imageBuffer, 0);
    
    void *baseAddress = CVPixelBufferGetBaseAddress(imageBuffer);
    
    size_t width = CVPixelBufferGetWidth(imageBuffer);
    size_t height = CVPixelBufferGetHeight(imageBuffer);
    
    IplImage *iplimage;
    if (baseAddress)
    {
        iplimage = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 4);
        iplimage->imageData = (char*)baseAddress;
    }
    
    IplImage *workingCopy = cvCreateImage(cvSize(height, width), IPL_DEPTH_8U, 4);
    
    if (_captureDevice.position == AVCaptureDevicePositionFront)
    {
        cvTranspose(iplimage, workingCopy);
    }
    else
    {
        cvTranspose(iplimage, workingCopy);
        cvFlip(workingCopy, nil, 1);
    }
    
    cvReleaseImageHeader(&iplimage);
    
    CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
    
    // NSLog(@"before invoking didcaptureImlImage");
    [self didCaptureIplImage:workingCopy];
}


#pragma mark - Image processing


static void ReleaseDataCallback(void *info, const void *data, size_t size)
{
#pragma unused(data)
#pragma unused(size)
    //  IplImage *iplImage = info;
    //  cvReleaseImage(&iplImage);
}


- (CGImageRef)getCGImageFromIplImage:(IplImage*)iplImage
{
    // NSLog(@"getCGImageFromIplImage invoked");
    size_t bitsPerComponent = 8;
    size_t bytesPerRow = iplImage->widthStep;
    
    size_t bitsPerPixel;
    CGColorSpaceRef space;
    
    if (iplImage->nChannels == 1)
    {
        bitsPerPixel = 8;
        space = CGColorSpaceCreateDeviceGray();
    }
    else if (iplImage->nChannels == 3)
    {
        bitsPerPixel = 24;
        space = CGColorSpaceCreateDeviceRGB();
    }
    else if (iplImage->nChannels == 4)
    {
        bitsPerPixel = 32;
        space = CGColorSpaceCreateDeviceRGB();
    }
    else
    {
        abort();
    }
    
    CGBitmapInfo bitmapInfo = kCGBitmapByteOrderDefault | kCGImageAlphaNone;
    CGDataProviderRef provider = CGDataProviderCreateWithData(iplImage,
                                                              iplImage->imageData,
                                                              0,
                                                              ReleaseDataCallback);
    const CGFloat *decode = NULL;
    bool shouldInterpolate = true;
    CGColorRenderingIntent intent = kCGRenderingIntentDefault;
    
    CGImageRef cgImageRef = CGImageCreate(iplImage->width,
                                          iplImage->height,
                                          bitsPerComponent,
                                          bitsPerPixel,
                                          bytesPerRow,
                                          space,
                                          bitmapInfo,
                                          provider,
                                          decode,
                                          shouldInterpolate,
                                          intent);
    CGColorSpaceRelease(space);
    CGDataProviderRelease(provider);
    return cgImageRef;
}


- (UIImage*)getUIImageFromIplImage:(IplImage*)iplImage
{
    // NSLog(@"getUIImageFromIplImage invoked");
    CGImageRef cgImage = [self getCGImageFromIplImage:iplImage];
    UIImage *uiImage = [[UIImage alloc] initWithCGImage:cgImage
                                                  scale:1.0
                                            orientation:UIImageOrientationUp];
    
    CGImageRelease(cgImage);
    return uiImage;
}


#pragma mark - Captured Ipl Image


//- (void)didCaptureIplImage:(IplImage *)iplImage
//{
//    IplImage *rgbImage = cvCreateImage(cvGetSize(iplImage), IPL_DEPTH_8U, 3);
//    cvCvtColor(iplImage, rgbImage, CV_BGR2RGB);
//    cvReleaseImage(&iplImage);
//
//    [self didFinishProcessingImage:rgbImage];
//}


#pragma mark - didFinishProcessingImage


- (void)didFinishProcessingImage:(IplImage *)iplImage
{
    //   NSLog(@"didFinishProcessingImage invoked");
    dispatch_async(dispatch_get_main_queue(), ^{
        //UIImage *uiImage =
        [self getUIImageFromIplImage:iplImage];
        //_imageView.image = uiImage;
    });
}



- (void)socketDidDisconnect:(GCDAsyncSocket *)sock withError:(NSError *)err
{
    if (sock != listenSocket)
    {
        dispatch_async(dispatch_get_main_queue(), ^{
            @autoreleasepool {
                [self log:FORMAT(@"Client Disconnected")];
            }
        });
        
        @synchronized(connectedSockets)
        {
            [connectedSockets removeObject:sock];
        }
    }
}


static BOOL _debug = NO;

//Started Commenting the did capture image

//- (void)didCaptureIplImage:(IplImage *)iplImage
//{
//    //ipl image is in BGR format, it needs to be converted to RGB for display in UIImageView
//    IplImage *imgRGB = cvCreateImage(cvGetSize(iplImage), IPL_DEPTH_8U, 3);
//    cvCvtColor(iplImage, imgRGB, CV_BGR2RGB);
//    cv::Mat matRGB = cv::Mat(imgRGB);
//    
//    //ipl imaeg is also converted to HSV; hue is used to find certain color
//    IplImage *imgHSV = cvCreateImage(cvGetSize(iplImage), 8, 3);
//    cvCvtColor(iplImage, imgHSV, CV_BGR2HSV);
//    
//    IplImage *imgThreshed = cvCreateImage(cvGetSize(iplImage), 8, 1);
//    
//    //it is important to release all images EXCEPT the one that is going to be passed to
//    //the didFinishProcessingImage: method and displayed in the UIImageView
//    cvReleaseImage(&iplImage);
//    
//    //filter all pixels in defined range, everything in range will be white, everything else
//    //is going to be black
//    cvInRangeS(imgHSV, cvScalar(160, 100, 100), cvScalar(179, 255, 255), imgThreshed);
//    
//    cvReleaseImage(&imgHSV);
//    
//    cv::Mat matThreshed = cv::Mat(imgThreshed);
//    
//    //smooths edges
//    cv::GaussianBlur(matThreshed,
//                     matThreshed,
//                     cv::Size(9, 9),
//                     2,
//                     2);
//    
//    //debug shows threshold image, otherwise the circles are detected in the
//    //threshold image and shown in the RGB image
//    if (_debug)
//    {
//        cvReleaseImage(&imgRGB);
//        [self didFinishProcessingImage:imgThreshed];
//    }
//    else
//    {
//        std::vector<cv::Vec3f> circles;
//        
//        //get circles
//        HoughCircles(matThreshed,
//                     circles,
//                     CV_HOUGH_GRADIENT,
//                     2,
//                     matThreshed.rows / 4,
//                     150,
//                     75,
//                     10,
//                     150);
//        
//        for (size_t i = 0; i < circles.size(); i++)
//        {
//            cout << "Circle position x = " << (int)circles[i][0] << ", y = " << (int)circles[i][1] << ", radius = " << (int)circles[i][2] << "\n";
//            
//            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//            
//            int radius = cvRound(circles[i][2]);
//            
//            circle(matRGB, center, 3, cvScalar(0, 255, 0), -1, 8, 0);
//            circle(matRGB, center, radius, cvScalar(0, 0, 255), 3, 8, 0);
//            
//            [self.roboMe sendCommand: kRobot_Stop];
//            //[self dealloc];
//            //NSLog(@"robome stopped");
//            //self.Romo.expression=RMCharacterExpressionChuckle;
//            //self.Romo.emotion=RMCharacterEmotionHappy;
//        }
//        
//        //threshed image is not needed any more and needs to be released
//        cvReleaseImage(&imgThreshed);
//        
//        //imgRGB will be released once it is not needed, the didFinishProcessingImage:
//        //method will take care of that
//        [self didFinishProcessingImage:imgRGB];
//    }
//}

// Ended commenting the did capture method

//Started Testing

- (void)didCaptureIplImage:(IplImage *)iplImage


{
    
    NSLog(@"11111");
    //    [self.view bringSubviewToFront:self.thumbNailImageView];
    
    //ipl image is in BGR format, it needs to be converted to RGB for display in UIImageView
    IplImage *imgRGB = cvCreateImage(cvGetSize(iplImage), IPL_DEPTH_8U, 1);
    //    cvCvtColor(iplImage, imgRGB, CV_BGR2GRAY);
    cvCvtColor(iplImage, imgRGB, CV_BGR2GRAY);
    //it is important to release all images once they are not needed EXCEPT the one
    //that is going to be passed to the didFinishProcessingImage: method and
    //displayed in the UIImageView
    cvReleaseImage(&iplImage);
    
    NSLog(@"222222");
    
    //here you can manipulate RGB image, e.g. blur the image or whatever OCV magic you want
    Mat matRGB = Mat(imgRGB);
    
    //smooths edges
    cv::GaussianBlur(matRGB,
                     matRGB,
                     cv::Size(19, 19),
                     10,
                     10);
    //Mat img_object = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
    //Mat img_scene = imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );
    
    NSLog(@"3333333");
    
    //    Mat img_object=imread([imageURL UTF8String], CV_LOAD_IMAGE_GRAYSCALE);
    Mat img_object= cv::imread([triggerImageURL UTF8String], CV_LOAD_IMAGE_GRAYSCALE);
    Mat img_scene = imgRGB;
    if( !img_object.data || !img_scene.data )
    { std::cout<< " --(!) Error reading images " << std::endl;  }
    
    
    NSLog(@"44444444");
    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    
    SurfFeatureDetector detector( minHessian );
    
    std::vector<KeyPoint> keypoints_object, keypoints_scene;
    
    detector.detect( img_object, keypoints_object );
    detector.detect( img_scene, keypoints_scene );
    
    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;
    
    NSLog(@"5555555");
    Mat descriptors_object, descriptors_scene;
    
    extractor.compute( img_object, keypoints_object, descriptors_object );
    extractor.compute( img_scene, keypoints_scene, descriptors_scene );
    
    //-- Step 3: Matching descriptor vectors using FLANN matcher
    if (keypoints_object.empty()) {
        //        cvError(0,"MatchFinder","1st key points descriptor empty",__FILE__,__LINE__);
    }
    else
        if (keypoints_scene.empty()) {
            //        cvError(0,"MatchFinder","2nd key points descriptor empty",__FILE__,__LINE__);
        }
        else{
            FlannBasedMatcher matcher;
            std::vector< DMatch > matches;
            if ( descriptors_object.empty() )
                cvError(0,"MatchFinder","1st descriptor empty",__FILE__,__LINE__);
            if ( descriptors_scene.empty() )
                cvError(0,"MatchFinder","2nd descriptor empty",__FILE__,__LINE__);
            else
                matcher.match( descriptors_object, descriptors_scene, matches );
            
            double max_dist = 0; double min_dist = 100;
            
            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i < descriptors_object.rows; i++ )
            { double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }
            
            printf("-- Max dist : %f \n", max_dist );
            printf("-- Min dist : %f \n", min_dist );
            
            //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
            std::vector< DMatch > good_matches;
            
            for( int i = 0; i < descriptors_object.rows; i++ )
            { if( matches[i].distance < 3*min_dist )
            { good_matches.push_back( matches[i]); }
            }
            
            Mat img_matches;
            cv::drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                            good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                            std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            
            //-- Localize the object
            std::vector<Point2f> obj;
            std::vector<Point2f> scene;
            
            for( int i = 0; i < good_matches.size(); i++ )
            {
                //-- Get the keypoints from the good matches
                obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
                scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
            }
            
            Mat H = findHomography( obj, scene, CV_RANSAC );
            
            //-- Get the corners from the image_1 ( the object to be "detected" )
            std::vector<Point2f> obj_corners(4);
            obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
            obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
            std::vector<Point2f> scene_corners(4);
            
            perspectiveTransform( obj_corners, scene_corners, H);
            
            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
            
            
            double area = contourArea(scene_corners);
            cout<<area;
            if(area>100)
            {
                cout<<"GOT IT !!!!\n Perform your action on object recognition here";
                
                [self.roboMe sendCommand:kRobot_Stop];
                
                [self.roboMe sendCommand:kRobot_HeartBeatOff];
                
                if ([mManager isAccelerometerActive] == YES) {
                    [self.roboMe sendCommand:kRobot_Stop];
                    [self play:@"Sending message to concerned authority"];
                    [self sendMessage];
                    [mManager stopAccelerometerUpdates];
                    
                    
                }
                
                [self turnCameraOff];
                
                NSLog(@"ROBOT HAS STOPPED");
            }
            //    double floored_scene_x[4], floored_scene_y[4];
            //
            //    for(int i=0;i<=3;i++)
            //    {
            //        cout<<scene_corners[i] <<img_object.cols;
            //        floored_scene_x[i]=floor(scene_corners[i].x);
            //        floored_scene_y[i]=floor(scene_corners[i].y);
            //    }
            //
            
            //-- Show detected matches
            //    imshow( "Good Matches & Object detection", img_matches );
            
            //    self.thumbNailImageView.image = [self imageWithCVMat:img_matches];
            //    waitKey(0);
            
            dispatch_async(dispatch_get_main_queue(), ^{
                
                _imageView.image=[self imageWithCVMat:img_matches];
            });
            
            //    _imageView.image=[UIImage imageNamed:@"hi.jpg"];
            //imgRGB will be released once it is not needed, the didFinishProcessingImage:
            //method will take care once it displays the image in UIImageView
        }
    
    [self didFinishProcessingImage:imgRGB];
}

bool isRectangle() {
    
}

- (UIImage *)imageWithCVMat:(const cv::Mat&)cvMat
{
    NSData *data = [NSData dataWithBytes:cvMat.data length:cvMat.elemSize() * cvMat.total()];
    
    CGColorSpaceRef colorSpace;
    
    if (cvMat.elemSize() == 1) {
        colorSpace = CGColorSpaceCreateDeviceGray();
    } else {
        colorSpace = CGColorSpaceCreateDeviceRGB();
    }
    
    CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);
    
    CGImageRef imageRef = CGImageCreate(cvMat.cols,                                     // Width
                                        cvMat.rows,                                     // Height
                                        8,                                              // Bits per component
                                        8 * cvMat.elemSize(),                           // Bits per pixel
                                        cvMat.step[0],                                  // Bytes per row
                                        colorSpace,                                     // Colorspace
                                        kCGImageAlphaNone | kCGBitmapByteOrderDefault,  // Bitmap info flags
                                        provider,                                       // CGDataProviderRef
                                        NULL,                                           // Decode
                                        false,                                          // Should interpolate
                                        kCGRenderingIntentDefault);                     // Intent
    
    UIImage *image = [[UIImage alloc] initWithCGImage:imageRef];
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);
    
    return image;
}


//Ended Testing


//clear video session
- (void)dealloc {
    AVCaptureInput* input = [_session.inputs objectAtIndex:0];
    [_session removeInput:input];
    AVCaptureVideoDataOutput* output = [_session.outputs objectAtIndex:0];
    [_session removeOutput:output];
    [_session stopRunning];
}
@end
