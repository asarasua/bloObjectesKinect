#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	//kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
    objectsImage.allocate(kinect.width, kinect.height);
    handsImage.allocate(kinect.width, kinect.height);
	//colorImg.allocate(kinect.width, kinect.height);
	//grayImage.allocate(kinect.width, kinect.height);
//	grayThreshNear.allocate(kinect.width, kinect.height);
//	grayThreshFar.allocate(kinect.width, kinect.height);
	
    //threshold = 300;
	floorNearThreshold = 10;
	floorFarThreshold = 50;
    
    handsNearThreshold = 51;
    handsFarThreshold = 100;
    
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
    
    bCalibratingBackground = false;
}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		//grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
			
			// or we do it ourselves - show people how they can work with the pixels
        unsigned char * pixObjects = objectsImage.getPixels();
        unsigned char * pixHands = handsImage.getPixels();
        
        for(int i = 0; i < kinect.getWidth(); i++) {
            for (int j = 0; j < kinect.getHeight(); j++) {
                //OBJECTS ON THE FLOOR
                if (ofInRange(distanceToBackground(i, j), floorNearThreshold, floorFarThreshold)){
                    pixObjects[i + j*objectsImage.width] = 255;
                    pixHands[i + j*handsImage.width] = 0;
                //HANDS ON OBJECTS
                } else if (ofInRange(distanceToBackground(i, j), handsNearThreshold, handsFarThreshold)){
                    pixObjects[i + j*objectsImage.width] = 0;
                    pixHands[i + j*handsImage.width] = 255;
                } else {
                    pixObjects[i + j*objectsImage.width] = 0;
                    pixHands[i + j*handsImage.width] = 0;
                }
            }
		}
		
		// update the cv images
		objectsImage.flagImageChanged();
        handsImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		//contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
        //objectsFinder
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
        ofSetColor(0, 255, 0);
        objectsImage.draw(0, 0, ofGetWindowWidth()/2, ofGetWindowHeight()/2);
        ofSetColor(255, 0, 0);
        handsImage.draw(ofGetWindowWidth()/2, 0, ofGetWindowWidth()/2, ofGetWindowHeight()/2);
//		easyCam.begin();
//		drawPointCloud();
//		easyCam.end();
	} else {
		// draw from the live kinect
		kinect.drawDepth(0, 0, ofGetWindowWidth(), ofGetWindowHeight());
        int kinectMouseX = ofMap(ofGetMouseX(), 0, ofGetWindowWidth(), 0, kinect.getWidth());
        int kinectMouseY = ofMap(ofGetMouseY(), 0, ofGetWindowHeight(), 0, kinect.getHeight());
        ofSetColor(255, 0, 0);
        if (backgroundPoints.size() == 3){
            ofDrawBitmapString(ofToString(distanceToBackground(kinectMouseX, kinectMouseY)), ofGetMouseX(), ofGetMouseY());
            //ofDrawBitmapString(ofToString(backgroundPoints), ofGetMouseX(), ofGetMouseY());
        }
        else{
            ofDrawBitmapString(ofToString(kinect.getWorldCoordinateAt(kinectMouseX, kinectMouseY)), ofGetMouseX(), ofGetMouseY());
        }
        ofSetColor(255, 255, 255);
		//kinect.draw(420, 10, 400, 300);
		
		
//		contourFinder.draw(10, 320, 400, 300);
		
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
    
	if (bCalibratingBackground){
        reportStream << "Choose 3 corners!";
    }
    else {
        if(kinect.hasAccelControl()) {
            reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
            << ofToString(kinect.getMksAccel().y, 2) << " / "
            << ofToString(kinect.getMksAccel().z, 2) << endl;
        } else {
            reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
            << "motor / led / accel controls are not currently supported" << endl << endl;
        }
        reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
        << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
        << "set near threshold " << floorNearThreshold << " (press: + -)" << endl
        //	<< "set far threshold " << floorFarThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
        << "press b to calibrate bacground"
        << ", fps: " << ofGetFrameRate() << endl
        << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;
        if(kinect.hasCamTiltControl()) {
            reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
            << "press 1-5 & 0 to change the led mode" << endl;
        }
    }
    
	ofDrawBitmapString(reportStream.str(), 20, 652);
    
}

void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
		
        //FLOOR THRESHOOLD
		case '.':
			floorFarThreshold ++;
			if (floorFarThreshold > 450) floorFarThreshold = 450;
			break;
		case ',':
			floorFarThreshold --;
			if (floorFarThreshold < 0) floorFarThreshold = 0;
			break;
		case 'l':
			floorNearThreshold ++;
			if (floorNearThreshold > 255) floorNearThreshold = 255;
			break;
		case 'k':
			floorNearThreshold --;
			if (floorNearThreshold < 0) floorNearThreshold = 0;
			break;
            
        //HANDS THRESHOOLD
		case 'm':
			handsFarThreshold ++;
			if (handsFarThreshold > 450) handsFarThreshold = 450;
			break;
		case 'n':
			handsFarThreshold --;
			if (handsFarThreshold < 0) handsFarThreshold = 0;
			break;
		case 'j':
			handsNearThreshold ++;
			if (handsNearThreshold > 255) handsNearThreshold = 255;
			break;
		case 'h':
			handsNearThreshold --;
			if (handsNearThreshold < 0) handsNearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
            
        case 'b':
            bCalibratingBackground = !bCalibratingBackground;
            if (bCalibratingBackground){
                backgroundPoints.clear();
            }
            break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
    if (bCalibratingBackground) {
        if (backgroundPoints.size() < 3) {
            int kinectMouseX = ofMap(ofGetMouseX(), 0, ofGetWindowWidth(), 0, kinect.getWidth());
            int kinectMouseY = ofMap(ofGetMouseY(), 0, ofGetWindowHeight(), 0, kinect.getHeight());
            backgroundPoints.push_back(kinect.getWorldCoordinateAt(kinectMouseX, kinectMouseY));
        }
        if (backgroundPoints.size() == 3){
            bCalibratingBackground = false;
            background_v0 = backgroundPoints[0];
            background_n = (backgroundPoints[1]-backgroundPoints[0]).getCrossed(backgroundPoints[2]-backgroundPoints[0]).limit(1.0);
        }
    }
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}

//--------------------------------------------------------------
float testApp::distanceToBackground(int kinectMouseX, int kinectMouseY){
//    int kinectMouseX = ofMap(x, 0, ofGetWindowWidth(), 0, kinect.getWidth());
//    int kinectMouseY = ofMap(y, 0, ofGetWindowHeight(), 0, kinect.getHeight());
    
    ofVec3f p = kinect.getWorldCoordinateAt(kinectMouseX, kinectMouseY);
    
    return abs(background_n.dot(p-background_v0));
}
