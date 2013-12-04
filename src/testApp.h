#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
    float distanceToBackground (int kinectX, int kinectY);
    
    //GUI control
    void floorThresholdChanged();
    void handsThresholdChanged();
    void objectsBlobSizeChanged();
    void handsBlobSizeChanged();
	
	ofxKinect kinect;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	
	//ofxCvColorImage colorImg;
    
    ofxCvGrayscaleImage objectsImage;
    ofxCvGrayscaleImage handsImage;
	
	//ofxCvGrayscaleImage grayImage; // grayscale depth image
//	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
//	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
//	
	ofxCvContourFinder objectsFinder;
    ofxCvContourFinder handsFinder;
    
    bool bCalibratingBackground;
    vector<ofVec3f> backgroundPoints;
    ofVec3f background_n;
    ofVec3f background_v0;
	
	bool bThreshWithOpenCV;
	bool bDrawDetectors;
    
    //GUI
    ofxPanel gui;
    ofxVec2Slider floorThresholdSlider;
    ofxVec2Slider handsThresholdSlider;
    ofxVec2Slider objectsBlobSize;
    ofxVec2Slider handsBlobSize;
	
	int angle;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
};
