//
//  ObjectTracker.h
//  bloObjectesKinect
//
//  Created by Álvaro Sarasúa Berodia on 11/12/13.
//
//

#ifndef bloObjectesKinect_ObjectTracker_h
#define bloObjectesKinect_ObjectTracker_h

#include "ofxCv.h"
#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOsc.h"

class Object{
protected:
    //cv::Point2f center;
    float area;
    unsigned int category;
    bool touched;
public:
    Object(){};
    Object(float Area);
    
    //cv::Point2f getCenter();
    
    void setArea (float _area);
    float getArea();
    
    void setCategory (unsigned int _category);
    unsigned int getCategory();
    
    void setTouched (bool _touched);
    bool isTouched ();
};

class ObjectTracker{
public:
    ObjectTracker();
    void update(ofxCv::ContourFinder& objectFinder, ofxCv::ContourFinder& handsFinder, ofxKinect& kinect);
 
private:
    void clearBundle();
	template <class T>
	void addMessage(string address, T data);
	void sendBundle();
    
    void selectCategory (unsigned int _label);
    
    map<unsigned int, Object> objects;
    
    ofxOscSender oscSender;
    ofxOscBundle bundle;
    
};

#endif
