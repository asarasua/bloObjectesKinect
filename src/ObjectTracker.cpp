//
//  ObjectTracker.cpp
//  bloObjectesKinect
//
//  Created by Álvaro Sarasúa Berodia on 11/12/13.
//
//

#include "ObjectTracker.h"

Object::Object(float _area){
    area = _area;
}

float Object::getArea(){
    return area;
}

void Object::setArea(float _area){
    area = _area;
}

unsigned int Object::getCategory(){
    return category;
}

void Object::setCategory(unsigned int _category){
    category = _category;
}

ObjectTracker::ObjectTracker(){
    
}

//_________________________________________________________________________OSC
void ObjectTracker::clearBundle() {
	bundle.clear();
}

template <>
void ObjectTracker::addMessage(string address, ofVec3f data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	msg.addFloatArg(data.x);
	msg.addFloatArg(data.y);
	msg.addFloatArg(data.z);
	bundle.addMessage(msg);
}

template <>
void ObjectTracker::addMessage(string address, ofVec2f data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	msg.addFloatArg(data.x);
	msg.addFloatArg(data.y);
	bundle.addMessage(msg);
}

template <>
void ObjectTracker::addMessage(string address, float data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	msg.addFloatArg(data);
	bundle.addMessage(msg);
}

template <>
void ObjectTracker::addMessage(string address, int data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	msg.addIntArg(data);
	bundle.addMessage(msg);
}

template <>
void ObjectTracker::addMessage(string address, unsigned int data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	msg.addIntArg(data);
	bundle.addMessage(msg);
}

template <>
void ObjectTracker::addMessage(string address, string data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	msg.addStringArg(data);
	bundle.addMessage(msg);
}

template <>
void ObjectTracker::addMessage(string address, bool data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	if (data) {
        msg.addIntArg(1);
    } else {
        msg.addIntArg(0);
    }
	bundle.addMessage(msg);
}

void ObjectTracker::sendBundle() {
	oscSender.sendBundle(bundle);
}

//_________________________________________________________________________________

void ObjectTracker::update(ofxCv::ContourFinder& objectFinder, ofxKinect& kinect){
    clearBundle();
    for (int i = 0; i < objectFinder.size(); ++i) {
        vector<cv::Point> quads = objectFinder.getFitQuad(i);
        float currentArea = kinect.getWorldCoordinateAt(quads[0].x, quads[0].y).distance(kinect.getWorldCoordinateAt(quads[1].x, quads[1].y))
        + kinect.getWorldCoordinateAt(quads[1].x, quads[1].y).distance(kinect.getWorldCoordinateAt(quads[2].x, quads[2].y));
        
        //Object already exists
        if (objects.find(objectFinder.getLabel(i)) != objects.end()) {
            //update object
            //update area if bigger
            if (currentArea > objects[objectFinder.getLabel(i)].getArea()) {
                objects[objectFinder.getLabel(i)].setArea(currentArea);
            }
        }
        //object doesn't exist
        else {
            //insert new object
            Object newObject(currentArea);
            objects[objectFinder.getLabel(i)] = newObject;
        }
        
        //Choose category
        selectCategory(objectFinder.getLabel(i));
        
        //Send OSC TODO: think about messages format
        addMessage("/Object/label", objectFinder.getLabel(i));
        addMessage("/Object/category", objects[i].getCategory());
    }        
}

void ObjectTracker::selectCategory(unsigned int _label){
    //TODO implement categories logic
    if (objects[_label].getArea() < 50) {
        objects[_label].setCategory(1);
    } else {
        objects[_label].setCategory(2);
    }
}