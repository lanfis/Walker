#pragma once
#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <ros/ros.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <vector>
#include <chrono>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/RegionOfInterest.h>
/*
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/UInt16MultiArray.h>
*/

#include "MATRIX_LINK.h"

using namespace std;
using namespace ros;


class Matrix
{    
    private:
        #define LINK_SIZE 4
        #define CELL_SIZE LINK_SIZE*2
        #define UPDATE_CYCLE 128
        #define UPDATE_DURATION 1000

    public:
        string nodeName_ = "Matrix";
    
    private:
        string ver_ = "1.0";
        ros::NodeHandle n_;
        int queue_size_pub_ = 4;
        int queue_size_sub_ = 4;
        std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
        double elapsed;
        int update_cycle = UPDATE_CYCLE;

    
    private:
        ROS_Link **link_;

        int size_cell_pub_ = 0;
        int size_cell_sub_ = 0;
        Publisher_Cell_Float32  *cell_pub_ = NULL;
        Subscriber_Cell_Float32 *cell_sub_ = NULL;
//      boost::shared_ptr< std_msgs::Float32 > msg_cell_pub_[CELL_SIZE];
//      boost::shared_ptr< std_msgs::Float32 > msg_cell_sub_[CELL_SIZE];

    private:
        bool add_cell_pub(string topic);
        bool add_cell_sub(string topic);
        bool erase_cell(string& topic);
        bool search_cell_pub(string& topic);
        bool search_cell_sub(string& topic);
        bool search_cell_pub(string& topic, Publisher_Cell_Float32*& ptr);
        bool search_cell_sub(string& topic, Subscriber_Cell_Float32*& ptr);
    
    private:

    public:
        Matrix(ros::NodeHandle& nh);
        ~Matrix();
        void construct_cell_to_link();
        void check_link();
        void update_link();
        void run();
    
    public:
        virtual void init()
        {
            for(int i = 0; i < LINK_SIZE; i++)
            {
                link_[i] -> pub_init();
            }
        }
    
};

Matrix::Matrix(ros::NodeHandle& nh) : n_(nh)
{    
    link_ = new ROS_Link *[LINK_SIZE];
    cell_pub_ = new Publisher_Cell_Float32;
    cell_sub_ = new Subscriber_Cell_Float32;

    stringstream token;
    string idx;
    for(int i = 0; i < LINK_SIZE; i++)
    {
        token.clear();
        token << i;
        token >> idx;
        link_[i] = new ROS_Link(n_, nodeName_);
        link_[i] -> topic_link_pub_ = nodeName_ + "/link/" + idx;
        link_[i] -> topic_link_sub_ = nodeName_ + "/status/" + idx;
        link_[i] -> queue_size_pub = queue_size_pub_;
        link_[i] -> queue_size_sub = queue_size_sub_;
    }
    start = std::chrono::high_resolution_clock::now();
}

Matrix::~Matrix()
{
    for(int i = 0; i < LINK_SIZE; i++)
        delete link_[i];
    delete [] link_;
    if(cell_pub_ != NULL)
        delete cell_pub_;
    if(cell_sub_ != NULL)
        delete cell_sub_;
}

void Matrix::run()
{
    now = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
    if(elapsed > UPDATE_DURATION && update_cycle > 0)
    {
        update_link();
        start = std::chrono::high_resolution_clock::now();
        update_cycle -= 1;
        construct_cell_to_link();
    }
}

void Matrix::construct_cell_to_link()
{
    for(int i = 0; i < LINK_SIZE; i++)
    {
//      int size_link_pub = link_[i] -> get_link_pub_size();
//      int size_link_sub = link_[i] -> get_link_sub_size();
        for(int j = 0; j < link_[i] -> get_link_sub_size(); j++)
            add_cell_pub(link_[i] -> get_link_sub_topic(j));
        for(int j = 0; j < link_[i] -> get_link_pub_size(); j++)
            add_cell_sub(link_[i] -> get_link_pub_topic(j));
    }
}

void Matrix::check_link()
{
    for(int i = 0; i < LINK_SIZE; i++)
        if(!link_[i] -> is_link())
            link_[i] -> ping();
}

void Matrix::update_link()
{
    for(int i = 0; i < LINK_SIZE; i++)
        if(link_[i] -> is_link())
            link_[i] -> status();
}

bool Matrix::add_cell_pub(string topic)
{
    if(search_cell_pub(topic)) return false;
    Publisher_Cell_Float32 *new_pub = new Publisher_Cell_Float32;
    new_pub -> init(n_, topic, this -> queue_size_pub_);
    if(size_cell_pub_ > 0)
        cell_pub_ -> next -> prev = new_pub;
    new_pub -> next = cell_pub_ -> next;
    cell_pub_ -> next = new_pub;
    new_pub -> prev = cell_pub_;
    size_cell_pub_ += 1;
    return true;
}

bool Matrix::add_cell_sub(string topic)
{
    if(search_cell_sub(topic)) return false;
    Subscriber_Cell_Float32 *new_sub = new Subscriber_Cell_Float32;
    new_sub -> init(n_, topic, this -> queue_size_sub_);
    if(size_cell_sub_ > 0)
        cell_sub_ -> next -> prev = new_sub;
    new_sub -> next = cell_sub_ -> next;
    cell_sub_ -> next = new_sub;
    new_sub -> prev = cell_sub_;
    size_cell_sub_ += 1;
    return true;
}

bool Matrix::erase_cell(string& topic)
{
    Publisher_Cell_Float32  *ptr_pub;
    Subscriber_Cell_Float32 *ptr_sub;
    if(search_cell_pub(topic, ptr_pub))
    {
        if(ptr_pub -> next != NULL)
            ptr_pub -> next -> prev = ptr_pub -> prev;
        if(ptr_pub -> prev != NULL)
            ptr_pub -> prev -> next = ptr_pub -> next;
        ptr_pub -> prev = ptr_pub -> next = NULL;
        delete ptr_pub;
        size_cell_pub_ -= 1;
        return true;
    }
    if(search_cell_sub(topic, ptr_sub))
    {
        if(ptr_sub -> next != NULL)
            ptr_sub -> next -> prev = ptr_sub -> prev;
        if(ptr_sub -> prev != NULL)
            ptr_sub -> prev -> next = ptr_sub -> next;
        ptr_sub -> prev = ptr_sub -> next = NULL;
        delete ptr_sub;
        size_cell_sub_ -= 1;
        return true;
    }
    return false;
}

bool Matrix::search_cell_pub(string& topic)
{
    for(Publisher_Cell_Float32 *ptr = cell_pub_ ;ptr != NULL; ptr = ptr -> next)
        if(ptr -> getTopic() == topic)
            return true;
    return false;
}

bool Matrix::search_cell_sub(string& topic)
{
    for(Subscriber_Cell_Float32 *ptr = cell_sub_ ;ptr != NULL; ptr = ptr -> next)
        if(ptr -> getTopic() == topic)
            return true;
    return false;
}

bool Matrix::search_cell_pub(string& topic, Publisher_Cell_Float32*& ptr)
{
    for(ptr = cell_pub_ ;ptr != NULL; ptr = ptr -> next)
        if(ptr -> getTopic() == topic)
            return true;
    return false;
}

bool Matrix::search_cell_sub(string& topic, Subscriber_Cell_Float32*& ptr)
{
    for(ptr = cell_sub_ ;ptr != NULL; ptr = ptr -> next)
        if(ptr -> getTopic() == topic)
            return true;
    return false;
}


#endif

