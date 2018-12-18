#pragma once
#ifndef _IRCAMERA_DRIVER_H_
#define _IRCAMERA_DRIVER_H_

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for converting the command line parameter to integer
#include <string>
#include <cstring>
#include <unistd.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "libirimager/IRDeviceUVC.h"
#include "libirimager/IRImager.h"
#include "libirimager/IRImagerClient.h"
#include "libirimager/IRLogger.h"

using namespace std;
using namespace cv;
using namespace evo;


class IRCamera_Driver : public IRImagerClient
{
    private:
      string ver_ = "1.1";

    public:
      string param_xml_file = "";
      unsigned long device_id = -1;                   /*!< serial number */
      int fov;                                        /*!< Field of view */
      Tchar* opticsText;                              /*!<  */
      Tchar* formatsPath;                             /*!< Path to Format.def file  */
      Tchar* caliPath;                                /*!< Path to calibration files */
      int min_temperature;// = 20;                       /*!< Minimum of temperature range */
      int max_temperature;// = 100;                      /*!< Maximum of temperature range */
      float framerate = 120.0;                         /*!< Frame rate */
      int videoFormatIndex = 0;                       /*!< Used video format index, if multiple modes are supported by the device, e.g. PI400 format index 0: 32 Hz, 1: 80 Hz. */
      int bispectral = 0;                             /*!< Use bi-spectral mode, if available (e.g. PI200). */
      int average = 0;                                /*!< Activate average filter, if data stream is subsampled. */
      int autoFlag = 0;                               /*!< Use auto flag procedure. */
      float min_interval = 15.0;                      /*!< Minimum interval for a flag cycle. It defines the time in which a flag cycle is forced at most once.*/
      float max_interval = 0.0;                       /*!< Maximum interval for a flag cycle. It defines the time in which a flag cycle is forced at least once. */
      int pifInMode = 0;                              /*!< PIF in mode: 0=Image capture, 1=Flag control */
      //int pifOutMode;                               /*!< PIF out mode: 0=Image capture, 1=Flag control */
      //int pifOutVoltage;                            /*!< PIF out voltage in mV */
      int chip_heating_mode = 0;                      /*!< Chip heating: 0=Floating, 1=Auto, 2=Fixed value */
      float chip_heating_fixed_value = 40.0;          /*!< Fixed value for tChipMode=2 */

    private:
      unsigned short size;      /*!< Size of this structure */
      unsigned int counter;     /*!< Consecutively numbered for each received frame */
      unsigned int counterHW;   /*!< Hardware counter received from device, multiply with value returned by IRImager::getAvgTimePerFrame() to get a hardware timestamp */
      long long timestamp;      /*!< Time stamp in UNITS (10000000 per second) */
      long long timestampMedia;
      EnumFlagState flagState;  /*!< State of shutter flag at capturing time */
      float tempChip;           /*!< Chip temperature */
      float tempFlag;           /*!< Shutter flag temperature */
      float tempBox;            /*!< Temperature inside camera housing */

    public:
      /*
      float resolution_ratio;
      Mat image;
      ushort max_temperature_val = 0;
      Point max_temperature_pos;
      ushort min_temperature_val = 0;
      Point min_temperature_pos;
      */
      int width;
      int height;
      /*
      Mat projection;
      Mat cameraMatrix;
      Mat distortion;
      */
      unsigned short* thermal_image = NULL;
      unsigned char*  visible_image = NULL;
      bool flag_ir_thermal_image_new_frame = false;
      bool flag_ir_visible_image_new_frame = false;
      int flag_device_status = IRIMAGER_DISCONNECTED;
      unsigned int flag_status = -1;
      unsigned int time_delay;//microseconds;
      
    private:
      IRDeviceUVC* ir_device_ = NULL;
      IRImager ir_imager_;
      IRDeviceParams ir_params_;
      
      unsigned char* buffer_raw_ = NULL;
      double time_stamp_ = 0;
      bool flag_ir_device_init_ = false;
      bool flag_ir_params_init_ = false;
      bool flag_ir_imager_init_ = false;
      
      
      virtual void onRawFrame (unsigned char *data, int size);
      virtual void onThermalFrame(unsigned short* thermal, unsigned int w, unsigned int h, IRFrameMetadata meta, void* arg);
      virtual void onVisibleFrame(unsigned char* image, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg);
      virtual void onFlagStateChange(EnumFlagState flagstate, void* arg);
      virtual void onProcessExit(void* arg);
      
      //bool data_transform();
      //bool data_transformation_init();
      bool ir_device_init();
      bool ir_device_shutdown();
      bool ir_params_init();
      bool ir_imager_init();
    
    public:
      IRCamera_Driver(string param_xml_file);
      ~IRCamera_Driver();
      bool init(string param_xml_file);
      unsigned short* run(){if(ir_device_run()) return get_thermal_image(); else return NULL;};
      bool ir_device_run();
      unsigned short* get_thermal_image();
      unsigned char* get_visible_image();
      bool is_init(){return flag_ir_device_init_ & flag_ir_params_init_ & flag_ir_imager_init_;};
      bool is_open(){flag_ir_device_init_ = (ir_device_ == NULL)? false : ir_device_ -> isOpen(); return flag_ir_device_init_;};
      bool is_thermal_image_new_frame(){return flag_ir_thermal_image_new_frame;};
      bool is_visible_image_new_frame(){return flag_ir_visible_image_new_frame;};
};

IRCamera_Driver::IRCamera_Driver(string param_xml_file="")
{
  IRLogger::setVerbosity(IRLOG_ERROR, IRLOG_OFF);
  if(param_xml_file.length() > 0)
    this -> param_xml_file = param_xml_file;

  formatsPath = new Tchar [100];
  caliPath = new Tchar [100];

  strcpy(formatsPath , "/usr/share/libirimager");  /*!< Path to Format.def file  */
  strcpy(caliPath , "/usr/share/libirimager/cali");/*!< Path to calibration files */

  //device_id = ir_device_.findFirstDevice();
  init(param_xml_file);
}

IRCamera_Driver::~IRCamera_Driver()
{
  if(buffer_raw_ != NULL)
    delete [] buffer_raw_;
  /*
  if(thermal_image != NULL)
    delete [] thermal_image;
  if(visible_image != NULL)
    delete [] visible_image;
  */
  delete [] formatsPath;
  delete [] caliPath;
  ir_device_shutdown();
}

bool IRCamera_Driver::ir_device_run()
{
  int old_status = flag_device_status;
  flag_device_status = ir_device_ -> getFrame(buffer_raw_);//, &time_stamp_);
  if(flag_device_status == IRIMAGER_SUCCESS)
  {
    if(old_status != flag_device_status)
      cout << "IR Camera connected !\n";
    ir_imager_.process(buffer_raw_, NULL);
  }
  else if(flag_device_status == IRIMAGER_DISCONNECTED)
  {
    if(old_status != flag_device_status)
      cout << "IR Camera disconnected !\n";
    return false;
  }
  else if(flag_device_status == IRIMAGER_NOSYNC)
  {
    if(old_status != flag_device_status)
      cout << "IR Camera not sync !\n";
    return false;
  }
  else if(flag_device_status == IRIMAGER_STREAMOFF)
  {
    if(old_status != flag_device_status)
      cout << "IR Camera streaming off !\n";
    return false;
  }
  else
  {
    cout << "IR Camera unknown error !\n";
    return false;
  }
  return true;
}

unsigned short* IRCamera_Driver::get_thermal_image()
{
  if(!is_thermal_image_new_frame()) return NULL;
  flag_ir_thermal_image_new_frame = false;
  return thermal_image;
}

unsigned char* IRCamera_Driver::get_visible_image()
{
  if(!is_visible_image_new_frame()) return NULL;
  flag_ir_visible_image_new_frame = false;
  return visible_image;
}

bool IRCamera_Driver::init(string param_xml_file="")
{
  bool ir_dev = false, ir_img = false;
  cout << "-- Initializing ir params ...";
  if(param_xml_file.length() > 0)
    this -> param_xml_file = param_xml_file;
  if(ir_params_init())   cout << "ok !\n";
  else                   cout << "fail !\n";
  
  cout << "-- Initializing ir device ...";
  if(ir_device_init())
  {
    cout << "ok !\n";
    cout << "---- Find camera id : " << device_id << endl;
    ir_dev = true;
  }
  else                   cout << "fail !\n";
  
  cout << "-- Initializing ir imager ...";
  if(ir_imager_init())
  {
    cout << "ok !\n";
    cout << "---- Using framerate : " << ir_imager_.getMaxFramerate() << endl;
    cout << "---- Using width : " << width << endl;
    cout << "---- Using height : " << height << endl;
    ir_img = true;
  }
  else                   cout << "fail !\n";
    
  if(ir_dev & ir_img)
    ir_device_ -> startStreaming();
}
/*
bool IRCamera_Driver::data_transform()
{
  ushort value = ushort(buffer_raw_[0]);// + ushort(buffer_raw_[1]) * 0xFF + ushort(buffer_raw_[1]);
  ushort tvalue = (value - 1000.0) / 10.0;
  max_temperature_val = tvalue;
  max_temperature_pos.x = max_temperature_pos.y = 0;
  min_temperature = tvalue;
  min_temperature_pos.x = min_temperature_pos.y = 0;
  for(int r = 0; r < height; r++)
  {
    for(int c = 0; c < width; c++)
    {
      value = ushort(buffer_raw_[r*width + c]);// + ushort(buffer_raw_[r*width*2 + c*2+1]) * 256;
      image.at<ushort>(r, c) = value;
      tvalue = (value - 1000.0) / 10.0;
      if(max_temperature_val < tvalue)
      {
        max_temperature_val = tvalue;
        max_temperature_pos.x = c;
        max_temperature_pos.y = r;
      }
      if(min_temperature > tvalue)
      {
        min_temperature_val = tvalue;
        min_temperature_pos.x = c;
        min_temperature_pos.y = r;
      }
    }
  }
  return true;
}

bool IRCamera_Driver::data_transformation_init()
{
  if(!flag_ir_imager_init_) return false;
  image = Mat::zeros(height, width, CV_16UC1);
  return true;
}

*/

/*
bool IRCamera_Driver::onAutoFlag(AutoFlag::Request &req, AutoFlag::Response &res)
{
  _imager.setAutoFlag(req.autoFlag);
  res.isAutoFlagActive = _imager.getAutoFlag();
  return true;
}

bool IRCamera_Driver::onForceFlag(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  _imager.forceFlagEvent();
  return true;
}

bool IRCamera_Driver::onSetTemperatureRange(TemperatureRange::Request &req, TemperatureRange::Response &res)
{
  bool validParam = _imager.setTempRange(req.temperatureRangeMin, req.temperatureRangeMax);

  if(validParam)
  {
    _imager.forceFlagEvent(1000.f);
  }

  res.success = validParam;

  return true;
}
*/

bool IRCamera_Driver::ir_params_init()
{
  ir_params_.serial = device_id;
  ir_params_.fov = fov;
  ir_params_.opticsText = opticsText;
  ir_params_.formatsPath = formatsPath;
  ir_params_.caliPath = caliPath;
  ir_params_.tMin = min_temperature;
  ir_params_.tMax = max_temperature;
  ir_params_.framerate = framerate;
  ir_params_.videoFormatIndex = videoFormatIndex;
  ir_params_.bispectral = bispectral;
  ir_params_.average = average;
  ir_params_.autoFlag = autoFlag;
  ir_params_.minInterval = min_interval;
  ir_params_.maxInterval = max_interval;
  ir_params_.pifInMode = pifInMode;
  ir_params_.tChipMode = chip_heating_mode;
  ir_params_.tChipFixedValue = chip_heating_fixed_value;
  if(param_xml_file.length() > 0)
  {
    if(!IRDeviceParamsReader::readXML(param_xml_file.c_str(), ir_params_))
    {
      flag_ir_params_init_ = false;
      return false;
    }
  }
  flag_ir_params_init_ = true;
  return true;
}

bool IRCamera_Driver::ir_device_init()
{
  //device_id = ir_device_.findFirstDevice();
  if(!flag_ir_params_init_) return false;
  ir_device_ = IRDeviceUVC::createInstance(NULL, ir_params_.serial, ir_params_.videoFormatIndex);
  //ir_device_.openDevice();
  if(!ir_device_)
  {
    flag_ir_device_init_ = false;
    return false;
  }
    
  //ir_device_ -> startStreaming();
  flag_ir_device_init_ = true;
  return true;
}

bool IRCamera_Driver::ir_device_shutdown()
{
  if(!is_open()) return false;
  if(!ir_device_) return false;
  ir_device_ -> stopStreaming();
  ir_device_ -> closeDevice();
  delete ir_device_;
  ir_device_ = NULL;
  flag_ir_device_init_ = false;
  return true;
}

bool IRCamera_Driver::ir_imager_init()
{
  if(!is_open())
  {
    flag_ir_imager_init_ = false;
    return false;
  }
  flag_ir_imager_init_ = ir_imager_.init(&ir_params_,
                                         ir_device_ -> getFrequency(),
                                         ir_device_ -> getWidth(), 
                                         ir_device_ -> getHeight(), 
                                         ir_device_ -> controlledViaHID());
  ir_imager_.setClient(this);
  if(buffer_raw_ != NULL)
    delete [] buffer_raw_;
  buffer_raw_ = new unsigned char[ir_imager_.getRawBufferSize()];//38720 = 160 * 120 + 2
  
  width = ir_imager_.getWidth();
  height = ir_imager_.getHeight();
  thermal_image = new unsigned short [width * height];
  time_delay = 1.0/ir_imager_.getMaxFramerate();//*1000000;

  flag_ir_imager_init_ = true;
  return true;
}

void IRCamera_Driver::onRawFrame (unsigned char *data, int size)
{
  ir_imager_.process(buffer_raw_, NULL);
}

void IRCamera_Driver::onThermalFrame(unsigned short* thermal, unsigned int w, unsigned int h, IRFrameMetadata meta, void* arg)
{
  //if(thermal_image == NULL) thermal_image = new unsigned short[w*h];
  //memcpy(thermal_image, thermal, w*h*sizeof(*thermal));
  thermal_image = thermal;
  flag_ir_thermal_image_new_frame = true;
}

void IRCamera_Driver::onVisibleFrame(unsigned char* image, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg)
{
  if(!ir_imager_.hasBispectralTechnology()) return;
  //if(visible_image == NULL) visible_image = new unsigned short[2*w*h];
  //memcpy(visible_image.data[0], image, 2 * w * h * sizeof(*image));
  visible_image = image;
  flag_ir_visible_image_new_frame = true;
}

void IRCamera_Driver::onFlagStateChange(EnumFlagState flagstate, void* arg)
{
  /*
  optris_drivers::Flag flag;
  flag.flag_state      = flagstate;
  flag.header.frame_id = _thermal_image.header.frame_id;
  flag.header.stamp    = _thermal_image.header.stamp;
  _flag_pub.publish(flag);
  */
  flag_status = flagstate;
  //cout << "Flag Status : " << flag_status << endl;
}

void IRCamera_Driver::onProcessExit(void* arg)
{
}


#endif
