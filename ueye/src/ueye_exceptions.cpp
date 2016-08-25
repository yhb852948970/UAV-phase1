#include "disparity/ueye_exceptions.h"
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <iostream>

const std::string libueye_error_message="Internal libueye error: error code ";
const std::string feature_error_message="Feature error of ";
const std::string camera_error_message="Camera error of ";

CUeyeInternalException::CUeyeInternalException(const std::string& where,int error_code):CException(where,libueye_error_message)
{
  std::stringstream text;

  text << error_code;
  this->error_msg+=text.str();
}

CUeyeCameraException::CUeyeCameraException(const std::string& where,const std::string& error_msg):CException(where,camera_error_message)
{
  this->error_msg+=error_msg;
}

CUeyeFeatureException::CUeyeFeatureException(const std::string& where,const std::string& feature,const std::string& error_msg):CException(where,feature_error_message)
{
  this->error_msg+=error_msg;
  this->error_msg+=feature;
}
