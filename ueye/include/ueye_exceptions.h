#ifndef _UEYE_EXCEPTIONS
#define _UEYE_EXCEPTIONS

#include "exceptions.h"

/**
 * \brief Internal firewire exception
 *
 * This class is used to report internal libdc errors. These errors are
 * normally caused by unexpected error codes returned by the libdc library
 * functions. 
 *
 * In this case the error message is always the same and the error code
 * returned by the library functions is used to identify what happened.
 * See the libdc documentation for further description of the library errors.
 *
 * It is important to note that not all error codes returned by the libdc
 * libraries are errors. Some of them are only warning (those with negative
 * value), and no exception is thrown.
 */
class CUeyeInternalException : public CException
{
  public:
    /**
     * \brief Class constructor 
     *
     * This constructor generates a standard error message with the error code
     * provided as a parameter. The standard error message is "Internal libdc 
     * error: error code   ", with the error code appended at the end. The base
     * class constructor is called, so the same prefix is appended at the 
     * beginning to label the message as an exception.
     *
     * \param where a null terminated string with the information about the name
     *              of the function, the source code filename and the line where
     *              the exception was generated. This string must be generated 
     *              by the _HERE_ macro.
     *
     * \param error_code the error codereturned by one of the libdc library
     *                   functions. See the libdc documentation for further
     *                   details on the possible errors.
     */
    CUeyeInternalException(const std::string& where,int error_code);
};

/**
 * \brief Camera driver Exception
 *
 * This class is used to report errors of the camera driver which are not caused
 * by calls to the libdc library functions. These errors are normally caused by
 * providing incorrect parameters to the driver functions, attempting to
 * perform invalid operations or performing a sequence of operations in the 
 * incorrect order.
 *
 * This class is only provided to help distinguish between internal firewire
 * errors and errors of the driver itself, but it does not add any additional
 * feature over the base class CFirewireException.
 *
 */
class CUeyeCameraException : public CException
{
  public:
    /**
     * \brief Class constructor
     *
     * This constructor initializes the error message by calling the constructor
     * of the base class. In this case, "[Exception caught] - " is also 
     * pre-appended to the error message to label it as an exception.
     *
     * \param where a null terminated string with the information about the name
     *              of the function, the source code filename and the line where
     *              the exception was generated. This string must be generated 
     *              by the _HERE_ macro.
     *
     * \param error_msg a null terminated string that contains the error message.
     *                  This string may have any valid character and there is no 
     *                  limit on its length.
     */ 
    CUeyeCameraException(const std::string& where,const std::string& error_msg);
};

class CUeyeFeatureException : public CException
{
  public:
    CUeyeFeatureException(const std::string& where,const std::string& error_msg,const std::string& feature);
};

#endif
