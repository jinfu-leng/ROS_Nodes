/****************************************************************************
*
*   Copyright (c) 2013 Jinfu Leng and Carrick Detweiler
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*
* Started off from: http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
*
******************************************************************************/

#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <bluefox_cam.h>

using namespace std;

class BluefoxCam{
public:
	BluefoxCam();
	~BluefoxCam();
	void Spin();
private:
	// other functions
	bool Initialize();
	void LiveLoop(bool boSingleShotMode);

	// parameters
	bool boStoreFrames;
	int deviceNum;
	int width;
	int height;
	int defaultRequestCount;
	int outputStat;
	string pixelFormat;
	string acquisitionMode;

	// device drive
	DeviceManager devMgr;
	Device* pDev;

	// ros
	ros::NodeHandle nh;
	image_transport::CameraPublisher image_pub_;	
	sensor_msgs::Image img_;
	sensor_msgs::CameraInfo info_;
};

BluefoxCam::BluefoxCam() : nh("~")
{
	// load the parameters
    	nh.param("device_num", deviceNum, 0);
	nh.param("image_width", width, 640);
	nh.param("image_height", height, 480);
	nh.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
	nh.param("output_stat", outputStat, 0);
	//nh.param("pixel_format", pixelFormat, std::string("mjpeg")); // possible values: yuyv, uyvy, mjpeg
	//nh.param("autofocus", autofocus_, false); // enable/disable autofocus

	// other values
	defaultRequestCount = -1;
	pDev = 0;

	info_.header.frame_id = img_.header.frame_id;
	info_.height = height;
 	info_.width = width;

	// publisher
	image_transport::ImageTransport it(nh);
    	image_pub_ = it.advertiseCamera("image_raw", 1);
}
BluefoxCam::~BluefoxCam()
{
	if(pDev!=0)
		pDev->close();
}
// initialize device
bool BluefoxCam::Initialize()
{
	pDev = getDeviceFromDeviceNum( devMgr, deviceNum);

	if( pDev == 0 )
	{
		cout << "Unable to initialize the device!" << endl;
		return false;
	}

	// create an interface to the first MATRIX VISION device with the serial number sDevSerial
	if( pDev )
	{
		cout << "Initialising device: " << pDev->serial.read() << ". This might take some time..." << endl
			<< "Using interface layout '" << pDev->interfaceLayout.readS() << "'." << endl;
		try
		{
			if( defaultRequestCount > 0 )
			{
				cout << "Setting default request count to " << defaultRequestCount << endl;
				pDev->defaultRequestCount.write( defaultRequestCount );
			}
			pDev->open();
			switch( pDev->interfaceLayout.read() )
			{
			case dilGenICam:
				{
					DeviceComponentLocator locator(pDev, dltSetting, "Base");
					locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
					PropertyI64 w, h, pf, am;
					locator.bindComponent( w, "Width" );
					locator.bindComponent( h, "Height" );
					locator.bindComponent( pf, "PixelFormat" );
					locator.bindComponent( am, "AcquisitionMode" );
					if( width > 0 )
					{
						w.write( width );
					}
					if( height > 0 )
					{
						h.write( height );
					}
					if( !pixelFormat.empty() )
					{
						pf.writeS( pixelFormat );
					}
					if( !acquisitionMode.empty() )
					{
						am.writeS( acquisitionMode );
					}
					acquisitionMode = am.readS();
					cout << "Device set up to " << pf.readS() << " " << w.read() << "x" << h.read() << endl;
				}
				break;
			case dilDeviceSpecific:
				{
					DeviceComponentLocator locator(pDev, dltSetting, "Base");
					locator.bindSearchBase( locator.searchbase_id(), "Camera/Aoi" );
					PropertyI w, h;
					locator.bindComponent( w, "W" );
					locator.bindComponent( h, "H" );
					if( width > 0 )
					{
						w.write( width );
					}
					if( height > 0 )
					{
						h.write( height );
					}
					cout << "Device set up to " << w.read() << "x" << h.read() << endl;
				}
				break;
			default:
				break;
			}
		}
		catch( ImpactAcquireException& e )
		{
			// this e.g. might happen if the same device is already opened in another process...
			cout << "*** " << __FUNCTION__ << " - An error occurred while opening the device " << pDev->serial.read()
				<< "(error code: " << e.getErrorCode() << ", " << e.getErrorCodeAsString() << ")." << endl;
			return false;
		}		
	}
	return true;
}
//-----------------------------------------------------------------------------
void BluefoxCam::LiveLoop(bool boSingleShotMode )
//-----------------------------------------------------------------------------
{
	cout << " == " << __FUNCTION__ << " - establish access to the statistic properties...." << endl;
	// establish access to the statistic properties
	Statistics statistics( pDev );
	cout << " == " << __FUNCTION__ << " - create an interface to the device found...." << endl;
	// create an interface to the device found
	FunctionInterface fi( pDev );

	SystemSettings ss(pDev);
	// Prefill the capture queue with ALL buffers currently available. In case the acquisition engine is operated
	// manually, buffers can only be queued when they have been queued before the acquisition engine is started as well.
	// Even though there can be more then 1, for this sample we will work with the default capture queue
	int requestResult = DMR_NO_ERROR;
	int requestCount = 0;

	if( boSingleShotMode )
	{
		fi.imageRequestSingle();
		++requestCount;
	}
	else
	{
		while( ( requestResult = fi.imageRequestSingle() ) == DMR_NO_ERROR )
		{
			++requestCount;
		}
	}

	if( requestResult != DEV_NO_FREE_REQUEST_AVAILABLE )
	{
		cout << "Last result: "<< requestResult << "(" << ImpactAcquireException::getErrorCodeAsString( requestResult ) << "), ";
	}
	cout << requestCount << " buffers requested";

	if( ss.requestCount.hasMaxValue() )
	{
		cout << ", max request count: " << ss.requestCount.getMaxValue();
	}
	cout << endl;

	const bool boManualAcquisitionEngineControl = pDev->acquisitionStartStopBehaviour.isValid() && ( pDev->acquisitionStartStopBehaviour.read() == assbUser );
	if( boManualAcquisitionEngineControl )
	{
		cout << "Manual start/stop of acquisition engine requested." << endl;
		const int startResult = fi.acquisitionStart();
		cout << "Result of start: " << startResult << "("
			<< ImpactAcquireException::getErrorCodeAsString( startResult ) << ")" << endl;
	}

	// run thread loop
	const Request* pRequest = 0;
	const unsigned int timeout_ms = 8000;	// USB 1.1 on an embedded system needs a large timeout for the first image
	int requestNr = -1;
	//bool boLoopRunning = true;
	unsigned int cnt = 0;
	while( nh.ok() )
	{
		// wait for results from the default capture queue
		requestNr = fi.imageRequestWaitFor( timeout_ms );
		if( fi.isRequestNrValid( requestNr ) )
		{
			pRequest = fi.getRequest( requestNr );
			if( pRequest->isOK() )
			{
				// fill the image and then publish it
				fillImage(img_, "bgra8", pRequest->imageHeight.read(), pRequest->imageWidth.read(), 4 *pRequest->imageWidth.read(), pRequest->imageData.read());
				img_.header.stamp = ros::Time::now();
				info_.header.stamp = img_.header.stamp;
				image_pub_.publish(img_, info_);

				// here we can display some statistical information every 100th image
				++cnt;
				if( outputStat==1 && cnt%100 == 0 )
				{
					cout << cnt << ": Info from " << pDev->serial.read()
						<< ": " << statistics.framesPerSecond.name() << ": " << statistics.framesPerSecond.readS()
						<< ", " << statistics.errorCount.name() << ": " << statistics.errorCount.readS() 
						<< ", " << statistics.captureTime_s.name() << ": " << statistics.captureTime_s.readS() << " Image count: " << cnt 
						<< " (dimensions: " << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read() << ", format: " << pRequest->imagePixelFormat.readS();
					if( pRequest->imageBayerMosaicParity.read() != bmpUndefined )
					{
						cout << ", " << pRequest->imageBayerMosaicParity.name() << ": " << pRequest->imageBayerMosaicParity.readS();
					}
					cout << "), line pitch: " << pRequest->imageLinePitch.read() << endl;
				}
			}
			else
			{
				cout << "*** Error: A request has been returned with the following result: " << pRequest->requestResult << endl;
			}

			// this image has been sent thus the buffer is no longer needed...
			fi.imageRequestUnlock( requestNr );
			// send a new image request into the capture queue
			fi.imageRequestSingle();
			if( boManualAcquisitionEngineControl && boSingleShotMode )
			{
				const int startResult = fi.acquisitionStart();
				if( startResult != DMR_NO_ERROR )
				{
					cout << "Result of start: " << startResult << "("
						<< ImpactAcquireException::getErrorCodeAsString( startResult ) << ")" << endl;
				}
			}
		}
		else
		{
			cout << "*** Error: Result of waiting for a finished request: " << requestNr << "("
				<< ImpactAcquireException::getErrorCodeAsString( requestNr ) << "). Timeout value too small?" << endl;
		}

		//boLoopRunning = waitForInput( 0, STDOUT_FILENO ) == 0 ? true : false; // break by STDIN
	}

	// free resources
	fi.imageRequestReset( 0, 0 );
}
void BluefoxCam::Spin()
{
	ROS_INFO("Spin() is going well\n");
	if(Initialize()==false){
		ROS_INFO("Initialize() is not going well");
		return;
	}

	LiveLoop(acquisitionMode == "SingleFrame" );
}
// main
int main(int argc, char** argv){
	ros::init(argc, argv, "bluefox_cam");
	ROS_INFO("Bluefox_cam started");
	BluefoxCam bc;
	bc.Spin();
	return 0;
}
