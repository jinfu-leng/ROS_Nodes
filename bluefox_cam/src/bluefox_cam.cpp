#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <exampleHelper.h>

using namespace std;

#ifdef linux
#	define NO_DISPLAY
#	include <stdint.h>
#	include <stdio.h>
	typedef uint8_t BYTE;
	typedef uint16_t WORD;
	typedef uint32_t DWORD;
	typedef int32_t LONG;
	typedef bool BOOLEAN;

#	ifdef __GNUC__
#		define BMP_ATTR_PACK __attribute__((packed)) __attribute__ ((aligned (2)))
#	else
#		define BMP_ATTR_PACK
#	endif // #ifdef __GNUC__

	typedef struct tagRGBQUAD {
		BYTE    rgbBlue;
		BYTE    rgbGreen;
		BYTE    rgbRed;
		BYTE    rgbReserved;
	} BMP_ATTR_PACK RGBQUAD;

	typedef struct tagBITMAPINFOHEADER {
		DWORD  biSize;
		LONG   biWidth;
		LONG   biHeight;
		WORD   biPlanes;
		WORD   biBitCount;
		DWORD  biCompression;
		DWORD  biSizeImage;
		LONG   biXPelsPerMeter;
		LONG   biYPelsPerMeter;
		DWORD  biClrUsed;
		DWORD  biClrImportant;
	} BMP_ATTR_PACK BITMAPINFOHEADER, *PBITMAPINFOHEADER;

	typedef struct tagBITMAPFILEHEADER {
		WORD    bfType;
		DWORD   bfSize;
		WORD    bfReserved1;
		WORD    bfReserved2;
		DWORD   bfOffBits;
	} BMP_ATTR_PACK BITMAPFILEHEADER, *PBITMAPFILEHEADER;
#else
#	undef NO_DISPLAY
#endif
//-----------------------------------------------------------------------------
int SaveBMP( const string& filename, const char* pdata, int XSize, int YSize, int pitch, int bitsPerPixel )
//------------------------------------------------------------------------------
{
	static const WORD PALETTE_ENTRIES = 256;

	if( pdata )
	{
		FILE* pFile = fopen( filename.c_str(), "wb" );
		if( pFile )
		{
			BITMAPINFOHEADER	bih;
			BITMAPFILEHEADER	bfh;
			WORD				linelen = static_cast<WORD>( ( XSize * bitsPerPixel + 31 ) / 32 * 4 );  // DWORD aligned
			int					YPos;
			int					YStart = 0;

			memset( &bfh, 0, sizeof(BITMAPFILEHEADER) );
			memset( &bih, 0, sizeof(BITMAPINFOHEADER) );
			bfh.bfType          = 0x4d42;
			bfh.bfSize          = sizeof(bih) + sizeof(bfh) + sizeof(RGBQUAD)*PALETTE_ENTRIES + static_cast<LONG>(linelen) * static_cast<LONG>(YSize);
			bfh.bfOffBits       = sizeof(bih) + sizeof(bfh) + sizeof(RGBQUAD)*PALETTE_ENTRIES;
			bih.biSize          = sizeof(bih);
			bih.biWidth         = XSize;
			bih.biHeight        = YSize;
			bih.biPlanes        = 1;
			bih.biBitCount      = static_cast<WORD>(bitsPerPixel);
			bih.biSizeImage     = static_cast<DWORD>(linelen) * static_cast<DWORD>(YSize);

			if( ( fwrite( &bfh, sizeof(bfh), 1, pFile ) == 1 ) && ( fwrite( &bih, sizeof(bih), 1, pFile ) == 1 ) )
			{
				RGBQUAD rgbQ;
				for( int i=0; i<PALETTE_ENTRIES; i++ )
				{
					rgbQ.rgbRed      = static_cast<BYTE>(i);
					rgbQ.rgbGreen    = static_cast<BYTE>(i);
					rgbQ.rgbBlue     = static_cast<BYTE>(i);
					rgbQ.rgbReserved = static_cast<BYTE>(0);
					fwrite( &rgbQ, sizeof(rgbQ), 1, pFile );
				}

				for( YPos = YStart+YSize-1; YPos>=YStart; YPos-- )
				{
					if( fwrite( &pdata[YPos*pitch], linelen, 1, pFile ) != 1 )
					{
						cout << "SaveBmp: ERR_WRITE_FILE: " << filename << endl;
					}
				}
			}
			else
			{
				cout << "SaveBmp: ERR_WRITE_FILE: " << filename << endl;
			}
			fclose(pFile);
		}
		else
		{
			cout << "SaveBmp: ERR_CREATE_FILE: " << filename << endl;
		}
	}
	else
	{
		cout << "SaveBmp: ERR_DATA_INVALID:" << filename << endl;
	}
	return 0;
}


class BluefoxCam{
public:
	BluefoxCam();
	~BluefoxCam();
	void Spin();
private:
	// other functions
	bool Initialize();
	void LiveLoop(bool boStoreFrames, bool boSingleShotMode);

	// parameters
	bool boStoreFrames;
	int deviceNum;
	int width;
	int height;
	int defaultRequestCount;
	string pixelFormat;
	string acquisitionMode;
	int blackWhite;

	// device drive
	DeviceManager devMgr;
	Device* pDev;

	// ros
	ros::NodeHandle nh;
	image_transport::CameraPublisher image_pub_;	
	sensor_msgs::Image img_;
	sensor_msgs::CameraInfo info_;
};

BluefoxCam::BluefoxCam()
{
	image_transport::ImageTransport it(nh);
    	image_pub_ = it.advertiseCamera("image_raw", 1);

	// load the parameters
    	nh.param("device_num", deviceNum, 0);
	nh.param("image_width", width, 640);
	nh.param("image_height", height, 480);
	nh.param("image_blackwhite", blackWhite, 0);
	//nh.param("pixel_format", pixelFormat, std::string("ibpfYUV422Packed"));
	//nh.param("pixel_format", pixelFormat, std::string("mjpeg")); // possible values: yuyv, uyvy, mjpeg
	//nh.param("autofocus", autofocus_, false); // enable/disable autofocus

	// default value	
	boStoreFrames=true;
	defaultRequestCount = -1;
	pDev = 0;

	info_.height = height;
 	info_.width = width;
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
				<< "(error code: " << e.getErrorCode() << ", " << e.getErrorCodeAsString() << "). Press any key to end the application..." << endl;
			//PRESS_A_KEY_AND_RETURN
			return false;
		}		
	}
	return true;
}
//-----------------------------------------------------------------------------
void BluefoxCam::LiveLoop(bool boStoreFrames, bool boSingleShotMode )
//-----------------------------------------------------------------------------
{
	cout << " == " << __FUNCTION__ << " - establish access to the statistic properties...." << endl;
	// establish access to the statistic properties
	Statistics statistics( pDev );
	cout << " == " << __FUNCTION__ << " - create an interface to the device found...." << endl;
	// create an interface to the device found
	FunctionInterface fi( pDev );

#if 0
	// if running mvBlueFOX on an embedded system (e.g. ARM) with USB 1.1 it may be necessary to change
	// a few settings and timeouts like this:

	// get other settings
	SettingsBlueFOX setting( pDev );

	// set request timeout higher because USB 1.1 on ARM is soooo slow
	setting.cameraSetting.imageRequestTimeout_ms.write( 5000 );
	// use on Demand mode
	setting.cameraSetting.triggerMode.write( ctmOnDemand );
#endif

#if 0
	// this section contains special settings that might be interesting for mvBlueCOUGAR or mvBlueLYNX-M7
	// related embedded devices
	CameraSettingsBlueCOUGAR cs(pDev);
	int maxWidth = cs.aoiWidth.getMaxValue();
	cs.aoiWidth.write( maxWidth );
	//cs.autoGainControl.write( agcOff );
	//cs.autoExposeControl.write( aecOff );
	//cs.exposeMode.write( cemOverlapped );
	//cs.pixelClock_KHz.write( cpc40000KHz );
	//cs.expose_us.write( 5000 );
#endif

	// If this is color sensor, we will NOT convert the Bayer data into a RGB image as this
	// will cost a lot of time on an embedded system


	ImageProcessing ip(pDev);
	if( (bool)blackWhite && ip.colorProcessing.isValid() )
	{
		ip.colorProcessing.write( cpmRaw );
	}

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
	cout << "Press <<ENTER>> to end the application!!" << endl;

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
				++cnt;
				fillImage(img_, "bgra8", pRequest->imageHeight.read(), pRequest->imageWidth.read(), 4 *pRequest->imageWidth.read(), pRequest->imageData.read());
				image_pub_.publish(img_, info_);

				// here we can display some statistical information every 100th image
				if( cnt%100 == 0 )
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
					if( boStoreFrames )
					{
						char filename[200];
						sprintf(filename,"/home/leng/ros/projects/bluefox_cam/output/single_%d.bmp",cnt);
						cout << "Storing the image as \"" << filename << "\"" << endl;
						SaveBMP( filename, reinterpret_cast<char*>(pRequest->imageData.read()), pRequest->imageWidth.read(), pRequest->imageHeight.read(), pRequest->imageLinePitch.read(), pRequest->imagePixelPitch.read()*8 );
					}
				}
			}
			else
			{
				cout << "*** Error: A request has been returned with the following result: " << pRequest->requestResult << endl;
			}

			// this image has been displayed thus the buffer is no longer needed...
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

	//if( boManualAcquisitionEngineControl && !boSingleShotMode )
	//{
	//	const int stopResult = fi.acquisitionStop();
	//	cout << "Manually stopping acquisition engine. Result: " << stopResult << "("
	//		<< ImpactAcquireException::getErrorCodeAsString( stopResult ) << ")" << endl;
	//}
	//cout << " == " << __FUNCTION__ << " - free resources...." << endl;

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
	else
	{
		ROS_INFO("Initialize() is going well");
	}
	ROS_INFO("End of Initialize()");

	LiveLoop( boStoreFrames, acquisitionMode == "SingleFrame" );
}
// main
int main(int argc, char** argv){
	ros::init(argc, argv, "bluefox_cam");
	ROS_INFO("Bluefox_cam started");
	BluefoxCam bc;
	bc.Spin();
	return 0;
}
