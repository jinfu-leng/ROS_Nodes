#include <exampleHelper.h>

//-----------------------------------------------------------------------------
mvIMPACT::acquire::Device* getDeviceFromDeviceNum( const mvIMPACT::acquire::DeviceManager& devMgr, unsigned int devNr = 0, SUPPORTED_DEVICE_CHECK pSupportedDeviceCheckFn = 0, bool boSilent = false )
//-----------------------------------------------------------------------------
{
	const unsigned int devCnt = devMgr.deviceCount();
	if( devCnt == 0 )
	{
		std::cout << "No compatible device found!" << std::endl;
		return 0;
	}

	std::set<unsigned int> validDeviceNumbers;

	// checkout every device detected that matches
	for( unsigned int i=0; i<devCnt; i++ )
	{
		Device* pDev = devMgr[i];
		if( pDev )
		{
			if( !pSupportedDeviceCheckFn || pSupportedDeviceCheckFn( pDev ) )
			{
				switchFromGenericToGenICamInterface( pDev );
				validDeviceNumbers.insert( i );
			}
		}
	}

	if( validDeviceNumbers.empty() )
	{
		std::cout << devMgr.deviceCount() << " devices have been detected:" << std::endl;
		for( unsigned int i=0; i<devCnt; i++ )
		{
			Device* pDev = devMgr[i];
			if( pDev )
			{
				std::cout << "  [" << i << "]: " << pDev->serial.read() << " (" << pDev->product.read() << ", " << pDev->family << ")" << std::endl;
			}
		}
		std::cout << "However none of these devices seems to be supported by this sample." << std::endl << std::endl;
		return 0;
	}

	if( validDeviceNumbers.find( devNr ) == validDeviceNumbers.end() )
	{
		std::cout << "Invalid selection!" << std::endl;
		return 0;
	}

	if( !boSilent )
	{
		std::cout << "Using device number " << devNr << "." << std::endl;
	}
	return devMgr[devNr];
}
