#include <stdio.h>
#include <atlstr.h> 
#include <dshow.h> 
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\core\core.hpp>
#include <opencv\cv.h>
#include <iostream>

CFactoryTemplate g_Templates[1];
int g_cTemplates;



void setCameraMode(ICaptureGraphBuilder2 *pCaptureGraphBuilder2, IAMStreamConfig *pConfig, IBaseFilter *pDeviceFilter, HRESULT hr)
{

	// Set res, frame rate, and color mode 

	hr = CoInitialize(0);

	hr = pCaptureGraphBuilder2->FindInterface(&PIN_CATEGORY_CAPTURE, 0, pDeviceFilter, IID_IAMStreamConfig, (void**)&pConfig);


	int iCount = 0, iSize = 0;

	hr = pConfig->GetNumberOfCapabilities(&iCount, &iSize);

	// Check the size to make sure we pass in the correct structure.

	if (iSize == sizeof(VIDEO_STREAM_CONFIG_CAPS))
	{

		// Use the video capabilities structure. 

		for (int iFormat = 0; iFormat < iCount; iFormat++)
		{

			VIDEO_STREAM_CONFIG_CAPS scc;
			AM_MEDIA_TYPE *pmtConfig;
			hr = pConfig->GetStreamCaps(iFormat, &pmtConfig, (BYTE*)&scc);

			if (SUCCEEDED(hr))
			{
				if ((pmtConfig->majortype == MEDIATYPE_Video)) //&& 
															   //(pmtConfig->subtype == MEDIASUBTYPE_RGB24))
				{

					VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)pmtConfig->pbFormat;

					// pVih contains the detailed format information. 

					LONG lWidth = pVih->bmiHeader.biWidth;
					LONG lHeight = pVih->bmiHeader.biHeight;
					pVih->bmiHeader.biWidth = 160;
					pVih->bmiHeader.biHeight = 120;
					pVih->bmiHeader.biSizeImage = DIBSIZE(pVih->bmiHeader);

					//  pVih->AvgTimePerFrame = 10000000;
				}
			}

			hr = pConfig->SetFormat(pmtConfig);
			hr = pConfig->GetStreamCaps(iFormat, &pmtConfig, (BYTE*)&scc);
			//DeleteMediaType(pmtConfig);
		}

	}


}





void setCameraControl(IBaseFilter *pDeviceFilter, HRESULT hr, int exposure, int focus)
{

	// Query the capture filter for the IAMCameraControl interface. 

	IAMCameraControl *pCameraControl = 0;
	hr = pDeviceFilter->QueryInterface(IID_IAMCameraControl, (void**)&pCameraControl);
	if (FAILED(hr))
	{

		// The device does not support IAMCameraControl 

	}

	else
	{

		long Min, Max, Step, Default, Flags, Val;

		// Get the range and default values 

		hr = pCameraControl->GetRange(CameraControl_Exposure, &Min, &Max, &Step, &Default, &Flags);
		hr = pCameraControl->GetRange(CameraControl_Focus, &Min, &Max, &Step, &Default, &Flags);

		if (SUCCEEDED(hr))
		{
			hr = pCameraControl->Set(CameraControl_Exposure, -10, CameraControl_Flags_Manual);

			// Min = -11, Max = 1, Step = 1 

			hr = pCameraControl->Set(CameraControl_Focus, focus, CameraControl_Flags_Manual);
		}

	}

}





void setCameraProperties(IBaseFilter *pDeviceFilter, HRESULT hr, int brightness, int backLightCompensation, int contrast, int saturation, int sharpness, int whiteBalance)
{

	// Query the capture filter for the IAMVideoProcAmp interface. 

	IAMVideoProcAmp *pProcAmp = 0;
	hr = pDeviceFilter->QueryInterface(IID_IAMVideoProcAmp, (void**)&pProcAmp);
	if (FAILED(hr))
	{

		// The device does not support IAMVideoProcAmp 

	}

	else
	{

		long Min, Max, Step, Default, Flags, Val;

		// Get the range and default values 

		hr = pProcAmp->GetRange(VideoProcAmp_Brightness, &Min, &Max, &Step, &Default, &Flags);
		hr = pProcAmp->GetRange(VideoProcAmp_BacklightCompensation, &Min, &Max, &Step, &Default, &Flags);
		hr = pProcAmp->GetRange(VideoProcAmp_Contrast, &Min, &Max, &Step, &Default, &Flags);
		hr = pProcAmp->GetRange(VideoProcAmp_Saturation, &Min, &Max, &Step, &Default, &Flags);
		hr = pProcAmp->GetRange(VideoProcAmp_Sharpness, &Min, &Max, &Step, &Default, &Flags);
		hr = pProcAmp->GetRange(VideoProcAmp_WhiteBalance, &Min, &Max, &Step, &Default, &Flags);

		if (SUCCEEDED(hr))
		{
			hr = pProcAmp->Set(VideoProcAmp_Brightness, 100, VideoProcAmp_Flags_Manual);
			hr = pProcAmp->Set(VideoProcAmp_BacklightCompensation, 0, VideoProcAmp_Flags_Manual);
			hr = pProcAmp->Set(VideoProcAmp_Contrast, 20, VideoProcAmp_Flags_Manual);
			hr = pProcAmp->Set(VideoProcAmp_Saturation, 50, VideoProcAmp_Flags_Manual);
			hr = pProcAmp->Set(VideoProcAmp_Sharpness, 0, VideoProcAmp_Flags_Manual);
			hr = pProcAmp->Set(VideoProcAmp_WhiteBalance, 0, VideoProcAmp_Flags_Manual);
		}

	}

}





//given in the example program

IPin *GetPin(IBaseFilter *pFilter, PIN_DIRECTION PinDir)
{
	BOOL       bFound = FALSE;
	IEnumPins  *pEnum;
	IPin       *pPin;

	pFilter->EnumPins(&pEnum);
	while (pEnum->Next(1, &pPin, 0) == S_OK)
	{
		PIN_DIRECTION PinDirThis;
		pPin->QueryDirection(&PinDirThis);
		if (bFound = (PinDir == PinDirThis))
			break;
		pPin->Release();
	}
	pEnum->Release();
	return (bFound ? pPin : 0);
}





int main()
{
	//  for playing 
	IGraphBuilder *pGraphBuilder;
	ICaptureGraphBuilder2 *pCaptureGraphBuilder2;
	IMediaControl *pMediaControl = NULL;
	IMediaEventEx *pEvent = NULL;
	//  multiple cameras 
	IBaseFilter *pDeviceFilter_0 = NULL;
	IBaseFilter *m_pGrabber_0 = NULL;
	ISampleGrabber *m_pGrabberSettings_0 = NULL;


	//  select camera 
	ICreateDevEnum *pCreateDevEnum = NULL;
	IEnumMoniker *pEnumMoniker = NULL;
	IMoniker *pMoniker = NULL;
	ULONG nFetched = 0;
	// initialize COM 
	CoInitialize(NULL);

	// selecting a device 
	// Create CreateDevEnum to list device 
	std::string USB1 = "\\\\?\\usb#vid_045e&pid_076d&mi_00#7&1ba27d43&0&0000#{65e8773d-8f56-11d0-a3b9-00a0c9223196}\\global";


	CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_ICreateDevEnum, (PVOID *)&pCreateDevEnum);

	// Create EnumMoniker to list VideoInputDevice 
	pCreateDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnumMoniker, 0);
	if (pEnumMoniker == NULL) {
		// this will be shown if there is no capture device 
		printf("no device\n");
		return 0;
	}

	// reset EnumMoniker 
	pEnumMoniker->Reset();

	// get each Moniker 
	while (pEnumMoniker->Next(1, &pMoniker, &nFetched) == S_OK)
	{
		IPropertyBag *pPropertyBag;
		TCHAR devname[256];
		TCHAR devpath[256];

		// bind to IPropertyBag 
		pMoniker->BindToStorage(0, 0, IID_IPropertyBag, (void **)&pPropertyBag);

		VARIANT var;

		// get FriendlyName 
		var.vt = VT_BSTR;
		pPropertyBag->Read(L"FriendlyName", &var, 0);
		WideCharToMultiByte(CP_ACP, 0, var.bstrVal, -1, devname, sizeof(devname), 0, 0);
		VariantClear(&var);

		// get DevicePath 
		// DevicePath : A unique string 

		var.vt = VT_BSTR;

		pPropertyBag->Read(L"DevicePath", &var, 0);

		WideCharToMultiByte(CP_ACP, 0, var.bstrVal, -1, devpath, sizeof(devpath), 0, 0);

		std::string devpathString = devpath;

		pMoniker->BindToObject(0, 0, IID_IBaseFilter, (void**)&pDeviceFilter_0);


		pMoniker->Release();
		pPropertyBag->Release();

		if (pDeviceFilter_0 == NULL)
		{
			MessageBox(NULL, "No MS HD-5000 cameras found", "No cameras", MB_OK);
			return 0;
		}

	}

	// create FilterGraph and CaptureGraphBuilder2 
	CoCreateInstance(CLSID_FilterGraph, NULL, CLSCTX_INPROC, IID_IGraphBuilder, (LPVOID *)&pGraphBuilder);
	CoCreateInstance(CLSID_CaptureGraphBuilder2, NULL, CLSCTX_INPROC, IID_ICaptureGraphBuilder2, (LPVOID *)&pCaptureGraphBuilder2);

	HRESULT hr = CoInitialize(0);
	IAMStreamConfig *pConfig = NULL;
	setCameraMode(pCaptureGraphBuilder2, pConfig, pDeviceFilter_0, hr);  //  FPS, Res, color mode 
	setCameraControl(pDeviceFilter_0, hr, 10, 12); //  Focus, exposure 
	setCameraProperties(pDeviceFilter_0, hr, 180, 0, 4, 100, 0, 2800);  //  Brightness, saturation, etc 



																		//  set grabber properties 
	AM_MEDIA_TYPE mt;
	hr = CoCreateInstance(CLSID_SampleGrabber, NULL, CLSCTX_INPROC_SERVER, IID_IBaseFilter, (void**)&m_pGrabber_0);   // create ISampleGrabber 
	pCaptureGraphBuilder2->SetFiltergraph(pGraphBuilder);   // set FilterGraph 
	pGraphBuilder->QueryInterface(IID_IMediaControl, (LPVOID *)&pMediaControl); // get MediaControl interface 

	m_pGrabber_0->QueryInterface(IID_ISampleGrabber, (void**)&m_pGrabberSettings_0);

	ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));
	mt.majortype = MEDIATYPE_Video;
	mt.subtype = MEDIASUBTYPE_RGB24;
	hr = m_pGrabberSettings_0->SetMediaType(&mt);

	if (FAILED(hr))
	{
		return hr;
	}
	hr = m_pGrabberSettings_0->SetOneShot(FALSE);
	hr = m_pGrabberSettings_0->SetBufferSamples(TRUE);


	//  build filter graph 
	pGraphBuilder->AddFilter(pDeviceFilter_0, L"Device Filter");
	pGraphBuilder->AddFilter(m_pGrabber_0, L"Sample Grabber");
	IPin* pSourceOut_0 = GetPin(pDeviceFilter_0, PINDIR_OUTPUT);
	IPin* pGrabberIn_0 = GetPin(m_pGrabber_0, PINDIR_INPUT);
	pGraphBuilder->Connect(pSourceOut_0, pGrabberIn_0);

	/*
	pMediaControl->Run();
	long pBufferSize;
	unsigned char* pBuffer_0 = 0;
	hr = m_pGrabberSettings_0->GetCurrentBuffer(&pBufferSize, NULL);
	if (FAILED(hr))
	{
	return 0;
	}

	pBuffer_0 = (BYTE*)CoTaskMemAlloc(pBufferSize);

	if (!pBuffer_0)
	{
	hr = E_OUTOFMEMORY;
	return 0;
	}


	long pBufferSize = 0;
	unsigned char* pBuffer_0 = 0;
	long Size=0;
	hr = m_pGrabberSettings_0->GetCurrentBuffer(&Size, NULL);

	if (Size != pBufferSize)
	{
	pBufferSize = Size;

	if (pBuffer_0 != 0)
	{
	delete[] pBuffer_0;
	}

	pBuffer_0= new unsigned char[pBufferSize];
	}


	long pBufferSize = 425;
	unsigned char* pBuffer_0 = 0;

	pBuffer_0 = new unsigned char[pBufferSize];



	// start playing
	pMediaControl->Run();

	while (1)   {

	if (MessageBox(NULL, "Grab frame?", "Grab?", MB_OKCANCEL) == 2)
	{
	break;
	}

	hr = m_pGrabberSettings_0->GetCurrentBuffer(&pBufferSize,(long*)pBuffer_0);

	Cleanup:

	//  convert to OpenCV format

	IplImage* img_0 = cvCreateImage(cvSize(160,120),IPL_DEPTH_8U,3);

	for (int i = 0; i < pBufferSize ; i++)
	{
	img_0->imageData[i] = pBuffer_0[i];
	}

	cvFlip(img_0, NULL, 0);



	//  show
	// cvNamedWindow("mainWin_0", CV_WINDOW_AUTOSIZE);
	// cvMoveWindow("mainWin_0", 100, 100);
	cvShowImage("mainWin_0", img_0 );
	cvSaveImage("c:\\users\\senthil\\desktop\\img.png",img_0 );

	//cvWaitKey(0);

	cvReleaseImage(&img_0 );

	}

	*/


	pMediaControl->Run();
	cvNamedWindow("Camera_Output", 1);    //Create window
	CvCapture* capture = cvCaptureFromCAM(0);  //Capture using any camera connected to your system
	while (1)
	{
		//Create infinte loop for live streaming
		if (MessageBox(NULL, "Grab frame?", "Grab?", MB_OKCANCEL) == 2)
		{
			break;
		}

		IplImage* frame = cvQueryFrame(capture); //Create image frames from capture
		cvShowImage("Camera_Output", frame);     //Show image frames on created window

		cvSaveImage("c:\\users\\senthil\\desktop\\img1.png", frame);


		//  cv::Mat img(frame);
		//  cv::imwrite("c:\\users\\selvaraj\\desktop\\img.png",img);

	}

	//std::cout << "FPS: " << fps << std::endl;
	//std::cout << "PROP_BRIGHTNESS: " << PROP_BRIGHTNESS << std::endl;
	//WriteComPort("COM3","A");
	cvReleaseCapture(&capture); //Release capture.
	cvDestroyWindow("Camera_Output"); //Destroy Window                      */


									  //  release 
	pMediaControl->Release();
	pCaptureGraphBuilder2->Release();
	pGraphBuilder->Release();
	pEnumMoniker->Release();
	pCreateDevEnum->Release();

	// finalize COM 
	CoUninitialize();

	return 0;
}