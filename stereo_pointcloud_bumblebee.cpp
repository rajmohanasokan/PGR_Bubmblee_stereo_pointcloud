//=============================================================================
// Author: Rajmohan Asokan
// Affiliation: Control, Dynamics and Estimation Lab, University at Buffalo
// Note: Modified from the sample program provided in the Point Grey Research SDK
//=============================================================================
//=============================================================================
// stereo_pointcloud_bumblebee
//
// Takes input from a Bumblebee and performs subpixel
// interpolation to create a 16-bit disparity image, which is saved.
// The disparity data is then converted to 3-dimensional X/Y/Z
// coordinates which is written as point cloud data to a file. 
//
// This point cloud file can be viewed with Matlab's showPointCloud() function, PCL functions or OpenGL functions
//
//=============================================================================

#include "triclops.h"
#include "fc2triclops.h"
#include <stdio.h>
#include <stdlib.h>
#include "PGRFlyCapture.h"


//
// Macro to check, report on, and handle Triclops API error codes.
//
#define _HANDLE_TRICLOPS_ERROR( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 triclopsErrorToString( error ) ); \
      exit( 1 ); \
   } \
} \

// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

struct ImageContainer
{
    FC2::Image unprocessed[2];	
    FC2::Image bgru[2];
    FC2::Image mono[2];
    FC2::Image packed;
};

enum IMAGE_SIDE
{
	RIGHT = 0, LEFT
};

// configue camera to capture image
int configureCamera( FC2::Camera &camera );

// generate Triclops context from connected camera
int generateTriclopsContext( FC2::Camera     & camera, 
                             TriclopsContext & triclops );

// capture image from connected camera
int grabImage ( FC2::Camera & camera, FC2::Image & grabbedImage );

// convert image to BRGU
int convertToBGRU( FC2::Image & image, FC2::Image & convertedImage );

// generate triclops input necessary to carry out stereo processing
int generateTriclopsInput( FC2::Image const & grabbedImage, 							
                           ImageContainer   & imageContainer,
                           TriclopsInput    & colorData,
                           TriclopsInput    & stereoData,
						   TriclopsContext const & triclops,
						   TriclopsImage16      & depthImage );


// save 3d points generated from stereo processing
int save3dPoints( FC2::Image      const & grabbedImage, 
                  TriclopsContext const & triclops, 
                  TriclopsImage16 const & disparityImage16, 
                  TriclopsInput   const & colorData );



int main( int /* argc */, char** /* argv */ ){
	
	int blValue;
	FlyCaptureContext flycap; 
	FlyCaptureError fe;
	flycaptureCreateContext(&flycap);printf("\n Success \n");
	printf("flycaptureCreateContext():%s\n",flycaptureErrorToString(fe));
	fe = flycaptureInitializeFromSerialNumber(flycap,11360482);//Serial Number of the camera hardware; can be used to fetch calibration details
	printf("flycaptureInitializeFromSerialNumber():%s\n",flycaptureErrorToString(fe));
	// Defining variables for Camera configuration
	bool              pbPresent;
	long              plMin;
	long              plMax;
	long              plDefault;
	bool              pbAuto;
	bool			   pbManual; 
	long               plValueA;
	long               plValueB;
	bool               pbAuto1;
	fe = flycaptureGetCameraPropertyRange(
				 flycap,
				 FLYCAPTURE_BRIGHTNESS,
				 &pbPresent,
				 &plMin,
				 &plMax,
				 &plDefault,
				 &pbAuto,
				 &pbManual);
	if(pbPresent==true && pbManual==true){
		
		blValue=1;
		fe = flycaptureSetCameraProperty(
			    flycap,
			    FLYCAPTURE_WHITE_BALANCE,
			    1000,
			    1000,
			    true);
		fe = flycaptureSetCameraProperty(
			    flycap,
			    FLYCAPTURE_BRIGHTNESS,
			    1023,
			    0,
			    true);
		fe = flycaptureGetCameraProperty(
			    flycap,
			    FLYCAPTURE_WHITE_BALANCE,
				&plValueA,
				&plValueB,
				&pbAuto1);
	}
	else if(pbPresent==true){
		
		blValue=1.5;
	}
	else if(pbPresent==false){
		
		blValue=0;
	}
	else{
		
		blValue=5;
	}
	printf("flycaptureGetCameraPropertyRange():%s\n",flycaptureErrorToString(fe));
	printf("Present:%d\tMin:%lu\t Max:%lu\n",blValue,plMin,plMax);
	printf("plValueA:%lu\tplValueB:%lu\n",plValueA,plValueB);
	
	// Defining Triclops variables
    TriclopsInput triclopsColorInput, triclopsMonoInput;
    TriclopsContext triclops;
	TriclopsImage16      depthImage;
    FC2::Camera camera;
    FC2::Image grabbedImage;
	FC2::Image convertedImage;

    camera.Connect();// Initiate Camera connect

    // configure camera
    if ( configureCamera( camera )){
		return EXIT_FAILURE;
    }

    // generate the Triclops context 
    if ( generateTriclopsContext( camera, triclops ) ){
		return EXIT_FAILURE;
    }
	
    // grab image from camera.
    // this image contains both right and left images
    if ( grabImage( camera, grabbedImage ) ){
		return EXIT_FAILURE;
    }

    // Container of Images used for processing
    ImageContainer imageContainer;

    // generate triclops inputs from grabbed image
    if ( generateTriclopsInput( grabbedImage, 
								imageContainer,
                                triclopsColorInput, 
                                triclopsMonoInput,
								triclops,
								depthImage) 
       ){
		
		return EXIT_FAILURE;
    }

    // output image disparity image with subpixel interpolation
    TriclopsImage16 disparityImage16;

    // carry out the stereo pipeline 


    // save text file containing 3d points
    if ( save3dPoints( grabbedImage, triclops, depthImage, triclopsColorInput ) )
    {
		return EXIT_FAILURE;
    }

    // Close the camera and disconnect
    camera.StopCapture();
    camera.Disconnect();
   
    // Destroy the Triclops context
    TriclopsError te;
    te = triclopsDestroyContext( triclops ) ;
    _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", te );
         
    return 0;   
}


int configureCamera( FC2::Camera & camera ){
	
	FC2T::ErrorType fc2TriclopsError;	      
	FC2T::StereoCameraMode mode = FC2T::TWO_CAMERA_NARROW;
    fc2TriclopsError = FC2T::setStereoMode( camera, mode );
    if ( fc2TriclopsError ){
		
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, "setStereoMode");
    }


    return 0;
}


int grabImage ( FC2::Camera & camera, FC2::Image& grabbedImage ){
	
	FC2::Error fc2Error = camera.StartCapture();
	if (fc2Error != FC2::PGRERROR_OK){
		
		return FC2T::handleFc2Error(fc2Error);
	}

	fc2Error = camera.RetrieveBuffer(&grabbedImage);
	if (fc2Error != FC2::PGRERROR_OK){
		
		return FC2T::handleFc2Error(fc2Error);
	}
	
	return 0;
}


int generateTriclopsContext( FC2::Camera     & camera, 
                             TriclopsContext & triclops ){
								 
	/*
	FC2::CameraInfo camInfo;
    FC2::Error fc2Error = camera.GetCameraInfo(&camInfo);
	if (fc2Error != FC2::PGRERROR_OK){
		
		return FC2T::handleFc2Error(fc2Error);
	}
   
	FC2T::ErrorType fc2TriclopsError; 
    fc2TriclopsError = FC2T::getContextFromCamera( camInfo.serialNumber, &triclops );
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK){
		
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, 
		                                    "getContextFromCamera");
    }
	*/
	char * pCalibration = "bumblebee11360482-current.cal"; //If the calibration file is readily available; use it directly
	TriclopsError tc;

	tc = triclopsGetDefaultContextFromFile( &triclops, pCalibration);
	_HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", tc );

	
	return 0;
}

int convertToBGRU( FC2::Image & image, FC2::Image & convertedImage ){
	
    FC2::Error fc2Error;
    fc2Error = image.SetColorProcessing(FC2::HQ_LINEAR);
    if (fc2Error != FC2::PGRERROR_OK){
		
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = image.Convert(FC2::PIXEL_FORMAT_BGRU, &convertedImage);
    if (fc2Error != FC2::PGRERROR_OK){
		
        return FC2T::handleFc2Error(fc2Error);
    }

    return 0;
}

int generateTriclopsInput( FC2::Image const & grabbedImage,
						   ImageContainer  & imageContainer,
                            TriclopsInput   & triclopsColorInput,
                            TriclopsInput   & triclopsMonoInput,
							TriclopsContext const & triclops, 
							TriclopsImage16      & depthImage ){
								
    FC2::Error fc2Error;
    FC2T::ErrorType fc2TriclopsError; 
    TriclopsError te;

    FC2::Image * unprocessedImage = imageContainer.unprocessed;

    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
                            grabbedImage, 
                            true /*assume little endian*/,
                            unprocessedImage[RIGHT], 
                            unprocessedImage[LEFT]);

    if (fc2TriclopsError != FC2T::ERRORTYPE_OK){
		
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, 
                                     "unpackUnprocessedRawOrMono16Image");
    }

    FC2::PGMOption pgmOpt;
    pgmOpt.binaryFile = true;
    unprocessedImage[RIGHT].Save("rawRightImage.pgm", &pgmOpt);
    unprocessedImage[LEFT].Save("rawLeftImage.pgm", &pgmOpt);

    FC2::Image * monoImage = imageContainer.mono;
	FC2::Image * bgruImage = imageContainer.bgru;

    // check if the unprocessed image is color
    if ( unprocessedImage[RIGHT].GetBayerTileFormat() != FC2::NONE ){
       

        for ( int i = 0; i < 2; ++i ){
			
            if ( convertToBGRU(unprocessedImage[i], bgruImage[i]) ){
				
                return 1;
            }
        }

        FC2::PNGOption pngOpt;
        pngOpt.interlaced = false;
        pngOpt.compressionLevel = 9;
        bgruImage[RIGHT].Save("colorImageRight.png", &pngOpt);
        bgruImage[LEFT].Save("colorImageLeft.png", &pngOpt);

        FC2::Image & packedColorImage = imageContainer.packed;

        // pack BGRU right and left image into an image
        fc2TriclopsError = FC2T::packTwoSideBySideRgbImage(bgruImage[RIGHT], 
                                                            bgruImage[LEFT],
                                                            packedColorImage);

        if (fc2TriclopsError != FC2T::ERRORTYPE_OK){
			
            return handleFc2TriclopsError(fc2TriclopsError, 
                                            "packTwoSideBySideRgbImage");
        }

        // Use the row interleaved images to build up a packed TriclopsInput.
        // A packed triclops input will contain a single image with 32 bpp.
        te = triclopsBuildPackedTriclopsInput( grabbedImage.GetCols(),
                                                grabbedImage.GetRows(),
                                                packedColorImage.GetStride(),
                                                (unsigned long)grabbedImage.GetTimeStamp().seconds, 
                                                (unsigned long)grabbedImage.GetTimeStamp().microSeconds, 
                                                packedColorImage.GetData(),
                                                &triclopsColorInput );

        _HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );


        // the following does not change the size of the image
        // and therefore it PRESERVES the internal buffer!
        packedColorImage.SetDimensions( packedColorImage.GetRows(), 
                                        packedColorImage.GetCols(), 
                                        packedColorImage.GetStride(),
                                        packedColorImage.GetPixelFormat(),
                                        FC2::NONE);

        packedColorImage.Save("packedColorImage.png",&pngOpt );

        for ( int i = 0; i < 2; ++i ){
			
            fc2Error = bgruImage[i].Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &monoImage[i]);
            if (fc2Error != FlyCapture2::PGRERROR_OK){
				
                return Fc2Triclops::handleFc2Error(fc2Error);
            }
        }

        monoImage[RIGHT].Save("monoImageRight.pgm", &pgmOpt);
        monoImage[LEFT].Save("monoImageLeft.pgm", &pgmOpt);
    }
    else{
		
        monoImage[RIGHT] = unprocessedImage[RIGHT];
        monoImage[LEFT] = unprocessedImage[LEFT];
    }
   
    // Use the row interleaved images to build up an RGB TriclopsInput.  
    // An RGB triclops input will contain the 3 raw images (1 from each camera).
    te = triclopsBuildRGBTriclopsInput( grabbedImage.GetCols(), 
                                        grabbedImage.GetRows(), 
                                        grabbedImage.GetCols(),  
                                        (unsigned long)grabbedImage.GetTimeStamp().seconds, 
                                        (unsigned long)grabbedImage.GetTimeStamp().microSeconds, 
                                        monoImage[RIGHT].GetData(), 
                                        monoImage[LEFT].GetData(), 
                                        monoImage[LEFT].GetData(), 
                                        &triclopsMonoInput );

    _HANDLE_TRICLOPS_ERROR( "triclopsBuildRGBTriclopsInput()", te );


  
	TriclopsImage rectifiedImageL;
	TriclopsImage rectifiedImageR;
	//TriclopsImage disp;

	
	te = triclopsSetResolutionAndPrepare(triclops, 
				 600,
				 800,
				 960,
				 1280);
	printf("triclopsSetResolutionAndPrepare(): %s\n",triclopsErrorToString(te));

	//te=triclopsSetResolution( triclops, 384, 512);
	//printf("triclopsSetResolution(): %s\n",triclopsErrorToString(te));
    
	// Set subpixel interpolation on to use 
    // TriclopsImage16 structures when we access and save the disparity image
    te = triclopsSetSubpixelInterpolation( triclops, 1 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te );

	te = triclopsSetRectImgQuality(triclops, TriRectQlty_ENHANCED_2);
	_HANDLE_TRICLOPS_ERROR( "triclopsSetRectImgQuality()", te );

	te = triclopsSetLowpass( triclops,true);
	_HANDLE_TRICLOPS_ERROR( "triclopsSetLowpass()", te );

    // Rectify the images
    te = triclopsRectify( triclops, const_cast<TriclopsInput *>(&triclopsMonoInput) );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", te );



    // Retrieve the rectified_left image from the triclops context
    te = triclopsGetImage( triclops, 
                          TriImg_RECTIFIED, 
                          TriCam_LEFT, 
						  &rectifiedImageL );
    printf("triclopsGetImage(): %s\n",triclopsErrorToString(te));

	  // Retrieve the rectified_right image from the triclops context
    te = triclopsGetImage( triclops, 
                          TriImg_RECTIFIED, 
                          TriCam_RIGHT, 
						  &rectifiedImageR );
    printf("triclopsGetImage(): %s\n",triclopsErrorToString(te));

	const char * pRectifiedFilenameL = "left_rectified1.pgm";
    te = triclopsSaveImage( &rectifiedImageL, const_cast<char *>(pRectifiedFilenameL) );
    printf("triclopsSaveImage(): %s\n",triclopsErrorToString(te));

	const char * pRectifiedFilenameR = "right_rectified1.pgm";
    te = triclopsSaveImage( &rectifiedImageR, const_cast<char *>(pRectifiedFilenameR) );
    printf("triclopsSaveImage(): %s\n",triclopsErrorToString(te));



	//set the disparity range

	te = triclopsSetEdgeCorrelation( triclops,true );
	printf("triclopsSetEdgeCorrelation(): %s\n",triclopsErrorToString(te));

	te = triclopsSetEdgeMask( triclops,7);
	printf("triclopsSetEdgeMask(): %s\n",triclopsErrorToString(te));

	te=triclopsSetStereoMask( triclops, 11);
	printf("triclopsSetStereoMask(): %s\n",triclopsErrorToString(te));

	te=triclopsSetDisparity( triclops, 0, 240);
	printf("triclopsSetDisparity(): %s\n",triclopsErrorToString(te));

	//set texture validation On
	te=triclopsSetTextureValidation(triclops,true);
	printf("triclopsSetTextureValidation(): %s\n",triclopsErrorToString(te));

	te = triclopsSetTextureValidationThreshold( triclops, 0.20);
	printf("triclopsSetTextureValidationThreshold(): %s\n",triclopsErrorToString(te));

	te=triclopsSetUniquenessValidation(triclops,true);
	printf("triclopsSetUniquenessValidation(): %s\n",triclopsErrorToString(te));

	te = triclopsSetUniquenessValidationThreshold( triclops, 1.40);
	printf("triclopsSetUniquenessValidationThreshold(): %s\n",triclopsErrorToString(te));
	


	//set Uniqueness validation On
	te=triclopsSetSurfaceValidation( triclops,true);
	printf("triclopsSetSurfaceValidation(): %s\n",triclopsErrorToString(te));

	te = triclopsSetSurfaceValidationSize( triclops,200 );
	printf("triclopsSetSurfaceValidationSize(): %s\n",triclopsErrorToString(te));

	te = triclopsSetSurfaceValidationDifference(triclops,1.0);
	printf("triclopsSetSurfaceValidationDifference(): %s\n",triclopsErrorToString(te));

	te=triclopsSetBackForthValidation( triclops,true);
	printf("triclopsSetBackForthValidation(): %s\n",triclopsErrorToString(te));

    // Do stereo processing
	te = triclopsSetStereoQuality(triclops, TriStereoQlty_STANDARD);
	printf("triclopsSetStereoQuality(): %s\n",triclopsErrorToString(te));

    te = triclopsStereo( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", te );
   
    // Retrieve the interpolated depth image from the context
	te = triclopsGetImage16( triclops, 
                            TriImg16_DISPARITY, 
                            TriCam_REFERENCE, 
                            &depthImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage16()", te );
	
	
    // Save the interpolated depth image
	char const * pDispFilename = "disparity16.pgm";
    te = triclopsSaveImage16( &depthImage, const_cast<char *>(pDispFilename) );
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage16()", te );

    return 0;
}

int save3dPoints( FC2::Image      const & grabbedImage, 
                  TriclopsContext const & triclops, 
                  TriclopsImage16 const & disparityImage16, 
                  TriclopsInput   const & colorData ){
					  
    TriclopsImage monoImage = {0};
    TriclopsColorImage colorImage = {0};
    TriclopsError te;

    float            x, y, z; 
    int	            r, g, b;
    FILE             * pPointFile;
    int              nPoints = 0;
    int	             pixelinc ;
    int	             i, j, k;
    unsigned short * row;
    unsigned short   disparity;

    // Rectify the color image if applicable
	    bool isColor = false;
    if ( grabbedImage.GetPixelFormat() == FC2::PIXEL_FORMAT_RAW16 ){
		
        isColor = true;
        te = triclopsRectifyColorImage( triclops, 
                                        TriCam_REFERENCE, 
	                                    const_cast<TriclopsInput *>(&colorData), 
	                                    &colorImage );
        _HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", te );
    }
    else{
  
        te = triclopsGetImage( triclops,
	                            TriImg_RECTIFIED,
	                            TriCam_REFERENCE,
	                            &monoImage );
        _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );
	}

  
    // Save points to disk
    const char * pFilename = "out.pts";
    pPointFile = fopen( pFilename, "w+" );
    if ( pPointFile != NULL ){
        printf("Opening output file %s\n", pFilename);
    }
    else{
        printf("Error opening output file %s\n", pFilename);
        return 1;
    }

    // The format for the output file is:
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // ...

    // Determine the number of pixels spacing per row
    pixelinc = disparityImage16.rowinc/2;
    for ( i = 0, k = 0; i < disparityImage16.nrows; i++ ){
		
        row = disparityImage16.data + i * pixelinc;
        for ( j = 0; j < disparityImage16.ncols; j++, k++ ){
			
            disparity = row[j];

            // do not save invalid points
            if ( disparity < 0xFF00 ){
				
                // convert the 16 bit disparity value to floating point x,y,z
                triclopsRCD16ToXYZ( triclops, i, j, disparity, &x, &y, &z );

                // look at points within a range
                if ( z < 5.0 ){//Define the Z-range upto which the points should be extracted for 3D point cloud. Z-range from camera center(Left hand coordinate system)
                
                     if ( isColor ){
						 
                        r = (int)colorImage.red[k];
                        g = (int)colorImage.green[k];
                        b = (int)colorImage.blue[k];		  
                    }
                    else{
                        // For mono cameras, we just assign the same value to RGB
                        r = (int)monoImage.data[k];
                        g = (int)monoImage.data[k];
                        b = (int)monoImage.data[k];
					}

                    fprintf( pPointFile, "%f %f %f %d %d %d %d %d\n", x, y, z, r, g, b, i, j );
                    nPoints++;
                }
            }
        }
    }

    fclose( pPointFile );
    printf( "Points in file: %d\n", nPoints );

    return 0;

}

