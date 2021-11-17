//tabs=4
// --------------------------------------------------------------------------------
// TODO fill in this information for your driver, then remove this line!
//
// ASCOM Camera driver for RaspberryPiMTP
//
// Description:	This ASCOM driver is for a Raspberry Pi camera running the MTP firmware
//
// Implements:	ASCOM Camera interface version: <To be completed by driver developer>
// Author:		Ian Cass (ian@wheep.co.uk)
//
// Edit Log:
//
// Date			Who	Vers	Description
// -----------	---	-----	-------------------------------------------------------
// dd-mmm-yyyy	XXX	6.0.0	Initial edit, created from ASCOM driver template
// --------------------------------------------------------------------------------
//

#define Camera

using ASCOM;
using ASCOM.Astrometry;
using ASCOM.Astrometry.AstroUtils;
using ASCOM.DeviceInterface;
using ASCOM.Utilities;
using ASCOM.RaspberryPiMTP.Enums;
using CameraControl.Devices;
using CameraControl.Devices.Classes;
using CameraControl.Devices.Custom;
using CameraControl.Devices.TransferProtocol;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;
using System.Text;
using System.IO;
using System.Threading;
using PortableDeviceLib;

namespace ASCOM.RaspberryPiMTP
{
    //
    // Your driver's DeviceID is ASCOM.RaspberryPiMTP.Camera
    //
    // The Guid attribute sets the CLSID for ASCOM.RaspberryPiMTP.Camera
    // The ClassInterface/None attribute prevents an empty interface called
    // _RaspberryPiMTP from being created and used as the [default] interface
    //
    // TODO Replace the not implemented exceptions with code to implement the function or
    // throw the appropriate ASCOM exception.
    //

    /// <summary>
    /// ASCOM Camera Driver for RaspberryPiMTP.
    /// </summary>
    [Guid("61551797-6b3e-4630-afe6-71e13013f747")]
    [ClassInterface(ClassInterfaceType.None)]
    public class Camera : ICameraV3
    {
        /// <summary>
        /// ASCOM DeviceID (COM ProgID) for this driver.
        /// The DeviceID is used by ASCOM applications to load the driver at runtime.
        /// </summary>
        internal static string driverID = "ASCOM.RaspberryPiMTP.Camera";
        // TODO Change the descriptive string for your driver then remove this line
        /// <summary>
        /// Driver description that displays in the ASCOM Chooser.
        /// </summary>
        private static string driverDescription = "ASCOM Camera Driver for Raspberry Pi with MTP camera firmware.";

        internal static string traceStateProfileName = "Trace Level";
        internal static string traceStateDefault = "false";

        /// <summary>
        /// Private variable to hold the connected state
        /// </summary>
        private bool _connectedState;

        /// <summary>
        /// Private variable to hold an ASCOM Utilities object
        /// </summary>
        private Util utilities;

        /// <summary>
        /// Private variable to hold an ASCOM AstroUtilities object to provide the Range method
        /// </summary>
        private AstroUtils astroUtilities;
        private CameraDeviceManager DeviceManager;

        /// <summary>
        /// Variable to hold the trace logger object (creates a diagnostic log file with information that you specify)
        /// </summary>
        internal TraceLogger tl;

        private bool _connectionInProgress;
        private Array cameraImageArray;
        private ImageDataProcessor _imageDataProcessor;
        private short _imageFormat = 0x04;
        private short _currentGain = 1;
        private short _currentBin = 1;
        private bool _fastReadout = false;
        private CameraStates _cameraState = CameraStates.cameraIdle;
        private SensorType _sensorType = SensorType.RGGB;

        private ArrayList _gains = new ArrayList()
            {"100", "200", "300", "400", "500", "600", "700", "800", "900", "1000", "1100", "1200", "1300", "1400", "1500", "1600", "3200", "4800", "6400"};

        /// <summary>
        /// Initializes a new instance of the <see cref="RaspberryPiMTP"/> class.
        /// Must be public for COM registration.
        /// </summary>
        public Camera()
        {
            tl = new TraceLogger("", "RaspberryPiMTP");
            ReadProfile(); // Read device configuration from the ASCOM Profile store

            tl.LogMessage("Camera", "Starting initialisation");

            _connectedState = false; // Initialise connected to false
            utilities = new Util(); //Initialise util object
            astroUtilities = new AstroUtils(); // Initialise astro-utilities object
            //TODO: Implement your additional construction here

            //TODO: separate out into its own method so we can reconnect if necessary
            // Initialize CameraDeviceManager
            DeviceManager = new CameraDeviceManager();

            // Tell it about the Rpi Camera
            DeviceManager.CustomDeviceClass.Add("Rpi Camera", typeof(RpiMTPCamera));

            // Callbacks
            DeviceManager.PhotoCaptured += DeviceManager_PhotoCaptured;
            //DeviceManager.CameraConnected += DeviceManager_CameraConnected;
            //DeviceManager.CameraDisconnected += DeviceManager_CameraDisconnected;

            // Image Processor
            _imageDataProcessor = new ImageDataProcessor();


            tl.LogMessage("Camera", "Completed initialisation");
        }



        private void PhotoCaptured(object o)
        {
            PhotoCapturedEventArgs eventArgs = o as PhotoCapturedEventArgs;
            if (eventArgs == null)
                return;
            try
            {
                //TODO: fetch to stream
                string fileName = Path.Combine(@"C:\Users\ian\Pictures", Path.GetFileName(eventArgs.FileName));
                // if file exist try to generate a new filename to prevent file lost. 
                // This useful when camera is set to record in ram the the all file names are same.
                if (File.Exists(fileName))
                    fileName =
                      StaticHelper.GetUniqueFilename(
                        Path.GetDirectoryName(fileName) + "\\" + Path.GetFileNameWithoutExtension(fileName) + "_", 0,
                        Path.GetExtension(fileName));

                // check the folder of filename, if not found create it
                if (!Directory.Exists(Path.GetDirectoryName(fileName)))
                {
                    Directory.CreateDirectory(Path.GetDirectoryName(fileName));
                }
                eventArgs.CameraDevice.TransferFile(eventArgs.Handle, fileName);
                // the IsBusy may used internally, if file transfer is done should set to false  

                PrepareCameraImageArray(fileName);

                _cameraImageReady = true;
            }
            catch (Exception exception)
            {
                tl.LogMessage("Camera", exception.StackTrace);
            }
            _cameraState = CameraStates.cameraIdle;
            eventArgs.CameraDevice.IsBusy = false;

        }

        private void PrepareCameraImageArray(string rawFileName)
        {

            if (_imageFormat == 0x04)
            {
                cameraImageArray = _imageDataProcessor.ReadRaw(rawFileName);                
            }
            else
            {
                cameraImageArray = _imageDataProcessor.ReadJpeg(rawFileName);
            }

            /*
            if (_currentBin > 1)
            {
                cameraImageArray = _imageDataProcessor.Binning(cameraImageArray, _currentBin, _currentBin, BinningMode.Sum);  // or BinningMode.Median
            }*/

            cameraImageArray = _imageDataProcessor.CutArray(cameraImageArray, StartX, StartY, NumX, NumY, CameraXSize / BinX, CameraYSize / BinY);
        }

        void DeviceManager_PhotoCaptured(object sender, PhotoCapturedEventArgs eventArgs)
        {
            // to prevent UI freeze start the transfer process in a new thread
            Thread thread = new Thread(PhotoCaptured);
            thread.Start(eventArgs);
        }

        //
        // PUBLIC COM INTERFACE ICameraV3 IMPLEMENTATION
        //

        #region Common properties and methods.

        /// <summary>
        /// Displays the Setup Dialog form.
        /// If the user clicks the OK button to dismiss the form, then
        /// the new settings are saved, otherwise the old values are reloaded.
        /// THIS IS THE ONLY PLACE WHERE SHOWING USER INTERFACE IS ALLOWED!
        /// </summary>
        public void SetupDialog()
        {
            // consider only showing the setup dialog if not connected
            // or call a different dialog if connected
            if (IsConnected)
                System.Windows.Forms.MessageBox.Show("Already connected, just press OK");

            using (SetupDialogForm F = new SetupDialogForm(tl))
            {
                var result = F.ShowDialog();
                if (result == System.Windows.Forms.DialogResult.OK)
                {
                    WriteProfile(); // Persist device configuration values to the ASCOM Profile store
                }
            }
        }

        public ArrayList SupportedActions
        {
            get
            {
                tl.LogMessage("SupportedActions Get", "Returning empty arraylist");
                return new ArrayList();
            }
        }

        public string Action(string actionName, string actionParameters)
        {
            LogMessage("", "Action {0}, parameters {1} not implemented", actionName, actionParameters);
            throw new ASCOM.ActionNotImplementedException("Action " + actionName + " is not implemented by this driver");
        }

        public void CommandBlind(string command, bool raw)
        {
            CheckConnected("CommandBlind");
            // TODO The optional CommandBlind method should either be implemented OR throw a MethodNotImplementedException
            // If implemented, CommandBlind must send the supplied command to the mount and return immediately without waiting for a response

            throw new ASCOM.MethodNotImplementedException("CommandBlind");
        }

        public bool CommandBool(string command, bool raw)
        {
            CheckConnected("CommandBool");
            // TODO The optional CommandBool method should either be implemented OR throw a MethodNotImplementedException
            // If implemented, CommandBool must send the supplied command to the mount, wait for a response and parse this to return a True or False value

            // string retString = CommandString(command, raw); // Send the command and wait for the response
            // bool retBool = XXXXXXXXXXXXX; // Parse the returned string and create a boolean True / False value
            // return retBool; // Return the boolean value to the client

            throw new ASCOM.MethodNotImplementedException("CommandBool");
        }

        public string CommandString(string command, bool raw)
        {
            CheckConnected("CommandString");
            // TODO The optional CommandString method should either be implemented OR throw a MethodNotImplementedException
            // If implemented, CommandString must send the supplied command to the mount and wait for a response before returning this to the client

            throw new ASCOM.MethodNotImplementedException("CommandString");
        }

        public void Dispose()
        {
            // Clean up the trace logger and util objects
            tl.Enabled = false;
            tl.Dispose();
            tl = null;
            utilities.Dispose();
            utilities = null;
            astroUtilities.Dispose();
            astroUtilities = null;
        }

        private bool ConnectDevice()
        {
            if (_connectionInProgress)
                return false;
            _connectionInProgress = true;

            Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;

            // Obtain the portable device collection
            if (PortableDeviceCollection.Instance == null)
            {
                PortableDeviceCollection.CreateInstance(driverID, version.MajorRevision, version.MinorRevision);
                PortableDeviceCollection.Instance.AutoConnectToPortableDevice = false;
            }

            //FIXME fake camera!!

            // Iterate, look for the Rpi Camera & connect to it
            Log.Debug("Connection device start");
            bool found = false;
            try
            {
                var devices = PortableDeviceCollection.Instance.Devices;
                foreach (PortableDevice portableDevice in devices)
                {
                    Log.Debug("Connection device " + portableDevice.DeviceId);

                    // Specificaly this camera!
                    if (!portableDevice.DeviceId.StartsWith("\\\\?\\usb#vid_1d6b&pid_0100"))
                        continue;

                    found = true;
                    portableDevice.ConnectToDevice(driverID, version.MajorRevision, version.MinorRevision);

                    ICameraDevice cameraDevice;
                    DeviceDescriptor descriptor = new DeviceDescriptor { WpdId = portableDevice.DeviceId };
                    //cameraDevice = (ICameraDevice)Activator.CreateInstance(typeof(RpiMTPCamera));
                    cameraDevice = new RpiMTPCamera();

                    MtpProtocol device = new MtpProtocol(descriptor.WpdId);
                    device.ConnectToDevice(driverID, version.MajorRevision, version.MinorRevision);

                    descriptor.StillImageDevice = device;

                    cameraDevice.SerialNumber = StaticHelper.GetSerial(portableDevice.DeviceId);
                    cameraDevice.Init(descriptor);

                    if (string.IsNullOrWhiteSpace(cameraDevice.SerialNumber))
                        cameraDevice.SerialNumber = StaticHelper.GetSerial(portableDevice.DeviceId);
                    descriptor.CameraDevice = cameraDevice;

                    DeviceManager.AddDevice(descriptor);
                }
            }
            catch (Exception exception)
            {
                Log.Error("Unable to connect to camera ", exception);
                return false;
            }

            _connectionInProgress = false;
            return found;
        }

        public bool Connected
        {
            get
            {
                LogMessage("Connected", "Get {0}", IsConnected);
                return IsConnected;
            }
            
            set
            {
                
                tl.LogMessage("Connected", "Set {0}", value);
                if (value == IsConnected)
                    return;

                if (value)
                {
                    LogMessage("Connected", "Connecting to camera");
                    //if (DeviceManager.ConnectToCamera())
                    if (ConnectDevice()) { }
                    {
                        // Wait a short while for initDone. Ideally we'd assign a callback but we can't do that
                        // in advance and by the time we can do it, it's too late
                        Thread.Sleep(1000);
                        _connectedState = true;
                    }

                }
                else
                {
                    LogMessage("Connected", "Disconnecting from camera");
                    DeviceManager.CloseAll();
                    _connectedState = false;
                }
            }
        }

        public string Description
        {
            get
            {
                tl.LogMessage("Description Get", driverDescription);
                return driverDescription;
            }
        }

        public string DriverInfo
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                // TODO customise this driver description
                string driverInfo = "Information about the driver itself. Version: " + String.Format(CultureInfo.InvariantCulture, "{0}.{1}", version.Major, version.Minor);
                tl.LogMessage("DriverInfo Get", driverInfo);
                return driverInfo;
            }
        }

        public string DriverVersion
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                string driverVersion = String.Format(CultureInfo.InvariantCulture, "{0}.{1}", version.Major, version.Minor);
                tl.LogMessage("DriverVersion Get", driverVersion);
                return driverVersion;
            }
        }

        public short InterfaceVersion
        {
            // set by the driver wizard
            get
            {
                LogMessage("InterfaceVersion Get", "3");
                return Convert.ToInt16("3");
            }
        }

        public string Name
        {
            get
            {
                string name = "Rpi Camera MTP";
                tl.LogMessage("Name Get", name);
                return name;
            }
        }

        #endregion

        #region ICamera Implementation

        private const int ccdWidth = 4056; // Constants to define the CCD pixel dimensions
        private const int ccdHeight = 3040;
        private const double pixelSize = 1.55; // Constant for the pixel physical dimension

        private int cameraNumX = ccdWidth; // Initialise variables to hold values required for functionality tested by Conform
        private int cameraNumY = ccdHeight;
        private int cameraStartX = 0;
        private int cameraStartY = 0;
        private DateTime exposureStart = DateTime.MinValue;
        private double cameraLastExposureDuration = 0.0;
        private bool _cameraImageReady = false;


        public void AbortExposure()
        {
            tl.LogMessage("AbortExposure", "Not implemented");
            throw new MethodNotImplementedException("AbortExposure");
        }

        public short BayerOffsetX
        {
            get
            {
                tl.LogMessage("BayerOffsetX Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("BayerOffsetX", false);
            }
        }

        public short BayerOffsetY
        {
            get
            {
                tl.LogMessage("BayerOffsetY Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("BayerOffsetX", true);
            }
        }

        public short BinX
        {
            get
            {
                tl.LogMessage("BinX Get", "1");
                return _currentBin;
            }
            set
            {
                tl.LogMessage("BinX Set", value.ToString());
                _currentBin = value;
            }
        }

        public short BinY
        {
            get
            {
                tl.LogMessage("BinY Get", "1");
                return _currentBin;
            }
            set
            {
                tl.LogMessage("BinY Set", value.ToString());
                _currentBin = value;
            }
        }

        public double CCDTemperature
        {
            get
            {
                tl.LogMessage("CCDTemperature Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("CCDTemperature", false);
            }
        }

        public CameraStates CameraState
        {
            get
            {
                tl.LogMessage("CameraState Get", _cameraState.ToString());
                return _cameraState;
            }
        }

        public int CameraXSize
        {
            get
            {
                tl.LogMessage("CameraXSize Get", ccdWidth.ToString());
                return ccdWidth;
            }
        }

        public int CameraYSize
        {
            get
            {
                tl.LogMessage("CameraYSize Get", ccdHeight.ToString());
                return ccdHeight;
            }
        }

        public bool CanAbortExposure
        {
            get
            {
                tl.LogMessage("CanAbortExposure Get", false.ToString());
                return false;
            }
        }

        public bool CanAsymmetricBin
        {
            get
            {
                tl.LogMessage("CanAsymmetricBin Get", false.ToString());
                return false;
            }
        }

        public bool CanFastReadout
        {
            get
            {
                tl.LogMessage("CanFastReadout Get", false.ToString());
                return true;
            }
        }

        public bool CanGetCoolerPower
        {
            get
            {
                tl.LogMessage("CanGetCoolerPower Get", false.ToString());
                return false;
            }
        }

        public bool CanPulseGuide
        {
            get
            {
                tl.LogMessage("CanPulseGuide Get", false.ToString());
                return false;
            }
        }

        public bool CanSetCCDTemperature
        {
            get
            {
                tl.LogMessage("CanSetCCDTemperature Get", false.ToString());
                return false;
            }
        }

        public bool CanStopExposure
        {
            get
            {
                tl.LogMessage("CanStopExposure Get", false.ToString());
                return false;
            }
        }

        public bool CoolerOn
        {
            get
            {
                tl.LogMessage("CoolerOn Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("CoolerOn", false);
            }
            set
            {
                tl.LogMessage("CoolerOn Set", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("CoolerOn", true);
            }
        }

        public double CoolerPower
        {
            get
            {
                tl.LogMessage("CoolerPower Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("CoolerPower", false);
            }
        }

        public double ElectronsPerADU
        {
            get
            {
                tl.LogMessage("ElectronsPerADU Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("ElectronsPerADU", false);
            }
        }

        public double ExposureMax
        {
            get
            {
                tl.LogMessage("ExposureMax Get", "230");
                return 230;
            }
        }

        public double ExposureMin
        {
            get
            {
                tl.LogMessage("ExposureMin Get", "0.000125");
                return 0.000125;
            }
        }

        public double ExposureResolution
        {
            get
            {
                return 0.01;
            }
        }

        public bool FastReadout
        {
            get
            {
                tl.LogMessage("FastReadout Get", "returning");
                return _fastReadout;
            }
            set
            {
                tl.LogMessage("FastReadout Set", "setting");
                _fastReadout = value;
            }
        }

        public double FullWellCapacity
        {
            get
            {
                tl.LogMessage("FullWellCapacity Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("FullWellCapacity", false);
            }
        }

        public short Gain
        {
            get
            {
                tl.LogMessage("Gain get", _currentGain.ToString());
                return _currentGain;
            }
            set
            {
                tl.LogMessage("Gain set", value.ToString()) ;
                _currentGain = value;                
            }
        }

        public short GainMax
        {
            get
            {
                tl.LogMessage("GainMax Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("GainMax", false);
            }
        }

        public short GainMin
        {
            get
            {
                tl.LogMessage("GainMin Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("GainMin", true);
            }
        }

        public ArrayList Gains
        {
            get
            {
                tl.LogMessage("Gains", "Fetched gains");
                return _gains;
            }
        }

        public bool HasShutter
        {
            get
            {
                tl.LogMessage("HasShutter Get", false.ToString());
                return false;
            }
        }

        public double HeatSinkTemperature
        {
            get
            {
                tl.LogMessage("HeatSinkTemperature Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("HeatSinkTemperature", false);
            }
        }

        public object ImageArray
        {
            get
            {
                if (!ImageReady)
                {                   
                    tl.LogMessage("ImageArray Get", "Throwing InvalidOperationException because of a call to ImageArray before the first image has been taken!");
                    throw new ASCOM.InvalidOperationException("Image not ready!");
                }

                //TODO: change ImageReady to false, delete image
                return cameraImageArray;
            }
        }

        public object ImageArrayVariant
        {
            get
            {
                if (!ImageReady)
                {
                    tl.LogMessage("ImageArrayVariant Get", "Throwing InvalidOperationException because of a call to ImageArrayVariant before the first image has been taken!");
                    throw new ASCOM.InvalidOperationException("Image not ready!");
                }
                //TODO: change ImageReady to false, delete image

                return cameraImageArray;
            }
        }

        public bool ImageReady
        {
            get
            {
                tl.LogMessage("ImageReady Get", _cameraImageReady.ToString());
                return _cameraImageReady;
            }
        }

        public bool IsPulseGuiding
        {
            get
            {
                tl.LogMessage("IsPulseGuiding Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("IsPulseGuiding", false);
            }
        }

        public double LastExposureDuration
        {
            get
            {
                tl.LogMessage("LastExposureDuration Get", cameraLastExposureDuration.ToString());
                return cameraLastExposureDuration;
            }
        }

        public string LastExposureStartTime
        {
            get
            {
                string exposureStartString = exposureStart.ToString("yyyy-MM-ddTHH:mm:ss");
                tl.LogMessage("LastExposureStartTime Get", exposureStartString.ToString());
                return exposureStartString;
            }
        }

        public int MaxADU
        {
            get
            {
                tl.LogMessage("MaxADU Get", "20000");
                return 20000;
            }
        }

        public short MaxBinX
        {
            get
            {
                tl.LogMessage("MaxBinX Get", "2");
                return 2;
            }
        }

        public short MaxBinY
        {
            get
            {
                tl.LogMessage("MaxBinY Get", "2");
                return MaxBinX;
            }
        }

        public int NumX
        {
            get
            {
                tl.LogMessage("NumX Get", cameraNumX.ToString());
                return cameraNumX / _currentBin;
            }
            set
            {
                //cameraNumX = value;
                tl.LogMessage("NumX set", value.ToString());
            }
        }

        public int NumY
        {
            get
            {
                tl.LogMessage("NumY Get", cameraNumY.ToString());
                return cameraNumY / _currentBin;
            }
            set
            {
                //cameraNumY = value;
                tl.LogMessage("NumY set", value.ToString());
            }
        }

        public int Offset
        {
            get
            {
                tl.LogMessage("Offset Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("Offset", false);
            }
            set
            {
                tl.LogMessage("Offset Set", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("Offset", true);
            }
        }

        public int OffsetMax
        {
            get
            {
                tl.LogMessage("OffsetMax Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("OffsetMax", false);
            }
        }

        public int OffsetMin
        {
            get
            {
                tl.LogMessage("OffsetMin Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("OffsetMin", true);
            }
        }

        public ArrayList Offsets
        {
            get
            {
                tl.LogMessage("Offsets Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("Offsets", true);
            }
        }

        public short PercentCompleted
        {
            get
            {
                tl.LogMessage("PercentCompleted Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("PercentCompleted", false);
            }
        }

        public double PixelSizeX
        {
            get
            {
                tl.LogMessage("PixelSizeX Get", pixelSize.ToString());
                return pixelSize;
            }
        }

        public double PixelSizeY
        {
            get
            {
                //FIXME = * binning?
                tl.LogMessage("PixelSizeY Get", pixelSize.ToString());
                return pixelSize;
            }
        }

        public void PulseGuide(GuideDirections Direction, int Duration)
        {
            tl.LogMessage("PulseGuide", "Not implemented");
            throw new ASCOM.MethodNotImplementedException("PulseGuide");
        }

        public short ReadoutMode
        {
            get
            {
                if (_imageFormat == 1)
                    return (short)ImageFormat.JPEG;
                return (short)ImageFormat.RAW;
            }
            set
            {
                switch(value)
                {
                    case ((short)ImageFormat.JPEG):
                        _imageFormat = 1;
                        _sensorType = SensorType.Color;
                        break;
                    case ((short)ImageFormat.RAW):
                        _imageFormat = 4;
                        _sensorType = SensorType.RGGB;
                        break;
                    default:
                        _imageFormat = 4;
                        _sensorType = SensorType.RGGB;
                        break;                        
                }
            }
        }

        public ArrayList ReadoutModes
        {
            get
            {
                return new ArrayList(new[] { ImageFormat.RAW.ToString(), ImageFormat.JPEG.ToString() });
            }
        }

        public string SensorName
        {
            get
            {
                //FIXME: allow other sensor types (also fix image sizes)
                return "IMX477";
            }
        }

        public SensorType SensorType
        {
            get
            {
                return _sensorType;
            }
        }

        public double SetCCDTemperature
        {
            get
            {
                tl.LogMessage("SetCCDTemperature Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("SetCCDTemperature", false);
            }
            set
            {
                tl.LogMessage("SetCCDTemperature Set", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("SetCCDTemperature", true);
            }
        }

        public void StartExposure(double Duration, bool Light)
        {

            tl.LogMessage("StartExposure", Duration.ToString() + " " + Light.ToString());

            if (Duration < 0.0) throw new InvalidValueException("StartExposure", Duration.ToString(), "0.0 upwards");
            if (cameraNumX > ccdWidth) throw new InvalidValueException("StartExposure", cameraNumX.ToString(), ccdWidth.ToString());
            if (cameraNumY > ccdHeight) throw new InvalidValueException("StartExposure", cameraNumY.ToString(), ccdHeight.ToString());
            if (cameraStartX > ccdWidth) throw new InvalidValueException("StartExposure", cameraStartX.ToString(), ccdWidth.ToString());
            if (cameraStartY > ccdHeight) throw new InvalidValueException("StartExposure", cameraStartY.ToString(), ccdHeight.ToString());

            // Send setup commands
            ICameraDevice camera = DeviceManager.SelectedCameraDevice;
            camera.Mode.SetValue("M");
            camera.CompressionSetting.SetValue(_imageFormat);            

            // Low level hackery so we can set the exposure time explicitly
            ((BaseMTPCamera)camera).SetProperty(RpiMTPCamera.CONST_CMD_SetDevicePropValue, BitConverter.GetBytes(((long)Duration * 10000)),
                                    RpiMTPCamera.CONST_PROP_ExposureTime);

            // Low level hackery so we can set the binned image size correctly
            int w = ccdWidth / BinX;
            int h = ccdHeight / BinY;

            String s = w + "x" + h;
            List<byte> vals = new List<byte>() { 10 };
            vals.AddRange(Encoding.Unicode.GetBytes(s));
            vals.Add(0x00);
            ((BaseMTPCamera)camera).SetProperty(RpiMTPCamera.CONST_CMD_SetDevicePropValue, vals.ToArray(), RpiMTPCamera.CONST_PROP_ImageSize);


            camera.IsoNumber.SetValue(Convert.ToInt16((String)_gains[_currentGain]));

            tl.LogMessage("StartExposure", "ss=" + ((long)Duration * 10000).ToString() + ", format=" + _imageFormat.ToString() + ", iso = " + Convert.ToInt16((String)_gains[_currentGain]));

            //TODO binning

            try
            {
                camera.CapturePhoto();
                cameraLastExposureDuration = Duration;
                exposureStart = DateTime.Now;

                _cameraState = CameraStates.cameraExposing;
                _cameraImageReady = false;

            } catch (Exception ex)
            {
                tl.LogMessage("StartExposure", ex.StackTrace);
                _cameraState = CameraStates.cameraIdle;
                throw new ASCOM.DriverException("Error during CapturePhoto", ex);
            }
        }

        public int StartX
        {
            get
            {
                tl.LogMessage("StartX Get", cameraStartX.ToString());
                return cameraStartX;
            }
            set
            {
                cameraStartX = value;
                tl.LogMessage("StartX Set", value.ToString());
            }
        }

        public int StartY
        {
            get
            {
                tl.LogMessage("StartY Get", cameraStartY.ToString());
                return cameraStartY;
            }
            set
            {
                cameraStartY = value;
                tl.LogMessage("StartY set", value.ToString());
            }
        }

        public void StopExposure()
        {
            tl.LogMessage("StopExposure", "Not implemented");
            throw new MethodNotImplementedException("StopExposure");
        }

        public double SubExposureDuration
        {
            get
            {
                tl.LogMessage("SubExposureDuration Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("SubExposureDuration", false);
            }
            set
            {
                tl.LogMessage("SubExposureDuration Set", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("SubExposureDuration", true);
            }
        }

        #endregion

        #region Private properties and methods
        // here are some useful properties and methods that can be used as required
        // to help with driver development

        #region ASCOM Registration

        // Register or unregister driver for ASCOM. This is harmless if already
        // registered or unregistered. 
        //
        /// <summary>
        /// Register or unregister the driver with the ASCOM Platform.
        /// This is harmless if the driver is already registered/unregistered.
        /// </summary>
        /// <param name="bRegister">If <c>true</c>, registers the driver, otherwise unregisters it.</param>
        private static void RegUnregASCOM(bool bRegister)
        {
            using (var P = new ASCOM.Utilities.Profile())
            {
                P.DeviceType = "Camera";
                if (bRegister)
                {
                    P.Register(driverID, driverDescription);
                }
                else
                {
                    P.Unregister(driverID);
                }
            }
        }

        /// <summary>
        /// This function registers the driver with the ASCOM Chooser and
        /// is called automatically whenever this class is registered for COM Interop.
        /// </summary>
        /// <param name="t">Type of the class being registered, not used.</param>
        /// <remarks>
        /// This method typically runs in two distinct situations:
        /// <list type="numbered">
        /// <item>
        /// In Visual Studio, when the project is successfully built.
        /// For this to work correctly, the option <c>Register for COM Interop</c>
        /// must be enabled in the project settings.
        /// </item>
        /// <item>During setup, when the installer registers the assembly for COM Interop.</item>
        /// </list>
        /// This technique should mean that it is never necessary to manually register a driver with ASCOM.
        /// </remarks>
        [ComRegisterFunction]
        public static void RegisterASCOM(Type t)
        {
            RegUnregASCOM(true);
        }

        /// <summary>
        /// This function unregisters the driver from the ASCOM Chooser and
        /// is called automatically whenever this class is unregistered from COM Interop.
        /// </summary>
        /// <param name="t">Type of the class being registered, not used.</param>
        /// <remarks>
        /// This method typically runs in two distinct situations:
        /// <list type="numbered">
        /// <item>
        /// In Visual Studio, when the project is cleaned or prior to rebuilding.
        /// For this to work correctly, the option <c>Register for COM Interop</c>
        /// must be enabled in the project settings.
        /// </item>
        /// <item>During uninstall, when the installer unregisters the assembly from COM Interop.</item>
        /// </list>
        /// This technique should mean that it is never necessary to manually unregister a driver from ASCOM.
        /// </remarks>
        [ComUnregisterFunction]
        public static void UnregisterASCOM(Type t)
        {
            RegUnregASCOM(false);
        }

        #endregion

        /// <summary>
        /// Returns true if there is a valid connection to the driver hardware
        /// </summary>
        private bool IsConnected
        {
            get
            {
                // TODO check that the driver hardware connection exists and is connected to the hardware
                return _connectedState;
            }
        }

        /// <summary>
        /// Use this function to throw an exception if we aren't connected to the hardware
        /// </summary>
        /// <param name="message"></param>
        private void CheckConnected(string message)
        {
            if (!IsConnected)
            {
                throw new ASCOM.NotConnectedException(message);
            }
        }

        /// <summary>
        /// Read the device configuration from the ASCOM Profile store
        /// </summary>
        internal void ReadProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Camera";
                tl.Enabled = Convert.ToBoolean(driverProfile.GetValue(driverID, traceStateProfileName, string.Empty, traceStateDefault));
            }
        }

        /// <summary>
        /// Write the device configuration to the  ASCOM  Profile store
        /// </summary>
        internal void WriteProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Camera";
                driverProfile.WriteValue(driverID, traceStateProfileName, tl.Enabled.ToString());
            }
        }

        /// <summary>
        /// Log helper function that takes formatted strings and arguments
        /// </summary>
        /// <param name="identifier"></param>
        /// <param name="message"></param>
        /// <param name="args"></param>
        internal void LogMessage(string identifier, string message, params object[] args)
        {
            var msg = string.Format(message, args);
            tl.LogMessage(identifier, msg);
        }
        #endregion
    }
}
