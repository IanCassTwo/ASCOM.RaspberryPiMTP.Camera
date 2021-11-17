// This implements a console application that can be used to test an ASCOM driver
//

// This is used to define code in the template that is specific to one class implementation
// unused code can be deleted and this definition removed.

#define Camera
// remove this to bypass the code that uses the chooser to select the driver
#define UseChooser

using System;
using System.Threading;

namespace ASCOM
{
    class Program
    {
        static void Main(string[] args)
        {
            // Uncomment the code that's required
#if UseChooser
            // choose the device
            string id = ASCOM.DriverAccess.Camera.Choose("");
            if (string.IsNullOrEmpty(id))
                return;
            // create this device
            ASCOM.DriverAccess.Camera device = new ASCOM.DriverAccess.Camera(id);
#else
            // this can be replaced by this code, it avoids the chooser and creates the driver class directly.
            ASCOM.DriverAccess.Camera device = new ASCOM.DriverAccess.Camera("ASCOM.RaspberryPiMTP.Camera");
#endif
            // now run some tests, adding code to your driver so that the tests will pass.
            // these first tests are common to all drivers.
            Console.WriteLine("name " + device.Name);
            Console.WriteLine("description " + device.Description);
            Console.WriteLine("DriverInfo " + device.DriverInfo);
            Console.WriteLine("driverVersion " + device.DriverVersion);
           
            // TODO add more code to test the driver.
            device.Connected = true;



            Console.WriteLine("Press Enter to Take raw");
            Console.ReadLine();
            device.ReadoutMode = 0x04;
            //device.BinX = 2;
            device.StartExposure(1, true);

            while (!device.ImageReady)
            {
                Thread.Sleep(100);
            }
            Array ary = (Array)device.ImageArray;


            Console.WriteLine("Press Enter to Take binned raw");
            Console.ReadLine();
            device.ReadoutMode = 0x04;
            device.BinX = 2;
            device.StartExposure(1, true);

            while (!device.ImageReady)
            {
                Thread.Sleep(100);
            }

            ary = (Array)device.ImageArray;


            Console.WriteLine("Press Enter to Take another raw");
            Console.ReadLine();
            device.ReadoutMode = 0x04;
            device.BinX = 1;
            device.StartExposure(1, true);

            while (!device.ImageReady)
            {
                Thread.Sleep(100);
            }

            ary = (Array)device.ImageArray;

            Console.WriteLine("Press Enter to Take jpeg");
            Console.ReadLine();
            device.ReadoutMode = 0x01;
            device.StartExposure(0.5, true);

            while (!device.ImageReady)
            {
                Thread.Sleep(100);
            }
            ary = (Array)device.ImageArray;
            

            Console.WriteLine("Press Enter to Finish");
            Console.ReadLine();

            device.Connected = false;
        }
    }
}
