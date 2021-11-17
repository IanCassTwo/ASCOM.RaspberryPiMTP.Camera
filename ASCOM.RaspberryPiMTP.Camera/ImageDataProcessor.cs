﻿using ASCOM.Utilities;
using ASCOM.RaspberryPiMTP.Native;
using ASCOM.RaspberryPiMTP.Enums;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;

namespace ASCOM.RaspberryPiMTP
{
    public class ImageDataProcessor
    {

        internal TraceLogger tl = new TraceLogger("", "RaspberryPiMTP");

        private IntPtr LoadRaw(string fileName)
        {
            tl.LogMessage("ImageDataProcessor InPtr", "libraw_init");
            IntPtr data = NativeMethods.libraw_init(LibRaw_constructor_flags.LIBRAW_OPIONS_NO_DATAERR_CALLBACK);
            tl.LogMessage("ImageDataProcessor InPtr", "libraw_open_file");
            CheckError(NativeMethods.libraw_open_file(data, fileName), "open file");
            tl.LogMessage("ImageDataProcessor InPtr", "unpack");
            CheckError(NativeMethods.libraw_unpack(data), "unpack");
            tl.LogMessage("ImageDataProcessor InPtr", "raw2image");
            CheckError(NativeMethods.libraw_raw2image(data), "raw2image");
            tl.LogMessage("ImageDataProcessor InPtr", "return");
            // Don't subtract black level as that pushes the histogram right down to the left hand side for dark areas - ie data being lost
            //            CheckError(NativeMethods.libraw_subtract_black(data), "subtract");

            return data;
        }

        private void CheckError(int errorCode, string action)
        {
            if (errorCode != 0)
                throw new Exception($"LibRaw returned error code {errorCode} when {action}");
        }

        public int[,,] ReadAndDebayerRaw(string fileName)
        {
            IntPtr data = LoadRaw(fileName);
            tl.LogMessage("ImageDataProcessor ReadAndDebayerRaw", "libraw_dcraw_process");
            NativeMethods.libraw_dcraw_process(data);

            var dataStructure = GetStructure<libraw_data_t>(data);
            ushort width = dataStructure.sizes.iwidth;
            ushort height = dataStructure.sizes.iheight;

            var pixels = new int[width, height, 3];

            Parallel.For(0, width * height, rc =>
            {
                var r = (ushort)Marshal.ReadInt16(dataStructure.image, rc * 8);
                var g = (ushort)Marshal.ReadInt16(dataStructure.image, rc * 8 + 2);
                var b = (ushort)Marshal.ReadInt16(dataStructure.image, rc * 8 + 4);

                int row = rc / width;
                int col = rc - width * row;
                //int rowReversed = height - row - 1;
                pixels[col, row, 0] = b;
                pixels[col, row, 1] = g;
                pixels[col, row, 2] = r;
            });

                /*
                for (int rc = 0; rc < height * width; rc++)
                {
                    var r = (ushort)Marshal.ReadInt16(dataStructure.image, rc * 8);
                    var g = (ushort)Marshal.ReadInt16(dataStructure.image, rc * 8 + 2);
                    var b = (ushort)Marshal.ReadInt16(dataStructure.image, rc * 8 + 4);

                    int row = rc / width;
                    int col = rc - width * row;
                    //int rowReversed = height - row - 1;
                    pixels[col, row, 0] = b;
                    pixels[col, row, 1] = g;
                    pixels[col, row, 2] = r;
                }*/

                NativeMethods.libraw_close(data);

            return pixels;
        }

        public int[,,] ReadJpeg(string fileName)
        {
            Bitmap img = new Bitmap(fileName);
            var result = ReadBitmap(img);
            return result;
        }

        public int From8To16Bit(int value)
        {
            int result = value << 5;

            return result;
        }

        public int[,] ToMonochrome(Array data, Func<int, int> process)
        {
            var result = new int[data.GetLength(0), data.GetLength(1)];
            for (int x = 0; x < data.GetLength(0); x++)
                for (int y = 0; y < data.GetLength(1); y++)
                {
                    for (int c = 0; c < 3; c++)
                    {
                        result[x, y] += (int)data.GetValue(x, y, c);
                    }
                    result[x, y] = process(result[x, y]);
                }

            return result;
        }



        public int[,,] ReadBitmap(Bitmap img)
        {
           
            BitmapData data = img.LockBits(new Rectangle(0, 0, img.Width, img.Height), ImageLockMode.ReadOnly, img.PixelFormat);
            IntPtr ptr = data.Scan0;
            int bytesCount = Math.Abs(data.Stride) * img.Height;
            var result = new int[img.Width, img.Height, 3];

            byte[] bytesArray = new byte[bytesCount];
            Marshal.Copy(ptr, bytesArray, 0, bytesCount);
            img.UnlockBits(data);

            var width = img.Width;
            var height = img.Height;

            Parallel.For(0, width * height, rc =>
            {
                var b = bytesArray[rc * 3];
                var g = bytesArray[rc * 3 + 1];
                var r = bytesArray[rc * 3 + 2];

                int row = rc / width;
                int col = rc - width * row;

                //var rowReversed = height - row - 1;
                result[col, row, 0] = b;
                result[col, row, 1] = g;
                result[col, row, 2] = r;
            }
            );

                /*
                for (int rc = 0; rc < width * height; rc++)
                {
                    var b = bytesArray[rc * 3];
                    var g = bytesArray[rc * 3 + 1];
                    var r = bytesArray[rc * 3 + 2];

                    int row = rc / width;
                    int col = rc - width * row;

                    //var rowReversed = height - row - 1;
                    result[col, row, 0] = b;
                    result[col, row, 1] = g;
                    result[col, row, 2] = r;
                }*/

                return result;
        }

        private void exif_parser_callback(IntPtr context, int tag, int type, int len, uint ord, IntPtr ifp)
        {

        }

        public unsafe int[,] ReadRaw(string fileName)
        {
            IntPtr data = LoadRaw(fileName);

            var dataStructure = GetStructure<libraw_data_t>(data);

            int xoffs = 0;
            int yoffs = 0;

            ushort width = dataStructure.sizes.iwidth;
            ushort height = dataStructure.sizes.iheight;

            var pixels = new int[width, height];

            /*
            Parallel.For(0, height - yoffs, y =>
             {

                 int i0 = NativeMethods.libraw_COLOR(data, y, 0);
                 int i1 = NativeMethods.libraw_COLOR(data, y, 1);
                 ushort* ptr = (ushort*)((byte*)dataStructure.image.ToPointer() + width * 8 * y);

                 for (int x = 0; x < width - xoffs; x += 2)
                 {
                     pixels[x + xoffs, y + yoffs] = *(ptr + i0);
                     ptr += 4;
                     pixels[x + xoffs + 1, y + yoffs] = *(ptr + i1);
                     ptr += 4;
                 }

             }
            );
            */
            
            
            for (int y = 0; y < height - yoffs; y++)
            {
                int i0 = NativeMethods.libraw_COLOR(data, y,0);
                int i1 = NativeMethods.libraw_COLOR(data, y,1);

                ushort* ptr = (ushort*) ((byte*)dataStructure.image.ToPointer() + width*8 * y);

                for (int x = 0; x < width - xoffs; x+=2)
                {
                    pixels[x + xoffs, y + yoffs] = *(ptr + i0);
                    ptr += 4;
                    pixels[x + xoffs+1, y + yoffs] = *(ptr + i1);
                    ptr += 4;
                }
            }
            


            if (dataStructure.color.maximum > 0)
            {
                int multiplier = (int) Math.Pow(2, Math.Floor(Math.Log(32768.0 / dataStructure.color.maximum, 2)));
                if (multiplier > 1)
                {

                    
                    Parallel.For(0, height, y =>
                    {

                        Parallel.For(0, width, x =>
                        {
                            pixels[x, y] *= multiplier;
                        }
                        );
                    }
                    );
                    
                    /*
                    for (int y = 0; y < height; y++)
                    {
                        for (int x = 0; x < width; x++)
                        {
                            pixels[x, y] *= multiplier;
                        }
                    }
                    */
                }
            }
            NativeMethods.libraw_close(data);

            pixels = BGGRToRGGB(pixels, width, height);

            return pixels;
        }

        public int[,] BGGRToRGGB(Array bayer, int width, int height)
        {

            // If we have BGGR, ASCOM expects to return RBBG so we need to map. Let's swap order of values

            var newImageArray = new int[width, height];

            for (int x = 0; x < width; x += 2)
                for (int y = 0; y < height; y += 2)
                {
                    int bluePixel = (int)bayer.GetValue(x, y);
                    int greenPixel1 = (int)bayer.GetValue(x + 1, y);
                    int greenPixel2 = (int)bayer.GetValue(x, y + 1);
                    int redPixel = (int)bayer.GetValue(x + 1, y + 1);
                    newImageArray.SetValue(bluePixel, x + 1, y + 1);
                    newImageArray.SetValue(greenPixel1, x, y + 1);
                    newImageArray.SetValue(greenPixel2, x + 1, y);
                    newImageArray.SetValue(redPixel, x, y);
                }
        

            return newImageArray;
        }

        public Array Binning(Array data, int binx, int biny, BinningMode binningMode)
        {
            int width = data.GetLength(0);
            int height = data.GetLength(1);
            int binWidth = width / binx;
            int binHeight = height / biny;

            var result = Array.CreateInstance(typeof(int), binWidth, binHeight);

            for (int x = 0; x < binWidth; x++)
                for (int y = 0; y < binHeight; y++)
                {
                    var binBlockData = new List<int>();
                    for (int x2 = x * binx; x2 < x * binx + binx; x2++)
                        for (int y2 = y * biny; y2 < y * biny + biny; y2++)
                        {
                            binBlockData.Add((int)data.GetValue(x2, y2));
                        }

                    int value = 0;
                    switch (binningMode)
                    {
                        case BinningMode.Sum:
                            value = GetSum(binBlockData, binx, biny);
                            break;
                        case BinningMode.Median:
                            value = GetMedian(binBlockData);
                            break;
                    }
                    result.SetValue(value, x, y);
                }

            return result;
        }

        public Array CutArray(Array data, int StartX, int StartY, int NumX, int NumY, int CameraXSize, int CameraYSize)
        {
            Array result = null;
            int rank = data.Rank;

            if (IsCutRequired(data.GetLength(0), data.GetLength(1), StartX, StartY, NumX, NumY, CameraXSize, CameraYSize))
            {
                int startXCorrected = StartX % 2 == 0 ? StartX : StartX - 1;
                int startYCorrected = StartY % 2 == 0 ? StartY : StartY - 1;

                result = rank == 3 ? Array.CreateInstance(typeof(int), NumX, NumY, 3)
                                   : Array.CreateInstance(typeof(int), NumX, NumY);

                /*
                Parallel.For(0, NumX, x =>
                {

                    Parallel.For(0, NumY, y =>
                    {
                        int dataX = startXCorrected + x;
                        int dataY = startYCorrected + y;
                        if (rank == 3)
                        {
                            Parallel.For(0, 3, r =>
                            {
                                result.SetValue(data.GetValue(dataX, dataY, r), x, y, r);
                            });
                        }
                        else
                        {
                            result.SetValue(data.GetValue(dataX, dataY), x, y);
                        }

                    });

                });
                */



                for (int x = 0; x < NumX; x++)
                    for (int y = 0; y < NumY; y++)
                    {
                        int dataX = startXCorrected + x;
                        int dataY = startYCorrected + y;
                        if (rank == 3)
                        {
                            for (int r = 0; r < 3; r++)
                            {
                                result.SetValue(data.GetValue(dataX, dataY, r), x, y, r);
                            }
                        }
                        else
                        {
                            result.SetValue(data.GetValue(dataX, dataY), x, y);
                        }
                    }
            }
            else
            {
                result = data;
            }
            return result;
        }

        private bool IsCutRequired(int dataXsize, int dataYsize, int StartX, int StartY, int NumX, int NumY, int CameraXSize, int CameraYSize)
        {
            bool sizeMatches = StartX == 0 && StartY == 0 && NumX == CameraXSize && NumY == CameraYSize
                && dataXsize == CameraXSize && dataYsize == CameraYSize;

            bool cut = !(sizeMatches || NumX == 0 || NumY == 0);
            return cut;
        }

        private int GetMedian(IEnumerable<int> sourceNumbers)
        {
            int[] sortedPNumbers = sourceNumbers.OrderBy(n => n).ToArray();

            int size = sortedPNumbers.Length;
            int mid = size / 2;
            int median = (size % 2 != 0) ? sortedPNumbers[mid] : (sortedPNumbers[mid] + sortedPNumbers[mid - 1]) / 2;
            return median;
        }

        private int GetSum(IEnumerable<int> sourceNumbers, int binx, int biny)
        {
            int binCount = binx * biny;
            var sum = sourceNumbers.Sum();

            if (binCount > 4)
            {
                sum = sum >> 2;
            }

            return sum;
        }

        private T GetStructure<T>(IntPtr ptr)
       where T : struct
        {
            return (T)Marshal.PtrToStructure(ptr, typeof(T));
        }
               
    }
}
