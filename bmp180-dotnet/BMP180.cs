using i2cdotnet;
using System;
using System.Threading;

namespace bmp180dotnet
{
    public class BMP180 : I2c, IDisposable
    {
        const int addr = 0x77;
        string device = "/dev/i2c-{0}";
        int fd;
        Int16 mode;

        const Int16 BMP085_CAL_AC1 = 0xAA;
        const Int16 BMP085_CAL_AC2 = 0xAC;
        const Int16 BMP085_CAL_AC3 = 0xAE;
        const UInt16 BMP085_CAL_AC4 = 0xB0;
        const UInt16 BMP085_CAL_AC5 = 0xB2;
        const UInt16 BMP085_CAL_AC6 = 0xB4;
        const Int16 BMP085_CAL_B1 = 0xB6;
        const Int16 BMP085_CAL_B2 = 0xB8;
        const Int16 BMP085_CAL_MB = 0xBA;
        const Int16 BMP085_CAL_MC = 0xBC;
        const Int16 BMP085_CAL_MD = 0xBE;
        const Int16 BMP085_CONTROL = 0xF4;
        const Int16 BMP085_READTEMPCMD = 0x2E;
        const Int16 BMP085_READPRESSURECMD = 0x34;
        const Int16 BMP085_TEMPDATA = 0xF6;
        const Int16 BMP085_PRESSUREDATA = 0xF6;

        const Int16 BMP085_ULTRALOWPOWER = 0;
        const Int16 BMP085_STANDARD = 1;
        const Int16 BMP085_HIGHRES = 2;
        const Int16 BMP085_ULTRAHIGHRES = 3;

        struct Bmp180_Calibration
        {
            public Int16 AC1;
            public Int16 AC2;
            public Int16 AC3;
            public UInt16 AC4;
            public UInt16 AC5;
            public UInt16 AC6;
            public Int16 B1;
            public Int16 B2;
            public Int16 MB;
            public Int16 MC;
            public Int16 MD;
        }

        Bmp180_Calibration bmp180_Calibration;
        Int32 X1;
        Int32 X2;
        Int32 X3;
        Int32 B3;
        Int32 B4;
        Int32 B5;
        Int32 B6;
        Int32 B7;
        Int32 T;
        Int32 p;

        public BMP180(int bus = 1, Int16 mode = BMP085_STANDARD)
        {
            this.mode = mode;
            device = String.Format(device, bus);
            fd = Open(device, OPEN_READ_WRITE);
            LoadCalibration();
        }

        public void Close()
        {
            Close(fd);
        }

        public int ReadTemperature()
        {
            Int32 UT = ReadRawTemperature();

            X1 = (UT - bmp180_Calibration.AC6) * bmp180_Calibration.AC5 / (1 << 15);
            X2 = bmp180_Calibration.MC * (1 << 11) / (X1 + bmp180_Calibration.MD);
            B5 = (X1 + X2);
            T = (B5 + 8) / (1 << 4);

            return T;
        }

        public int ReadPressure()
        {
            Int32 UT = ReadRawTemperature();
            Int32 UP = ReadRawPressure();

            X1 = (UT - bmp180_Calibration.AC6) * bmp180_Calibration.AC5 / (1 << 15);
            X2 = bmp180_Calibration.MC * (1 << 11) / (X1 + bmp180_Calibration.MD);
            B5 = (X1 + X2);

            B6 = B5 - 4000;
            X1 = (bmp180_Calibration.B2 * (B6 * B6 / (1 << 12))) / (1 << 11);
            X2 = bmp180_Calibration.AC2 * B6 / (1 << 11);
            X3 = X1 + X2;
            B3 = (((bmp180_Calibration.AC1 * 4 + X3) << mode) + 2) / 4;
            X1 = bmp180_Calibration.AC3 * B6 / (1 << 13);
            X2 = (bmp180_Calibration.B1 * (B6 * B6 / (1 << 12))) / (1 << 16);
            X3 = ((X1 + X2) + 2) / (1 << 2);
            B4 = bmp180_Calibration.AC4 * (X3 + 32768) / (1 << 15);
            B7 = (UP - B3) * (50000 >> mode);

            p = (B7 / B4) * 2;
            X1 = (p / (1 << 8)) * (p / (1 << 8));
            X1 = (X1 * 3038) / (1 << 16);
            X2 = (-7357 * p) / (1 << 16);
            p = p + (X1 + X2 + 3791) / (1 << 4);

            return p;
        }

        public double ReadAltitude(double sealevel_pa = 101325.0)
        {
            var pressure = (double)ReadPressure();
            var altitude = 44330.0 * (1.0 - Math.Pow((pressure / sealevel_pa), (1.0 / 5.255)));


            return altitude;
        }

        public double ReadSeaAltitude(double altitude = 0.0)
        {
            var pressure = (double)ReadPressure();
            var p0 = pressure / Math.Pow((1.0 - (altitude / 44333.0)), 5.255);

            return p0;
        }

        int ReadRawTemperature()
        {
            Int16 raw = 0;
            RegWriteByte(fd, addr, BMP085_CONTROL, BMP085_READTEMPCMD);
            Thread.Sleep(5);
            RegReadShort(fd, addr, BMP085_TEMPDATA, ref raw);

            return raw;
        }


        int ReadRawPressure()
        {
            int msb = 0;
            int lsb = 0;
            int xlsb = 0;

            RegWriteByte(fd, addr, BMP085_CONTROL, (Int16)(BMP085_READPRESSURECMD + (mode << 6)));
            Thread.Sleep(25);
            RegReadByte(fd, addr, BMP085_PRESSUREDATA, ref msb);
            RegReadByte(fd, addr, BMP085_PRESSUREDATA + 1, ref lsb);
            RegReadByte(fd, addr, BMP085_PRESSUREDATA + 2, ref xlsb);
            var raw = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - this.mode);

            return raw;
        }

        Bmp180_Calibration LoadCalibration()
        {
            bmp180_Calibration = new Bmp180_Calibration()
            {
                AC1 = 0,
                AC2 = 0,
                AC3 = 0,
                AC4 = 0,
                AC5 = 0,
                AC6 = 0,
                B1 = 0,
                B2 = 0,
                MB = 0,
                MC = 0,
                MD = 0
            };

            RegReadShort(fd, addr, BMP085_CAL_AC1, ref bmp180_Calibration.AC1);
            RegReadShort(fd, addr, BMP085_CAL_AC2, ref bmp180_Calibration.AC2);
            RegReadShort(fd, addr, BMP085_CAL_AC3, ref bmp180_Calibration.AC3);
            RegReadShort(fd, addr, BMP085_CAL_AC4, ref bmp180_Calibration.AC4);
            RegReadShort(fd, addr, BMP085_CAL_AC5, ref bmp180_Calibration.AC5);
            RegReadShort(fd, addr, BMP085_CAL_AC6, ref bmp180_Calibration.AC6);
            RegReadShort(fd, addr, BMP085_CAL_B1, ref bmp180_Calibration.B1);
            RegReadShort(fd, addr, BMP085_CAL_B2, ref bmp180_Calibration.B2);
            RegReadShort(fd, addr, BMP085_CAL_MB, ref bmp180_Calibration.MB);
            RegReadShort(fd, addr, BMP085_CAL_MC, ref bmp180_Calibration.MC);
            RegReadShort(fd, addr, BMP085_CAL_MD, ref bmp180_Calibration.MD);

            return bmp180_Calibration;
        }

        private bool disposedValue = false;

        protected virtual void Dispose(bool disposing)
        {
            if (!disposedValue)
            {
                Close(fd);
                disposedValue = true;
            }
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }
    }
}
