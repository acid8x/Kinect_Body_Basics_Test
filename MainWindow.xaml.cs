namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.IO.Ports;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Threading;
    using System.Windows.Media.Media3D;
    using System.Management;
    using System.Xml;
    using System.Xml.Serialization;

    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        static SerialPort sp;
        static int[] rxBytes;

        static string speed = "S600", SSC32_pnpId = "VID_0403+PID_6001+A50285BIA", recordingFile = "recordingFile.txt", servoValuesObject = "servoValuesObject.txt", projectDirectory = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDi‌​rectory, "..\\..\\..\\")).ToString();
        static int servoNumber = 6, recordingFPS = 2;
        static bool pauseServoUpdates = true, recordPosition = false;
        long timer;

        servoValues[] _servoValues = new servoValues[servoNumber];
        static servoJoints[] _servoJoints = new servoJoints[servoNumber];
        static Body thisBody;

        private const double HandSize = 30;
        private const double JointThickness = 3;
        private const double ClipBoundsThickness = 10;
        private const float InferredZPositionClamp = 0.1f;
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        private KinectSensor kinectSensor = null;
        private CoordinateMapper coordinateMapper = null;
        private BodyFrameReader bodyFrameReader = null;
        private Body[] bodies = null;
        private int bodyIndex;
        private bool bodyTracked = false;
        private List<Tuple<JointType, JointType>> bones;
        private int displayWidth;
        private int displayHeight;
        private List<Pen> bodyColors;
        private string statusText = null;

        public bool initSerial(string port)
        {
            sp = new SerialPort(port);
            sp.PortName = port;
            sp.BaudRate = 115200;
            sp.NewLine = "\r";
            sp.Parity = Parity.None;
            sp.StopBits = StopBits.One;
            sp.DataBits = 8;
            sp.Handshake = Handshake.None;
            sp.DtrEnable = true;
            sp.RtsEnable = true;
            sp.DataReceived += Sp_DataReceived;
            try { sp.Open(); }
            catch (Exception) { }
            return sp.IsOpen;
        }

        private void Sp_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            int len = sp.BytesToRead;
            for (int i = 0; i < len; i++) rxBytes[i] = sp.ReadByte();
        }

        public MainWindow()
        {
            _servoJoints[0] = new servoJoints(JointType.HipLeft, JointType.ShoulderLeft, JointType.ElbowLeft);
            _servoJoints[1] = new servoJoints(JointType.HipRight, JointType.ShoulderRight, JointType.ElbowRight);
            _servoJoints[2] = new servoJoints(JointType.ShoulderRight, JointType.ShoulderLeft, JointType.HandLeft);
            _servoJoints[3] = new servoJoints(JointType.ShoulderLeft, JointType.ShoulderRight, JointType.HandRight);
            _servoJoints[4] = new servoJoints(JointType.ShoulderLeft, JointType.ElbowLeft, JointType.HandLeft);
            _servoJoints[5] = new servoJoints(JointType.ShoulderRight, JointType.ElbowRight, JointType.HandRight);
            
            _servoValues[0] = new servoValues(2450, 2450, 750); // MIN, INIT, MAX *** MIN = MIN_SERVO_POSITION = MIN_ANGLE !!!
            _servoValues[1] = new servoValues(550, 550, 2250);
            _servoValues[2] = new servoValues(550, 1120, 2450);
            _servoValues[3] = new servoValues(2450, 1340, 550);
            _servoValues[4] = new servoValues(975, 2245, 2245);
            _servoValues[5] = new servoValues(2100, 900, 900);

            string filename = projectDirectory + servoValuesObject;
            servoValues[] svs = null;
            if (File.Exists(filename)) svs = DeSerializeObject<servoValues[]>(filename);
            if (svs != null && svs.Length == _servoValues.Length) _servoValues = svs;

            ManagementObjectSearcher searcher = new ManagementObjectSearcher("root\\WMI", "SELECT * FROM MSSerial_PortName");
            foreach (ManagementObject queryObj in searcher.Get())
            {
                if (queryObj["InstanceName"].ToString().Contains(SSC32_pnpId))
                {
                    string port = queryObj["PortName"].ToString();
                    if (initSerial(port))
                    {
                        new BackgroundWorker().DoWork += (s, e) =>
                        {
                            for (int i = _servoValues.Length - 1; i >= 0; i -= 2)
                            {
                                int servoSpeed = 500;
                                if (i == 0 || i == 1) servoSpeed = 2000;
                                string stringSpeed = "T"+servoSpeed;
                                if (sp != null && sp.IsOpen)
                                {
                                    //todo check position if reach go to next servo
                                    sp.WriteLine("#" + i + "P" + _servoValues[i].init + "#" + (i - 1) + "P" + _servoValues[i - 1].init + stringSpeed);
                                    Thread.Sleep(servoSpeed);
                                }
                            }
                        };
                    }
                    break;
                }
            }
            
            this.kinectSensor = KinectSensor.GetDefault();
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();
            this.bones = new List<Tuple<JointType, JointType>>();
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bodyColors = new List<Pen>();
            this.bodyColors.Add(new Pen(Brushes.Red, 2));
            this.bodyColors.Add(new Pen(Brushes.Green, 2));
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;
            this.kinectSensor.Open();
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;
            this.drawingGroup = new DrawingGroup();
            this.imageSource = new DrawingImage(this.drawingGroup);
            this.DataContext = this;
            this.InitializeComponent();
        }

        private void MouseHook_MouseAction(object sender, mEventArgs e)
        {
            if (e.Args == 0) pauseServoUpdates = !pauseServoUpdates;
            else
            {
                recordPosition = !recordPosition;
                if (recordPosition) timer = now();
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        public string StatusText
        {
            get
            {
                return this.statusText;
            }
            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
            MouseHook.Start();
            MouseHook.MouseAction += MouseHook_MouseAction;
        }

        private void MainWindow_Closing(object s, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }
            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }

            //todo check position if reach go to next servo
            var doneEvent = new AutoResetEvent(false);
            var bw = new BackgroundWorker();
            bw.DoWork += (s2, e2) =>
            {
                try
                {
                    if (!e.Cancel)
                    {
                        string filename = projectDirectory + servoValuesObject;
                        SerializeObject<servoValues[]>(_servoValues, filename);
                        for (int i = _servoValues.Length - 1; i >= 0; i -= 2)
                        {
                            int servoSpeed = 500;
                            if (i == 0 || i == 1) servoSpeed = 2000;
                            string stringSpeed = "T" + servoSpeed;
                            if (sp != null && sp.IsOpen)
                            {
                                sp.WriteLine("#" + i + "P" + _servoValues[i].init + "#" + (i - 1) + "P" + _servoValues[i - 1].init + stringSpeed);
                                Thread.Sleep(servoSpeed);
                            }
                        }
                        for (int i = _servoValues.Length - 1; i >= 0; i--) if (sp != null && sp.IsOpen) sp.WriteLine("#" + i + "L");
                        if (sp != null && sp.IsOpen) sp.Close();
                        if (sp != null)
                        {
                            sp.Dispose();
                            sp = null;
                        }
                    }
                }
                finally
                {
                    doneEvent.Set();
                }
            };
            bw.RunWorkerAsync();
            doneEvent.WaitOne();
        }

        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }
            if (dataReceived)
            {
                Body body = null;
                if (this.bodyTracked)
                {
                    if (this.bodies[this.bodyIndex].IsTracked)
                    {
                        body = this.bodies[this.bodyIndex];
                    }
                    else
                    {
                        bodyTracked = false;
                    }
                }
                if (!bodyTracked)
                {
                    for (int i = 0; i < this.bodies.Length; ++i)
                    {
                        if (this.bodies[i].IsTracked)
                        {
                            this.bodyIndex = i;
                            this.bodyTracked = true;
                            break;
                        }
                    }
                }
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    int penIndex;
                    if (sp != null && sp.IsOpen) penIndex = 1;
                    else penIndex = 0;
                    Pen drawPen = this.bodyColors[penIndex];
                    if (body != null && this.bodyTracked && body.IsTracked)
                    {
                        IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
                        Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();
                        foreach (JointType jointType in joints.Keys)
                        {
                            CameraSpacePoint position = joints[jointType].Position;
                            if (position.Z < 0)
                            {
                                position.Z = InferredZPositionClamp;
                            }
                            DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                            jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                        }
                        thisBody = bodies[bodyIndex];
                        this.servoUpdate(joints);
                        this.DrawBody(joints, jointPoints, dc, drawPen);

                        this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                        this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                    }
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            if (recordPosition && now() - timer > 1000 / recordingFPS) {
                string msg = "";
                for (int i = 0; i < _servoValues.Length; i++)
                {
                    if (_servoValues[i].last < 1000) msg += "0";
                    msg += _servoValues[i].last;
                    msg += " ";
                }
                msg += "\r\n";
                string filename = projectDirectory + recordingFile;
                var bw = new BackgroundWorker();
                bw.DoWork += (s, e) =>
                {
                    try
                    {
                        if (!e.Cancel)
                        {
                            File.AppendAllText(filename, msg);
                        }
                    }
                    catch (Exception e2) { }
                };
                bw.RunWorkerAsync();
                timer = now();
            }
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;
                TrackingState trackingState = joints[jointType].TrackingState;
                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }
                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }
            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        public void servoUpdate(IReadOnlyDictionary<JointType, Joint> joints)
        {
            for (int i = 0; i < _servoValues.Length; i++) {
                Joint joint0 = joints[_servoJoints[i].start];
                Joint joint1 = joints[_servoJoints[i].mid];
                Joint joint2 = joints[_servoJoints[i].end];
                if (joint0.TrackingState == TrackingState.Tracked && joint1.TrackingState == TrackingState.Tracked && joint2.TrackingState == TrackingState.Tracked)
                {
                    if (i < 2 || i > 3) _servoValues[i].setAngle(Angle3D(_servoJoints[i]));
                    _servoValues[i].Update(i);
                }
            }
        }

        public double Angle3D(servoJoints sj)
        {
            Body body = this.bodies[this.bodyIndex];
            Joint _start = body.Joints[sj.start];
            Joint _middle = body.Joints[sj.mid];
            Joint _end = body.Joints[sj.end];
            Vector3D start = new Vector3D(_start.Position.X, _start.Position.Y, _start.Position.Z);
            Vector3D middle = new Vector3D(_middle.Position.X, _middle.Position.Y, _middle.Position.Z);
            Vector3D end = new Vector3D(_end.Position.X, _end.Position.Y, _end.Position.Z);
            Vector3D _vector1 = middle - start;
            Vector3D _vector2 = middle - end;
            _vector1.Normalize();
            _vector2.Normalize();
            double _angle = Vector3D.AngleBetween(_vector1, _vector2);
            return _angle;
        }

        public static double map(double x, double in_min, double in_max, double out_min, double out_max)
        {
            double a = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            if (out_min < out_max)
            {
                if (a < out_min) a = out_min;
                else if (a > out_max) a = out_max;
            }
            else
            {
                if (a > out_min) a = out_min;
                else if (a < out_max) a = out_max;
            }
            return a;
        }

        public void SerializeObject<T>(T serializableObject, string fileName)
        {
            if (serializableObject == null) { return; }
            try
            {
                XmlDocument xmlDocument = new XmlDocument();
                XmlSerializer serializer = new XmlSerializer(serializableObject.GetType());
                using (MemoryStream stream = new MemoryStream())
                {
                    serializer.Serialize(stream, serializableObject);
                    stream.Position = 0;
                    xmlDocument.Load(stream);
                    xmlDocument.Save(fileName);
                    stream.Close();
                }
            }
            catch (Exception ex) { }
        }

        public T DeSerializeObject<T>(string fileName)
        {
            if (string.IsNullOrEmpty(fileName)) { return default(T); }
            T objectOut = default(T);
            try
            {
                XmlDocument xmlDocument = new XmlDocument();
                xmlDocument.Load(fileName);
                string xmlString = xmlDocument.OuterXml;
                using (StringReader read = new StringReader(xmlString))
                {
                    Type outType = typeof(T);
                    XmlSerializer serializer = new XmlSerializer(outType);
                    using (XmlReader reader = new XmlTextReader(read))
                    {
                        objectOut = (T)serializer.Deserialize(reader);
                        reader.Close();
                    }
                    read.Close();
                }
            }
            catch (Exception ex) { }
            return objectOut;
        }

        public struct servoValues
        {
            public bool debug;
            public int min;
            public int init;
            public int max;
            public int last;
            public double minAngle;
            public double angle;
            public double maxAngle;

            public servoValues(int a, int b, int c, bool d = false)
            {
                debug = d;
                min = a;
                init = b;
                max = c;
                last = b;
                minAngle = 360;
                angle = 0;
                maxAngle = 0;
            }

            public void setAngle(double a)
            {
                angle = a;
                minAngle += 0.01;
                maxAngle -= 0.01;
                if (a < minAngle) minAngle = a;
                else if (a > maxAngle) maxAngle = a;
            }

            public void Update(int i)
            {
                int pos = last;
                if (i < 2 || i > 3)
                {
                    if (minAngle + 10 > maxAngle) return;
                    pos = (int)map(angle, minAngle, maxAngle, min, max);                    
                }
                else
                {
                    if (thisBody == null) return;
                    servoJoints sj = _servoJoints[i];
                    Joint _start = thisBody.Joints[sj.start];
                    Joint _middle = thisBody.Joints[sj.mid];
                    Joint _end = thisBody.Joints[sj.end];
                    double start = _start.Position.X;
                    double mid = _middle.Position.X;
                    double end = _end.Position.X;
                    double maxLen = ((mid - start) * 2) + mid;
                    bool reverse = false;
                    if (start > mid)
                    {
                        maxLen = mid - ((start - mid) * 2);
                        reverse = true;
                    }
                    if ((mid < end && !reverse) || (mid > end && reverse)) pos = (int)map(end, mid, maxLen, init, max);
                    else pos = (int)map(end, start, mid, min, init);
                }
                if (last + 10 < pos || last - 10 > pos)
                {
                    last = pos;
                    if (sp != null && sp.IsOpen && !pauseServoUpdates)
                    {
                        string s = "#" + i + "P" + pos + speed;
                        sp.WriteLine(s);
                    }
                }
                if (debug) Console.WriteLine("#" + i + ": " + last);
            }
        }

        public struct servoJoints
        {
            public JointType start;
            public JointType mid;
            public JointType end;

            public servoJoints(JointType a, JointType b, JointType c)
            {
                start = a;
                mid = b;
                end = c;
            }
        }

        public long now()
        {
            return DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
        }
    }
}
