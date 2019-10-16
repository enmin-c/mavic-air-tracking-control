using System;
using System.Threading;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Runtime.InteropServices;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using DJI.WindowsSDK;
using Windows.UI.Xaml.Media.Imaging;
using DJIVideoParser;
using System.Threading.Tasks;
using OpenCvSharp;


// The Blank Page item template is documented at https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace DJIWSDKDemo
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        private DJIVideoParser.Parser videoParser;
        public int isFly__ = 0;
        public int target_X = 0,target_Y = 0;
        public bool target_location_changed = false;
        //public double x0, y0;
        public double g = 9.8, m = 0.43;
        public MainPage()
        {
            this.InitializeComponent();
            DJISDKManager.Instance.SDKRegistrationStateChanged += Instance_SDKRegistrationEvent;
            //Replace app key with the real key registered. Make sure that the key is matched with your application's package id.
            DJISDKManager.Instance.RegisterApp("670540341457ae29e878a15d");
            
        }

        private void Starttakeoff_Click(object sender, RoutedEventArgs e)
        {
            DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartTakeoffAsync();
            GimbalAngleRotation camera_angle = new GimbalAngleRotation();
            camera_angle.mode = GimbalAngleRotationMode.ABSOLUTE_ANGLE;
            camera_angle.pitch = -90;
            camera_angle.pitchIgnored = false;
            camera_angle.rollIgnored = true;
            camera_angle.yawIgnored = true;
            camera_angle.duration = 1;
            DJISDKManager.Instance.ComponentManager.GetGimbalHandler(0, 0).RotateByAngleAsync(camera_angle);
            //var pos0 = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetAircraftLocationAsync();
            //x0 = pos0.Result.value.Value.latitude;
            //y0 = pos0.Result.value.Value.longitude;
            var z0 = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetAltitudeAsync();
            System.Diagnostics.Debug.WriteLine("z0:{0}", z0);
        }

        private void FLY_PID()
        {
            float pid_x = 0, pid_y = 0;
            float kp_x = (float)0.01, ki_x = (float)0, kd_x = (float)-0.1;
            float kp_y = (float)-0.01, ki_y = (float)0, kd_y = (float)0.1;
            float ki_sum_x = 0,ki_sum_y = 0;
            float t_x = 0, t_y = 0;
            int jud;
            float t_t = (float)0.02;
            jud = isFly__;
            while (jud == isFly__)
            {
                var velocity = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetVelocityAsync();
                if(target_location_changed)
                {
                    t_x = target_X;
                    t_y = target_Y;
                }
                else
                {
                    t_x = (float)(target_X + velocity.Result.value.Value.x * 0.03 * 1.2 / 0.025);
                    t_y = (float)(target_Y + velocity.Result.value.Value.y * 0.03 * 1.2 / 0.025);
                }
                if (t_x < 40 && t_x > -40)
                {
                    t_x = 0;
                }
                if (t_y < 40 && t_y > -40)
                {
                    t_y = 0;
                }

                ki_sum_x = ki_sum_x + t_x * t_t; 
                if (ki_sum_x > 1000)
                {
                    ki_sum_x = 1000;
                }
                else if (ki_sum_x < -1000)
                {
                    ki_sum_x = -1000;
                }
                pid_x = kp_x * t_x + ki_x * ki_sum_x + kd_x * (float)velocity.Result.value.Value.x;
                if (pid_x > 0.3)
                {
                    pid_x = (float)0.3;
                }
                else if (pid_x < -0.3)
                {
                    pid_x = (float)-0.3;
                }

                ki_sum_y = ki_sum_y + t_y * t_t;
                if (ki_sum_y > 1000)
                {
                    ki_sum_y = 1000;
                }
                else if (ki_sum_y < -1000)
                {
                    ki_sum_y = -1000;
                }
                pid_y = kp_y * t_y + ki_y * ki_sum_y + kd_y * (float)velocity.Result.value.Value.y;
                if (pid_y > 0.3)
                {
                    pid_y = (float)0.3;
                }
                else if (pid_y < -0.3)
                {
                    pid_y = (float)-0.3;
                }
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)pid_y, (float)pid_x);//yaw pitch roll
                System.Threading.Thread.Sleep(30);
            }
        }

        private void FlyCtrl()
        {
            /*
            float x1 = (float)0.2, y1 = (float)0.2;
            float xx, yy;
            int jud;
            jud = isFly__;
            while (jud == isFly__)
            {
                if (target_Y > 40)
                {
                    var velocity = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetVelocityAsync();
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)-0.5, (float)0);//yaw pitch roll
                    System.Threading.Thread.Sleep(20);
                    xx = -1 * (float)velocity.Result.value.Value.x * x1;
                    yy = -1 * (float)velocity.Result.value.Value.y * y1;
                    if (xx > 0.3) xx = (float)0.3;
                    else if (xx < -0.3) xx = (float)-0.3;
                    if (yy > 0.3) yy = (float)0.3;
                    else if (yy < -0.3) yy = (float)-0.3;
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, xx, (float)0);
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)0, yy);
                    System.Threading.Thread.Sleep(20);
                }
                else if (target_Y < -40)
                {
                    var velocity = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetVelocityAsync();
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)0.5, (float)0);//yaw pitch roll
                    System.Threading.Thread.Sleep(20);
                    xx = -1 * (float)velocity.Result.value.Value.x * x1;
                    yy = -1 * (float)velocity.Result.value.Value.y * y1;
                    if (xx > 0.3) xx = (float)0.3;
                    else if (xx < -0.3) xx = (float)-0.3;
                    if (yy > 0.3) yy = (float)0.3;
                    else if (yy < -0.3) yy = (float)-0.3;
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, xx, (float)0);
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)0, yy);
                    System.Threading.Thread.Sleep(20);
                }

                if (target_X > 40)
                {
                    var velocity = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetVelocityAsync();
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)0, (float)0.5);//yaw pitch roll
                    System.Threading.Thread.Sleep(20);
                    xx = -1 * (float)velocity.Result.value.Value.x * x1;
                    yy = -1 * (float)velocity.Result.value.Value.y * y1;
                    if (xx > 0.3) xx = (float)0.3;
                    else if (xx < -0.3) xx = (float)-0.3;
                    if (yy > 0.3) yy = (float)0.3;
                    else if (yy < -0.3) yy = (float)-0.3;
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, xx, (float)0);
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)0, yy);
                    System.Threading.Thread.Sleep(20);
                }
                else if (target_X < -40)
                {
                    var velocity = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetVelocityAsync();
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)0, (float)-0.5);//yaw pitch roll
                    System.Threading.Thread.Sleep(20);
                    xx = -1 * (float)velocity.Result.value.Value.x * x1;
                    yy = -1 * (float)velocity.Result.value.Value.y * y1;
                    if (xx > 0.3) xx = (float)0.3;
                    else if (xx < -0.3) xx = (float)-0.3;
                    if (yy > 0.3) yy = (float)0.3;
                    else if (yy < -0.3) yy = (float)-0.3;
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, xx, (float)0);
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)0, yy);
                    System.Threading.Thread.Sleep(20);
                }
            }*/
            pos_ctrl(20);
        }

        private void vel_ctrl(float x,float y,int t)
        {/*
            forward pitch >0 x>0
            right roll >0 y>0
            */
            float xx, yy;
            float xp = (float)0.1, xd = (float)0.001; 
            float yp = (float)0.1, yd = (float)0.001;
            var velocity = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetVelocityAsync();
            var att = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetAttitudeAsync();
            xx = (x - (float)velocity.Result.value.Value.x) * xp + (float)(xd * (0-g*Math.Atan(att.Result.value.Value.pitch)));
            yy = (y - (float)velocity.Result.value.Value.y) * yp + (float)(yd * (0-g*Math.Atan(att.Result.value.Value.roll)));
            if (xx > 0.3) xx = (float)0.3;
            else if (xx < -0.3) xx = (float)-0.3;
            if (yy > 0.3) yy = (float)0.3;
            else if (yy < -0.3) yy = (float)-0.3;
            DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, xx, yy); //yaw pitch roll
            System.Threading.Thread.Sleep(t);
        }

        private void pos_ctrl(int t)
        {
            /*float ex, ey, ez, thro, pitch_d, roll_d;
            float kxp = (float)0.1, kxd = (float)1;
            float kyp = (float)0.1, kyd = (float)1;
            float kzp = (float)0.1, kzd = (float)1;
            var pos = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetAircraftLocationAsync();
            var v = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetVelocityAsync();
            var z = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetAltitudeAsync();
            ex = kxp * (x_d - (float)pos.Result.value.Value.latitude) + kxd * (float)v.Result.value.Value.x;
            ey = kyp * (y_d - (float)pos.Result.value.Value.longitude) + kyd * (float)v.Result.value.Value.y;
            ez = kzp * (z_d - (float)z.Result.value.Value.value);
            //thro = (float)(m*Math.Sqrt(ex*ex+ey*ey+(ez+g)*(ez+g)));*/
            float ex, ey,t_x,t_y;
            float kxp = (float)0.1, kxd = (float)0.001, kyp = (float)0.1, kyd = (float)0.001;
            int jud;
            jud = isFly__;
            while (jud == isFly__)
            {
                var velocity = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetVelocityAsync();
                if (target_location_changed)
                {
                    t_x = target_X;
                    t_y = target_Y;
                }
                else
                {
                    t_x = (float)(target_X - velocity.Result.value.Value.x * 0.03 * 1.2 / 0.025);
                    t_y = (float)(target_Y - velocity.Result.value.Value.y * 0.03 * 1.2 / 0.025);
                }
                if (Math.Abs(t_x) < 100)
                {
                    t_x = 0;
                }
                if (Math.Abs(t_y) < 100)
                {
                    t_y = 0;
                }
                ex = (float)(kxp * t_x - kxd * (velocity.Result.value.Value.x * 0.03 * 1.2 / 0.025));
                ey = (float)(kyp * t_y - kyd * (velocity.Result.value.Value.y * 0.03 * 1.2 / 0.025));
                vel_ctrl(ex, ey, t);
            }
        }


        private void Startflycontrol_Click(object sender, RoutedEventArgs e)
        {
            isFly__ = isFly__ + 1;
            Thread fly_ctrl = new Thread(new ThreadStart(FlyCtrl));
            fly_ctrl.Start();
        }

        private void Keep_position()
        {
            float x1 = (float)0.6, y1 = (float)0.6;
            float xx, yy;
            int jud;
            jud = isFly__;
            while (jud == isFly__)
            {
                var velocity = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetVelocityAsync();
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)0, (float)0);//yaw pitch roll
                System.Threading.Thread.Sleep(10);
                xx = -1 * (float)velocity.Result.value.Value.x * x1;
                yy = -1 * (float)velocity.Result.value.Value.y * y1;
                if (xx > 0.3) xx = (float)0.3;
                else if (xx < -0.3) xx = (float)-0.3;
                if (yy > 0.3) yy = (float)0.3;
                else if (yy < -0.3) yy = (float)-0.3;
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, xx, (float)0);
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)0, yy);
                System.Threading.Thread.Sleep(30);
            }
        }

        private void Keepposition_Click(object sender, RoutedEventArgs e)
        {
            isFly__ = isFly__ + 1;
            Thread fly_ctrl = new Thread(new ThreadStart(Keep_position));
            fly_ctrl.Start();
        }

        private void Go_forward()
        {
            float x1 = (float)0.3, y1 = (float)0.3;
            float xx, yy;
            int jud;
            jud = isFly__;
            while (jud == isFly__)
            {
                var velocity = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetVelocityAsync();
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)0.5, (float)0);//yaw pitch roll
                System.Threading.Thread.Sleep(20);
                xx = -1 * (float)velocity.Result.value.Value.x * x1;
                yy = -1 * (float)velocity.Result.value.Value.y * y1;
                if (xx > 0.3) xx = (float)0.3;
                else if (xx < -0.3) xx = (float)-0.3;
                if (yy > 0.3) yy = (float)0.3;
                else if (yy < -0.3) yy = (float)-0.3;
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, xx, (float)0);
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)0, yy);
                System.Threading.Thread.Sleep(10);
            }
        }

        private void Goforward_Click(object sender, RoutedEventArgs e)
        {
            isFly__ = isFly__ + 1;
            Thread fly_ctrl = new Thread(new ThreadStart(Go_forward));
            fly_ctrl.Start();
        }

        private void Go_back()
        {
            float x1 = (float)0.3, y1 = (float)0.3;
            float xx, yy;
            int jud;
            jud = isFly__;
            while (jud == isFly__)
            {
                var velocity = DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetVelocityAsync();
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)-0.5, (float)0);//yaw pitch roll
                System.Threading.Thread.Sleep(20);
                xx = -1 * (float)velocity.Result.value.Value.x * x1;
                yy = -1 * (float)velocity.Result.value.Value.y * y1;
                if (xx > 0.3) xx = (float)0.3;
                else if (xx < -0.3) xx = (float)-0.3;
                if (yy > 0.3) yy = (float)0.3;
                else if (yy < -0.3) yy = (float)-0.3;
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, xx, (float)0);
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)0, (float)0, (float)0, yy);
                System.Threading.Thread.Sleep(10);
            }
        }

        private void Goback_Click(object sender, RoutedEventArgs e)
        {
            isFly__ = isFly__ + 1;
            Thread fly_ctrl = new Thread(new ThreadStart(Go_back));
            fly_ctrl.Start();
        }

        private async void Startlanding_Click(object sender, RoutedEventArgs e)
        {
            isFly__ = isFly__ + 1;
            await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartAutoLandingAsync();
            var x = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetIsLandingConfirmationNeededAsync();
            while (!(x.value.Value.value))
            {
                x = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetIsLandingConfirmationNeededAsync();
            }
            await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).ConfirmLandingAsync();
        }

        //Callback of SDKRegistrationEvent
        private async void Instance_SDKRegistrationEvent(SDKRegistrationState state, SDKError resultCode)
        {
            if (resultCode == SDKError.NO_ERROR)
            {
                System.Diagnostics.Debug.WriteLine("Register app successfully.");

                //Must in UI thread
                await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, async () =>
                {
                    //Raw data and decoded data listener
                    if (videoParser == null)
                    {
                        videoParser = new DJIVideoParser.Parser();
                        videoParser.Initialize(delegate (byte[] data)
                        {
                            //Note: This function must be called because we need DJI Windows SDK to help us to parse frame data.
                            return DJISDKManager.Instance.VideoFeeder.ParseAssitantDecodingInfo(0, data);
                        });
                        //Set the swapChainPanel to display and set the decoded data callback.
                        videoParser.SetSurfaceAndVideoCallback(0, 0, swapChainPanel, ReceiveDecodedData);
                        DJISDKManager.Instance.VideoFeeder.GetPrimaryVideoFeed(0).VideoDataUpdated += OnVideoPush;
                    }
                    //get the camera type and observe the CameraTypeChanged event.
                    DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).CameraTypeChanged += OnCameraTypeChanged;
                    var type = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).GetCameraTypeAsync();
                    OnCameraTypeChanged(this, type.value);
                });

            }
            else
            {
                System.Diagnostics.Debug.WriteLine("SDK register failed, the error is: ");
                System.Diagnostics.Debug.WriteLine(resultCode.ToString());
            }
        }

        //raw data
        void OnVideoPush(VideoFeed sender, byte[] bytes)
        {
            videoParser.PushVideoData(0, 0, bytes, bytes.Length);
        }

        //Decode data. Do nothing here. This function would return a bytes array with image data in RGBA format.
        int frameNum = 0;
        int num_frame = 0;
        Int64 itime;
        OpenCvSharp.Window wid = new OpenCvSharp.Window("image",WindowMode.AutoSize);
        async void ReceiveDecodedData(byte[] data, int width, int height)
        {
            /*
            num_frame++;
            var itimee = System.DateTime.Now.Ticks;
            if (num_frame > 1)
            {
                System.Diagnostics.Debug.WriteLine((itimee - itime) / 10000);
            }
            itime = itimee;
            target_location_changed = false;
            if (frameNum > 3)
            {
                var itime1 = System.DateTime.Now.Ticks;
                
                frameNum = 0;
     
                int i, j;
                int max_x = 0, max_y = 0, num = 0;
                int r, g, b,tol;
                for (i = 0; i < height; i++)
                {
                    for (j = 0; j < width; j++)
                    {
                        r = data[(i * width + j) * 4];
                        g = data[(i * width + j) * 4 + 1];
                        b = data[(i * width + j) * 4 + 2];
                        tol = r + g + b; 
                        if (tol> 120 && r > 0.8 * tol )
                        {
                            num = num + 1;
                            max_x = max_x + j;
                            max_y = max_y + i;
                        }
                    }
                }
                var itime2 = System.DateTime.Now.Ticks;
                System.Diagnostics.Debug.WriteLine((itime2-itime1)/10000);
                var image = new Mat(height, width, MatType.CV_8UC4, data);
                if (num > 0)
                {
                    target_X = max_x / num - width / 2;
                    target_Y = max_y / num - height / 2;
                    image.Circle(max_x / num, max_y / num, 20, Scalar.Black, 5);
                    System.Diagnostics.Debug.WriteLine("(target_X,target_Y):{0},{0}", target_X, target_Y);
                }
                else
                {
                    target_X = target_Y = 0;
                }
                target_location_changed = true;
                wid.ShowImage(image);
                //Cv2.ImWrite("D:\\Windows-FPVDemo-master\\img\\test.jpg", image);

                var itime3 = System.DateTime.Now.Ticks;
                System.Diagnostics.Debug.WriteLine((itime3-itime2)/10000);
            }
            else
            {
                frameNum++;
            }
            */
        }

        //We need to set the camera type of the aircraft to the DJIVideoParser. After setting camera type, DJIVideoParser would correct the distortion of the video automatically.
        private void OnCameraTypeChanged(object sender, CameraTypeMsg? value)
        {
            if (value != null)
            {
                switch (value.Value.value)
                {
                    case CameraType.MAVIC_2_ZOOM:
                        this.videoParser.SetCameraSensor(AircraftCameraType.Mavic2Zoom);
                        break;
                    case CameraType.MAVIC_2_PRO:
                        this.videoParser.SetCameraSensor(AircraftCameraType.Mavic2Pro);
                        break;
                    default:
                        this.videoParser.SetCameraSensor(AircraftCameraType.Others);
                        break;
                }

            }
        }

        private async void StartShootPhoto_Click(object sender, RoutedEventArgs e)
        {
            
            if (DJISDKManager.Instance.ComponentManager != null)
            {
                var retCode = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).StartShootPhotoAsync();
                if (retCode != SDKError.NO_ERROR)
                {
                    OutputTB.Text = "Failed to shoot photo, result code is " + retCode.ToString();
                } else
                {
                    OutputTB.Text = "Shoot photo successfully";
                }
            }
            else
            {
                OutputTB.Text = "SDK hasn't been activated yet.";
            }
        }

        private async void StartRecordVideo_Click(object sender, RoutedEventArgs e)
        {
            if (DJISDKManager.Instance.ComponentManager != null)
            {
                var retCode = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).StartRecordAsync();
                if (retCode != SDKError.NO_ERROR)
                {
                    OutputTB.Text = "Failed to record video, result code is " + retCode.ToString();
                }
                else
                {
                    OutputTB.Text = "Record video successfully";
                }
            }
            else
            {
                OutputTB.Text = "SDK hasn't been activated yet.";
            }
        }

        private async void StopRecordVideo_Click(object sender, RoutedEventArgs e)
        {
            if (DJISDKManager.Instance.ComponentManager != null)
            {
                var retCode = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).StopRecordAsync();
                if (retCode != SDKError.NO_ERROR)
                {
                    OutputTB.Text = "Failed to stop record video, result code is " + retCode.ToString();
                }
                else
                {
                    OutputTB.Text = "Stop record video successfully";
                }
            }
            else
            {
                OutputTB.Text = "SDK hasn't been activated yet.";
            }
        }

        private async void SetCameraWorkModeToShootPhoto_Click(object sender, RoutedEventArgs e)
        {
            SetCameraWorkMode(CameraWorkMode.SHOOT_PHOTO);
        }

        private void SetCameraModeToRecord_Click(object sender, RoutedEventArgs e)
        {
            SetCameraWorkMode(CameraWorkMode.RECORD_VIDEO);
        }

        private async void SetCameraWorkMode(CameraWorkMode mode)
        {
            if (DJISDKManager.Instance.ComponentManager != null)
            {
                CameraWorkModeMsg workMode = new CameraWorkModeMsg
                {
                    value = mode,
                };
                var retCode = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).SetCameraWorkModeAsync(workMode);
                if (retCode != SDKError.NO_ERROR)
                {
                    OutputTB.Text = "Set camera work mode to " + mode.ToString() + "failed, result code is " + retCode.ToString();
                }
            }
            else
            {
                OutputTB.Text = "SDK hasn't been activated yet.";
            }
        }
    }
}
