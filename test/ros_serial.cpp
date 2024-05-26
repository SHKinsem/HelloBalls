#include<ros/ros.h>
#include<serial/serial.h>
#include<std_msgs/String.h>

#include<iostream>
#include<string>
#include<sstream>

using namespace std;


/**************************************************************************
函数功能：将数据经由串口发送出去
入口参数：[serial::Serial &ser]：              串口类名称；
                      [std::string &serial_msg]:      发送的数据数组；
返  回  值：无
说        明：无
**************************************************************************/
int serial_write(serial::Serial &ser, std::string &serial_msg)
{
    ser.write(serial_msg);
    return 0;
}

/**************************************************************************
函数功能：将数据经由串口发送出去
入口参数：[serial::Serial &ser]：              串口类名称；
                      [std::string &serial_msg]:      接收的数据数组；
返  回  值：无
说        明：无
**************************************************************************/
int serial_read(serial::Serial &ser, std::string &result)
{
    result = ser.read( ser.available() );
    return 0;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv,"my_serial_port");
    ros::NodeHandle n;

    //创建一个serial类
   serial::Serial ser;

    //初始化串口相关设置
   ser.setPort("/dev/ttyUSB0");         //设置打开的串口名称
   ser.setBaudrate(115200);                //设置串口的波特率
   serial::Timeout to = serial::Timeout::simpleTimeout(1000);           //创建timeout
   ser.setTimeout(to);                           //设置串口的timeout

    //打开串口
    try
    {
        ser.open();         //打开串口
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");           //打开串口失败，打印信息
        return -1;
    }

    //判断串口是否成功打开
    if( ser.isOpen() )
    { 
        ROS_INFO_STREAM("Serial Port initialized. \n");         //成功打开串口，打印信息
    }
    else
    {
        return -1;
    }


    ros::Rate loop_rate(50);
	
	//data 为发送数据
    //result 为接收数据
  	std::string data, result;
    int func(0);
    
    cout << "Please input function number:" << endl;

    while( ros::ok() )
    {
      	//从键盘中读取键入数据
      	cout << "Your function number is: ";
        cin >> func;
        
  /*****************************************************************************
 * 以下逻辑可以按照你自己的写，主要工作是根据键盘键入的数据，为 data 赋值
 *****************************************************************************/
     	//  switch (func)
        // {
        //     case 0:     data = "A 800 456\r\n";           break;
        //     case 1:     data = "B 1200 456\r\n";        break;
        //     case 2:     data = "C 1600 456\r\n";        break;
        //     case 3:     data = "D 1800 456\r\n";        break;
        //     default:    ROS_ERROR_STREAM("No this function number!!!");     break;
        // }

        // //串口写数据
        // serial_write(ser, data);
        // cout << " the data write to serial is :  " << data.c_str();
        // //串口读数据
        // serial_read(ser, result);
        // cout << " the data read from serial is : " << result.c_str();
        // cout << endl;

        data = int(func);
        //串口写数据
        serial_write(ser, data);
        cout << " the data write to serial is :  " << data.c_str();
        //串口读数据
        serial_read(ser, result);
        cout << " the data read from serial is : " << result.c_str();
        cout << endl;

    }

	ser.close();
    return 0;
}