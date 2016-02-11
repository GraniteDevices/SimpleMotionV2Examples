# SimpleMotionV2Examples
Collection of sample programs for SimpleMotion V2 library. Can be used as with all SimpleMotion V2 compatible motor drives such as Granite Devices ARGONand IONI. This project is work in progress and this notice will be removed when all examples are finished.

Projets are made with Qt. For instructions of compilation, see:
http://granitedevices.com/wiki/Installing_Qt_SDK_from_scratch

Point-to-point motion example
-----------------------------
This app implements simple case where simple setpoint targets can be sent to up to two axis. Motion of axis are not synchronized and they independently try to reach the target setpoints.

This example does not do anything yet.

Buffered motion stream example
------------------------------
This app implements a buffered motion stream with multiple drives. Simple sinusoidal setpoint is sent to multiple drives and drives are kept in synchronism infinitely by synchronizing the buffer clock between the devices.

This project is based on the development version of SMV library. This example is the most complete and works already.

Real-time control example
-------------------------
Not yet started. This will use the new fast SM command.

Screenshots
-----------

Buffered motion stream example

![screenshot](https://raw.githubusercontent.com/GraniteDevices/SimpleMotionV2Examples/master/BufferedMotionStreamExample/screenshot.png)
