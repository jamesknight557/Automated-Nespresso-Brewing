import rclpy
from rclpy.node import Node
from GripperTest.srv import GripperTest


class GripperTestServer(Node):

    def __init__(self):
        super().__init__('gripper_test_server')
        self.srv = self.create_service(GripperTest, 'gripper_test', self.handle_gripper_test)

    def handle_gripper_test(self, request, response):
        # Replace the provided code with your gripper testing logic

        # sudo chmod 666 /dev/ttyUSB0


        import serial

        import time

        import binascii

        ser = serial.Serial(port='/dev/ttyUSB0',baudrate=115200,timeout=1,

        parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)

        counter = 0

        while counter < 1:

            counter = counter + 1

            ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")

            data_raw = ser.readline()

            print(data_raw)

            data = binascii.hexlify(data_raw)

            print ("Response 1 ", data)

            time.sleep(0.01)

            

            ser.write(b"\x09\x03\x07\xD0\x00\x01\x85\xCF")

            data_raw = ser.readline()

            print(data_raw)

            data = binascii.hexlify(data_raw)

            print ("Response 2 ", data)

            time.sleep(1)

        count = 0 

        while count < 1:

            count = count + 1

            print ("Close gripper")

            ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")

            data_raw = ser.readline()

            print(data_raw)

            data = binascii.hexlify(data_raw)

            print ("Response 3 ", data)

            time.sleep(2)

        

        print ("Open gripper")

        ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")

        data_raw = ser.readline()

        print(data_raw)


        print ("Response 4 ", data)

        time.sleep(2)
        ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")

        data_raw = ser.readline()

        print(data_raw)


        print ("Response 4 ", data)

        time.sleep(2)




        # For demonstration, returning a successful status
        response.status = 1
        return response

def main(args=None):
    rclpy.init(args=args)
    server = GripperTestServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
