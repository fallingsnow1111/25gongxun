import sensor, image,time
from pyb import LED,Pin
from pyb import UART

uart = UART(3, 115200)
uart.init(115200, bits = 8, parity = None, stop = 1)     #8位数据位，无校验位， 1位停止位

led = LED(1)
led.on()
p_out = Pin('P8', Pin.OUT_PP)#设置p_out为输出引脚
p_out.high()#设置p_out引脚为高
p_out.low()#设置p_out引脚为低

data0 = bytearray([0xAA,0x07,0X00,0xB4])
data1 = bytearray([0xAA,0x07,0X01,0xB4])
data2 = bytearray([0xAA,0x07,0X02,0xB4])
data3 = bytearray([0xAA,0x07,0X03,0xB4])

green_threshold=[(61, 74, -66, -23, 10, 57)]
red_threshold=[(31, 55, 12, 63, 12, 50)]
blue_threshold=[(34, 47, -22, 7, -42, -17)]

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # can be QVGA on M7...
sensor.skip_frames(30)
sensor.set_auto_gain(False) # must turn this off to prevent image washout...
time.sleep_ms(500)

while(True):
    global code
    img = sensor.snapshot()
    blob1 = img.find_blobs(red_threshold,roi=(66,20,191,154),pixels_threshold=5,area_threshold=5,merge=True,margin=2)
    blob2 = img.find_blobs(green_threshold,roi=(66,20,191,154),pixels_threshold=5,area_threshold=5,merge=True,margin=2)
    blob3 = img.find_blobs(blue_threshold,roi=(66,20,191,154),pixels_threshold=5,area_threshold=5,merge=True,margin=2)
    if blob1:
        if blob1[0][4]>500:
            img.draw_cross(int(blob1[0][0] + blob1[0][2]/2), int(blob1[0][1]+ blob1[0][3]/2))
            uart.write(data1)
            print(blob1[0][4])
    elif blob2:
        if blob2[0][4]>500:
            img.draw_cross(int(blob2[0][0] + blob2[0][2]/2), int(blob2[0][1]+ blob2[0][3]/2))
            uart.write(data2)
            print(blob2[0][4])
    elif blob3:
        if blob3[0][4]>500:
            img.draw_cross(int(blob3[0][0] + blob3[0][2]/2), int(blob3[0][1]+ blob3[0][3]/2))
            uart.write(data3)
            print(blob3[0][4])
    else:
        uart.write(data0)
