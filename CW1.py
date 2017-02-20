import network
import time
import machine
import ujson as json

bus_addr = 30  # set up your bus address here
from machine import Pin

switch = Pin(15, Pin.IN)  # create switch pin input


def pin_change(p):
    global reset
    reset = 1
    print('reset begin')


def do_connect():   # set the connection
    global client
    ap_if = network.WLAN(network.AP_IF)
    ap_if.active(False)
    sta_if = network.WLAN(network.STA_IF)
    sta_if.active(True)
    sta_if.connect('EEERover', 'exhibition')
    from umqtt.simple import MQTTClient
    client = MQTTClient('TinyMotion', '192.168.0.10')
    client.connect()


def data_reading(MSB,LSB):  # this function does convert output data of the registers to real unit (miliGauss) under default setup.
    reading = MSB * 256 + LSB
    if (MSB & 0x80 == 0x80):  # detect if this number is negative
        reading = reading - 65536  # 2's complement calculation for 16bit binary number (65536=2^16)
    reading = int(reading * 0.92)  # under default setup, 1LSB=0.92miliGauss (see datasheet)
    return reading


def sub_cb(topic, msg):
    print ((topic, msg))
    time_message = json.loads(msg)
    timeo = time_message['date']
    rtc = machine.RTC()
    rtc.datetime((int(timeo[0:4]), int(timeo[5:7]), int(timeo[8:10]), 0, int(timeo[11:13]), int(timeo[14:16]),int(timeo[17:19]), int(timeo[20:22])))
    print('time has been set up successfully:',rtc.datetime())  # synchronise the real time clock with the date published on the broker


# it is noted that the forth point of the date tuple is set to be 0. Because this point indicates weekday (0-6 correspond to Mon-Sun) and is automatically calibrated by the RTC system even if we input a wrong value.


def timeset():
    from machine import Pin, PWM
    LED_timeset = PWM(Pin(14), freq=2,duty=512)  # flash LED indicates the timeset is not finished #flash frequency=1Hz, dutycycle=50%
    client.set_callback(sub_cb)  # indicates that function 'sub_cb' will be executed after messsage has been received
    client.subscribe('esys/time')
    client.wait_msg()  # blocking wait receiving datetime from the broker
    LED_timeset.deinit()  # turn off PWM on the pin


def main():
    global reset
    reset = 0
    from machine import Pin, I2C
    i2c = I2C(scl=Pin(5), sda=Pin(4), freq=100000)
    LED_warning = Pin(12, Pin.OUT)
    rtc = machine.RTC()  # get system local time
    warning = 0

    i2c.writeto_mem(bus_addr, 02, b'\x01')  # first measurement is not used because we find that the first measurement occationally fails
    time.sleep(1)

    i2c.writeto_mem(bus_addr, 02, b'\x01')  # edit the mode register of the sensor to enable it to work
    data = i2c.readfrom_mem(30, 03, 6)  # read the second measurement as the initial position of the sensor # registers 03-08 are the output registers of the sensor

    datax = data_reading(MSB=data[0], LSB=data[1])

    dataz = data_reading(MSB=data[2], LSB=data[3])

    datay = data_reading(MSB=data[4], LSB=data[5])

    print ('initial measurement:', 'X=', datax, 'Y=', datax, 'Z=', dataz, 'time:', rtc.datetime())

    while (warning == 0 and reset == 0):
        i2c.writeto_mem(bus_addr, 02, b'\x01')
        data = i2c.readfrom_mem(30, 03, 6)

        datax_next = data_reading(MSB=data[0], LSB=data[1])

        dataz_next = data_reading(MSB=data[2], LSB=data[3])

        datay_next = data_reading(MSB=data[4], LSB=data[5])

        changex = datax_next - datax
        changey = datay_next - datay
        changez = dataz_next - dataz
        print ('X=', datax_next, 'Y=', datax_next, 'Z=', dataz_next, 'time:', rtc.datetime())
        payload_before_warning = json.dumps({'X=': datax_next, 'Y=': datay_next, 'Z=': dataz_next, 'time at:': rtc.datetime()})
        client.publish('esys/PentaQ/test', bytes(payload_before_warning, 'utf-8'))

        if abs(changex) >= 100 or abs(changey) >= 100 or abs(changez) >= 100:  # sensitivity set change threshold to be 100 miliGauss
            warning_time = rtc.datetime()
            warning = 1
            LED_warning.high()
            payload_after_warning = json.dumps({'warning at:': warning_time})
            print('warning at:', warning_time, switch.value())
        time.sleep(0.5)  # measurement frequency

    while (warning == 1 and reset == 0):
        print('warning at:', warning_time, 'reset=', reset)
        client.publish('esys/PentaQ', bytes(payload_after_warning, 'utf-8'))
        time.sleep(0.5)  # warning frequency


if __name__ == '__main__':

    do_connect()
    timeset()
    switch.irq(trigger=Pin.IRQ_RISING,handler=pin_change)  # interrupt happens at rising edge of the switch. it goes to 'pin_change()' where we set 'reset' to be 1 in order to jump out of the two while loops in 'main()'
    var = 1
    while (var == 1):
        main()
