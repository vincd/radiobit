import radio
from microbit import *


radio.on()
# radio.config(data_rate=radio.RATE_2MBIT, channel=32, raw=False, queue=10)

channel = 1
found_my_mice = False

TIMEOUT = 2000
RED_MICE = [0x41, 0x75, 0xc4, 0x9d, 0xa4]
# RED_MICE_CHANNELS = range(2, 74+1, 3) # 2402 - 2474, 3MHz spacing
RED_MICE_CHANNELS = [5, 8, 14, 17, 32, 35, 41, 44, 62, 65, 71, 74]
RED_MICE_ADDR = RED_MICE[0]<<24 | RED_MICE[1]<<16 | RED_MICE[2]<<8 | RED_MICE[3]
RED_MICE_GROUP = RED_MICE[4]



channel_index = 5
channel = RED_MICE_CHANNELS[channel_index]
radio.config(data_rate=radio.RATE_2MBIT, channel=channel, queue=20)
radio.sniff_on()

print('[+] Scanning channels for my mice on channel %s!' % channel)

while True:

    last = running_time()
    # prev_pid = 0xFF   

    while not found_my_mice:
        now = running_time()

        if (now - last) > TIMEOUT:
            last = now

            channel_index += 1
            if channel_index >= len(RED_MICE_CHANNELS):
                channel_index = 0       

            # channel += 1
            # if channel > 100:
            #     channel = 1

            channel = RED_MICE_CHANNELS[channel_index]
            print('[-] Set channel %s' % channel)
            # radio.config(channel=channel)
            # radio.sniff_on()

            radio.config(channel=channel)
            radio.sniff_on()
            # radio.esb()

        pkt = radio.sniff()
        # pkt = radio.receive_bytes()
        if pkt is not None:
            size = len(pkt)
            # pid = pkt[0]
            # addr = 'toto' # ':'.join(['%02x'%c for c in pkt[:5]])
            pkt_size = pkt[0] & 0x3F
            payload = ''.join(['%02x '%c for c in pkt[0:6 + pkt_size]])
            print('[%03d] [0/%03d] %s' % (channel, len(pkt), payload))
            if pkt_size == 2:
                print('[Payload] %s' % (''.join(['%02x '%c for c in pkt[8:38]])))
            if size > (38 * 1):
                pkt_size = pkt[38*1] & 0x3F
                payload = ''.join(['%02x '%c for c in pkt[38:38 + 6 + pkt_size]])
                print('[%03d] [1/%03d] %s' % (channel, len(pkt), payload))
            if size > (38 * 2):
                pkt_size = pkt[38*2] & 0x3F
                payload = ''.join(['%02x '%c for c in pkt[76:76 + 6 + pkt_size]])
                print('[%03d] [2/%03d] %s' % (channel, len(pkt), payload))
            if size > (38 * 3):
                pkt_size = pkt[38*3] & 0x3F
                payload = ''.join(['%02x '%c for c in pkt[114:114 + 6 + pkt_size]])
                print('[%03d] [3/%03d] %s' % (channel, len(pkt), payload))
            if size > (38 * 4):
                pkt_size = pkt[38*4] & 0x3F
                payload = ''.join(['%02x '%c for c in pkt[152:152 + 6 + pkt_size]])
                print('[%03d] [4/%03d] %s' % (channel, len(pkt), payload))
            # prev_pid = pid
            # if is_my_mice(pkt):
            #     found_my_mice = FTrue
            #     sleep(1000)

            # del pid
            # del addr
            # del payload
        del pkt

    # pkt = radio.sniff()
    # if pkt is not None:
    #     parse_paket(pkt)

    # del pkt