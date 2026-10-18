"""Offline decoder for observed RK_GAS01 reply logs; never connects to hardware.

Layout/scaling derived from /home/root/robot debug symbols and ParsePacket.
Outputs software values only: physical units and sensor calibration are unknown.
"""
import argparse
import binascii
import json
import re
from pathlib import Path


def normalize(value, power):
    if power == 1:
        return value * 10
    if power in (3, 4):
        divisor = 10 if power == 3 else 100
        return (abs(value) // divisor) * (-1 if value < 0 else 1)
    return value


def decode_reply(hex_text):
    packet = bytes.fromhex(hex_text)
    if len(packet) != 12 or packet[:4] != bytes.fromhex('5B5B0C01'):
        raise ValueError('Only observed 12-byte query replies are supported')
    channel, error, power = packet[4:7]
    crc_ok = (binascii.crc_hqx(packet[:-1], 0) & 255) == packet[-1]
    result = dict(packet_hex=packet.hex().upper(), channel=channel,
                  error_code=error, error_name={0: 'success', 1: 'nocfg', 2: 'nodata'}.get(error, 'unknown'),
                  gas_power=power, crc_ok=crc_ok, physical_unit=None,
                  sensor_validity='not_verified')
    result['channel_name'] = {3: 'temperature_humidity', 5: 'oxygen', 10: 'carbon_monoxide'}.get(channel, 'unverified')
    # Reject failed/error replies before presenting usable software values.
    result['software_values'] = None
    if crc_ok and error == 0 and power in (0, 1, 2, 3, 4):
        if channel == 3:
            raw = [int.from_bytes(packet[7:9], 'little', signed=True),
                   int.from_bytes(packet[9:11], 'little')]
        elif channel in (5, 10):
            raw = [int.from_bytes(packet[7:11], 'little', signed=True)]
        else:
            return result
        result['raw_values'] = raw
        result['software_values'] = [normalize(v, power) for v in raw]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    args = parser.parse_args()
    for line in args.log.read_text(errors='replace').splitlines():
        if 'PtlGasSensor_RK_GAS01.cpp' not in line:
            continue
        match = re.search(r'RECV\s*\[([0-9A-Fa-f]+)\]', line)
        if match:
            try:
                result = decode_reply(match[1])
            except ValueError as exc:
                result = {'decode_error': str(exc), 'packet_hex': match[1]}
            result['source_log_time'] = line[:19]
            print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
