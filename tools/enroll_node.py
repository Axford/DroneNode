#!/usr/bin/env python3
"""Pair the physically connected DroneNode with a cloud enrollment claim."""

import argparse
from getpass import getpass
import json
from pathlib import Path
import re
import ssl
from time import monotonic
from urllib.parse import urlsplit
from urllib.request import HTTPSHandler, HTTPRedirectHandler, Request, build_opener

import serial


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        raise ValueError("Enrollment endpoint redirected")


def console(port, command, response):
    port.reset_input_buffer()
    port.write((command + '\n').encode('ascii'))
    deadline = monotonic() + 18
    while monotonic() < deadline:
        line = port.readline().decode('ascii', errors='ignore').strip()
        if line == 'enroll error':
            raise ValueError('Device rejected ' + command.split(' ', 2)[1])
        if re.fullmatch(response, line):
            return line
    raise TimeoutError('No device enrollment reply')


def enroll(args, claim):
    endpoint = urlsplit(args.endpoint)
    if endpoint.scheme != 'https' or not endpoint.hostname or endpoint.username or endpoint.password or endpoint.path not in ('', '/') or endpoint.query or endpoint.fragment:
        raise ValueError('Use an HTTPS origin for the enrollment endpoint')
    if not re.fullmatch(r'[a-f0-9]{64}', claim) or not 1 <= args.node <= 254 or not re.fullmatch(r'[a-zA-Z0-9_-]{1,200}', args.domain):
        raise ValueError('Enter a valid claim and node address')
    ca = args.ca.read_text(encoding='ascii')
    if len(ca) > 2048 or not ca.startswith('-----BEGIN CERTIFICATE-----'):
        raise ValueError('Supply the PEM trust anchor for the node')
    context = ssl.create_default_context(cadata=ca)
    opener = build_opener(HTTPSHandler(context=context), NoRedirect())
    port = serial.Serial(port=None, baudrate=115200, timeout=.25, write_timeout=2, exclusive=True)
    port.dtr = False
    port.rts = False
    port.port = args.serial
    with port:
        # Opening the USB adapter can reset the board. Wait for the console and radio.
        deadline = monotonic() + 55
        while True:
            try:
                response = console(port, 'enroll key', r'enroll public [0-9]{1,3} [a-f0-9]{128}')
                break
            except (ValueError, TimeoutError):
                if monotonic() >= deadline:
                    raise
        _, _, device_address, public = response.split()
        if int(device_address) != args.node:
            raise ValueError('The USB device reports a different node address')
        body = json.dumps(dict(domain=args.domain, id=args.node, claim=claim, publicKey=public)).encode()
        req = Request(args.endpoint.rstrip('/') + '/node-auth/enroll', body, headers={'Content-Type': 'application/json'})
        with opener.open(req, timeout=12) as reply:
            result = json.loads(reply.read(4097))
        identity = result.get('data', {}).get('enrollmentId', '')
        if result.get('success') is not True or not re.fullmatch(r'[a-f0-9]{32}', identity):
            raise ValueError('Enrollment response was incomplete')
        profile = '|'.join((args.endpoint.rstrip('/'), args.domain, str(args.node), identity, ca.replace('\n', '~')))
        console(port, 'enroll begin', 'enroll ok')
        for offset in range(0, len(profile), 16):
            console(port, 'enroll part ' + profile[offset:offset + 16], 'enroll ok')
        console(port, 'enroll save', 'enroll ok')
        port.write(b'reset\n')
    print('Paired node ' + str(args.node) + ' and requested restart')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--endpoint', required=True)
    parser.add_argument('--domain', required=True)
    parser.add_argument('--node', type=int, required=True)
    parser.add_argument('--ca', type=Path, required=True)
    args = parser.parse_args()
    try:
        enroll(args, getpass('One-time enrollment claim: ').strip())
    except (OSError, ValueError, serial.SerialException) as error:
        parser.exit(1, f'Enrollment failed: {error}\n')


if __name__ == '__main__':
    main()
