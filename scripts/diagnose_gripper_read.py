#!/usr/bin/env python3
"""Read-only Modbus timing diagnostic for the Doosan flange gripper port."""

import argparse
import time

import rclpy
from dsr_msgs2.srv import (
    FlangeSerialClose,
    FlangeSerialOpen,
    FlangeSerialRead,
    FlangeSerialWrite,
)
from rclpy.node import Node


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def fc03(start, count):
    data = bytes([1, 3, start >> 8, start & 0xFF, count >> 8, count & 0xFF])
    crc = crc16(data)
    return list(data + bytes([crc & 0xFF, crc >> 8]))


class Diagnostic(Node):
    def __init__(self):
        super().__init__("diagnose_gripper_read")
        prefix = "/dsr01/gripper"
        self.open = self.create_client(FlangeSerialOpen, f"{prefix}/flange_serial_open")
        self.close = self.create_client(FlangeSerialClose, f"{prefix}/flange_serial_close")
        self.write = self.create_client(FlangeSerialWrite, f"{prefix}/flange_serial_write")
        self.read = self.create_client(FlangeSerialRead, f"{prefix}/flange_serial_read")

    def call(self, client, request, timeout=6.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError(f"service timeout: {client.srv_name}")
        return future.result()

    def close_port(self):
        request = FlangeSerialClose.Request()
        request.port = 1
        return self.call(self.close, request)

    def run(self, delay, start_register, count):
        self.close_port()
        request = FlangeSerialOpen.Request()
        request.port = 1
        request.baudrate = 57600
        request.bytesize = 8
        request.parity = 0
        request.stopbits = 1
        opened = self.call(self.open, request)
        if not opened.success:
            raise RuntimeError("flange serial open failed")

        request = FlangeSerialWrite.Request()
        request.port = 1
        request.data = fc03(start_register, count)
        written = self.call(self.write, request)
        print(f"request={bytes(request.data).hex()} write_success={written.success}")
        time.sleep(max(0.0, delay))

        chunks = []
        for index in range(6):
            request = FlangeSerialRead.Request()
            request.port = 1
            request.timeout = 0.1
            response = self.call(self.read, request)
            data = bytes(response.data)
            print(
                f"read[{index}] success={response.success} size={response.size} "
                f"data={data.hex() or '-'}")
            if data:
                chunks.append(data)
            time.sleep(0.01)
        self.close_port()
        print(f"combined={b''.join(chunks).hex() or '-'}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--delay-sec", type=float, default=0.0)
    parser.add_argument("--start-register", type=int, default=287)
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--execute-read", action="store_true")
    args = parser.parse_args()
    if not args.execute_read:
        raise SystemExit("READ-ONLY DRY RUN: add --execute-read to query the gripper.")
    rclpy.init()
    node = Diagnostic()
    try:
        node.run(args.delay_sec, args.start_register, args.count)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
