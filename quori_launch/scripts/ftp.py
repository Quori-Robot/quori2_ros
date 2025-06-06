#! /usr/bin/env python3

from ftplib import FTP as ftp, all_errors, error_perm
import bincopy
import sys
import CRC32_STM32 as crc32
import os
import io

USER = ""
PASSWORD = ""
HOST_IP = "10.2.99.111"

class FTPClient(object):
    def __init__(self):
        try:
            self.client = ftp(HOST_IP, USER, PASSWORD)
            self.client.login()
        except all_errors as e:
            print(f"FTP connection/login error: {e}")
            raise
        self.sent = 0
        self.length = 0

    def put(self, file_name, file, length):
        self.sent = 0
        self.length = length
        try:
            self.client.storbinary(f"STOR {file_name}", file, 8192, self.progress)
        except error_perm as e:
            print(f"FTP permission error during upload of {file_name}: {e}")
            raise
        except all_errors as e:
            print(f"FTP error during upload of {file_name}: {e}")
            raise
        except Exception as e:
            print(f"Unexpected error during upload of {file_name}: {e}")
            raise

    def progress(self, data):
        self.sent += len(data)
        try:
            percent = (100 * self.sent / self.length)
        except ZeroDivisionError:
            percent = 100.0
        print(f"sent {percent:5.1f}%, [{self.sent}/{self.length}] bytes")

def usage():
    print("Usage: ftp.py <file> \n\tfile: filename to send\n")


if __name__ == "__main__":
    try:
        if len(sys.argv) < 2:
            usage()
            sys.exit(-1)

        file_name = sys.argv[1]


        ftp = FTPClient()
        with open(file_name, "rb") as fh:
            f_len = os.fstat(fh.fileno()).st_size
            ftp.put(file_name, fh, f_len)  

        sys.exit(0)

    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(-3)