'''
Created on 2019. 4. 4.

@author: JSk
'''
import os
import sys
import socket
import platform

##
# @brief get CPU Architecture
def machine():
    """Return type of machine."""
    if os.name == 'nt' and sys.version_info[:2] < (2,7):
        return os.environ.get("PROCESSOR_ARCHITEW6432",
               os.environ.get('PROCESSOR_ARCHITECTURE', ''))
    else:
        return platform.machine()


##
# @brief get ip addess
def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]
