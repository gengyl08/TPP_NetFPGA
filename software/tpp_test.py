from scapy import *
from scapy.all import *

LOAD_REG_HOPMEM = 0
STORE_HOPMEM_REG = 2

padding_head = 'A'*64

instruction = chr(LOAD_REG_HOPMEM)

reg_addr = chr(2)

padding_tail = chr(0)*100

data = padding_head + instruction + reg_addr + padding_tail

sendp(Ether(data), iface='nf3')
