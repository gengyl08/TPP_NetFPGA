from scapy import *
from scapy.all import *

data = 'A'*64 + chr(0)+chr(3)+chr(0)*100
sendp(Ether(data), iface='nf3')
