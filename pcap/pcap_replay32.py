#!/usr/bin/python
import time, socket, sys, numpy as np, socket

# ImportError: No module named numpy
# 	http://stackoverflow.com/questions/7818811/import-error-no-module-named-numpy

def print_help_and_exit():
	print('Usage: replay_cap.py <xyz.pcap> [--start=<N>|--end=<N>]')
	sys.exit()


if len(sys.argv) < 2:
	print_help_and_exit()


fn=sys.argv[1]
try:
	f=open(fn, 'rb')
except:
	print_help_and_exit()

START=0
END  =1000000

try:
	for arg in sys.argv[2:]:
		if arg.startswith('--start='):
			START=int(arg[8:])
		elif arg.startswith('--end='):
			END=int(arg[6:])
except:
	print_help_and_exit()
	
	
s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ADDR=('127.0.0.1', 2368)
T=time.time()

packet_counter=-1
global_header=f.read(24)
while 1:
	try:
		packet_header=np.fromstring(f.read(16), 'uint32')
		if len(packet_header)<1:
			break
		timestamp=packet_header[0]+packet_header[1]/1e6
		
		if packet_counter==-1:
			start_time = timestamp

		duration = timestamp - start_time

		size=packet_header[2]
		if size!=1248:
			f.read(size)
			continue
		packet=f.read(1248)
		data=packet[42:]

		packet_counter+=1
		if packet_counter<START:
			continue
		
		time.sleep(max(0,timestamp-T))
		T=timestamp
		
		s.sendto(data, ADDR)
		
		if packet_counter>END:
			break
	except KeyboardInterrupt:
		break

print('# of Packet replayed: ' + str(packet_counter))
print('duration ' + str(int(duration/60)) + ':' + str(int(duration)%60))
