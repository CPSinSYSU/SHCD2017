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
# fileObject.read();
# 	>0: # of bytes
#	<0 or none: the whole file
# in write_file_header(): sizeof(global_header)==24
global_header=f.read(24)
while 1:
	try:
		# in handler function: sizeof(local_header)=16
		"""
		struct pcap_pkthdr {
			struct timeval ts;	/* time stamp */
			bpf_u_int32 caplen;	/* length of portion present */
			bpf_u_int32 len;	/* length this packet (off wire) */
		};

		struct timeval
		  {
		    __time_t tv_sec;		/* Seconds.  */
		    __suseconds_t tv_usec;	/* Microseconds.  */
		  };

		__time_t & __suseconds_t following:
			/* X32 kernel interface is 64-bit.  */
			#if defined __x86_64__ && defined __ILP32__
			# define __SYSCALL_SLONG_TYPE	__SQUAD_TYPE
			# define __SYSCALL_ULONG_TYPE	__UQUAD_TYPE
			#else
			# define __SYSCALL_SLONG_TYPE	__SLONGWORD_TYPE
			# define __SYSCALL_ULONG_TYPE	__ULONGWORD_TYPE
			#endif
		__SQUAD_TYPE: __quad_t --> int
		__UQUAD_TYPE: __u_quad_t --> unsigned int
		__SLONGWORD_TYPE: long int
		__ULONGWORD_TYPE: unsigned long int
		"""
		packet_header=np.fromstring(f.read(24), 'uint32')
		# packet_header=np.fromstring(f.read(32), 'uint64')
		if len(packet_header)<1:
			break
		timestamp=packet_header[0]+packet_header[2]/1e6
		
		if packet_counter==-1:
			start_time = timestamp

		duration = timestamp - start_time
		# print('duration ' + str(int(duration/60)) + ':' + str(int(duration)%60))

		size=packet_header[4]
		# print("size=" + str(size))
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
