import Tkinter as tk
import tkFileDialog
import tkMessageBox
import threading
import time, socket, sys, numpy as np, socket


# print_help_and_exit
def print_help_and_exit():
    tkMessageBox.showerror(title='Pcap Player',message='It is not a pcap file.')
    

# select file
def select_file():
    file_name = tkFileDialog.askopenfilename(filetypes=(('pcap file','*.pcap'),
            ('all file','*.*')))
    filename.set(file_name)


# start play
def start_play():
    # get the name of pcap file
    fn=filename.get()
    # try to open the file
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

    # open socket
    s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ADDR=('127.0.0.1', 2368)
    T=time.time()
     
    packet_counter=-1
    global_header=f.read(24)
    global scale_value
    while 1:
    	try:
            if pause_flag == 'stop':
                break
            # read packet_header
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
            while pause_flag=='pause':
                print ('pause')
                pass

            # show on scale
            scale_value = packet_counter

    	except KeyboardInterrupt:
    		break
    # print some information
    print('# of Packet replayed: ' + str(packet_counter))
    print('duration ' + str(int(duration/60)) + ':' + str(int(duration)%60))

# call start play
def call_start_play():
    global pause_flag
    global num_of_thread
    pause_flag = 'run'
    if num_of_thread < 1:
        start_button_thread = threading.Thread(target=start_play,
                name='start_button_thread')
        start_button_thread.start()

def pause():
    global pause_flag
    if pause_flag !='stop' and pause_flag != 'pause':
        pause_flag = 'pause'
    elif pause_flag == 'pause':
        pause_flag = 'running'
    global pause_bg
    global pause_button
    if pause_bg == 'red':
        pause_bg = 'gray'
        pause_button.configure(bg=pause_bg)
    else :
        pause_bg = 'red'
        pause_button.configure(bg=pause_bg)

def layout():
    # main window layout
    main_window = tk.Tk()
    main_window.title('Pcap Player')
    main_window.geometry('500x200')
    
    # update scale
    def set_scale():
        global scale, scale_value
        scale.set(scale_value)
        main_window.after(1000,set_scale)
    def destroy():
        global main_window
        global pause_flag
        pause_flag = 'stop'
        main_window.destroy()
    # file name label
    file_name_label = tk.Label(main_window,text='FILE NAME',bg='gray',
            font=('New Times Roman',16))
    file_name_label.place(x=30,y=10,anchor='nw')
    
    # file name display
    global filename
    filename = tk.StringVar()
    show_filename_entry= tk.Entry(main_window,textvariable=filename,width=33,
            bg='gray',font=('New Times Roman',16))
    show_filename_entry.place(x=130,y=10)
    
    # Scale
    global scale_value, scale
    scale_value = 0
    scale = tk.Scale(main_window,from_=0,to=500000,resolution=1,width=30,
            orient=tk.HORIZONTAL,length=430)
    scale.place(x=30,y=50)
    set_scale()
    
        
    # Start Button
    global num_of_thread
    num_of_thread = 0
    start_button = tk.Button(main_window,text='START',font=('New Time Roman',16),
            bg='gray',command=call_start_play)
    start_button.place(x=30,y=110)
    
    # Pause Button
    global pause_bg, pause_flag, pause_button
    pause_flag = 'pause'
    pause_bg = 'gray'
    pause_button = tk.Button(main_window,text='PAUSE',font=('New Time Roman',16),
            bg=pause_bg,command=pause)
    pause_button.place(x=180,y=110)

    
    # Select File Button
    select_button = tk.Button(main_window,text='SELECT FILE',font=('New Time Roman',16),
       bg='gray',command=select_file)
    select_button.place(x=330,y=110)
    
    main_window.mainloop()
    if pause_flag == 'stop':
        print ('stop')
        main_window().destroy()
        

if __name__=='__main__':
    try:
        layout()
    except KeyboardInterrupt:
        global pause_flag
        pause_flag = 'stop'
