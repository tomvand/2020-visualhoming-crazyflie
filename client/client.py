import tkinter as tk
import zmq
import subprocess


class CFZmq(object):
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://localhost:1213")

        msg = {
            'version': 1,
            'cmd': 'toc',
            'name': '',
            'value': ''
        }
        self.socket.send_json(msg)
        print(self.socket.recv_json())

    def set_parameter(self, name, value):
        msg = {
            'version': 1,
            'cmd': 'set',
            'name': name,
            'value': value
        }
        self.socket.send_json(msg)
        print(self.socket.recv_json())


class Client(tk.Tk):
    def __init__(self):
        super().__init__()

        # Launch cfclient
        self.p_cfclient = subprocess.Popen(['cfclient'])

        # Start ZMQ
        self.zmq = CFZmq()

        # Make bigger
        self.geometry('300x200-1-1')

        # Keep on top
        self.attributes('-topmost', True)

        # Set up grid
        self.columnconfigure(0, weight=3)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        # Create land button
        self.land_btn = tk.Button(self, text='Land now', bg='yellow', activebackground='yellow')
        self.land_btn['command'] = self.land_btn_clicked
        self.land_btn.grid(row=0, column=0, sticky=tk.NSEW)

        # Create enable and kill buttons
        self.enable_kill_f = tk.Frame(self)
        self.enable_kill_f.grid(row=0, column=1, sticky=tk.NSEW)
        self.enable_kill_f.rowconfigure(0, weight=1)
        self.enable_kill_f.rowconfigure(1, weight=1)
        self.enable_kill_f.columnconfigure(0, weight=1)

        self.enable_btn = tk.Button(self.enable_kill_f, text='Enable', bg='green', activebackground='green')
        self.enable_btn['command'] = self.enable_btn_clicked
        self.enable_btn.grid(row=0, column=0, sticky=tk.NSEW)

        self.kill_btn = tk.Button(self.enable_kill_f, text='Kill', bg='red', activebackground='red')
        self.kill_btn['command'] = self.kill_btn_clicked
        self.kill_btn.grid(row=1, column=0, sticky=tk.NSEW)

    def __del__(self):
        try:
            self.p_cfclient.terminate()
        except:
            pass

    def land_btn_clicked(self):
        self.zmq.set_parameter('vh.SW_ENABLE', 0)

    def enable_btn_clicked(self):
        self.zmq.set_parameter('vh.SW_ENABLE', 1)

    def kill_btn_clicked(self):
        self.zmq.set_parameter('vh.SW_KILL', 1)


if __name__ == '__main__':
    app = Client()
    app.mainloop()
