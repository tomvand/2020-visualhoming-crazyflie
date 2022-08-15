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
        columns = 5
        rows = 2
        grid_size = 100
        width = columns * grid_size
        height = rows * grid_size
        self.geometry(f'{width}x{height}-1-1')

        # Keep on top
        self.attributes('-topmost', True)

        # Set up grid
        for i in range(columns):
            self.columnconfigure(i, weight=1)
        for i in range(rows):
            self.rowconfigure(i, weight=1)

        # Create land button
        self.land_btn = tk.Button(self, text='Land now', bg='yellow', activebackground='yellow')
        self.land_btn['command'] = self.land_btn_clicked
        self.land_btn.grid(row=0, rowspan=2, column=0, columnspan=2, sticky=tk.NSEW)

        # Create enable and kill buttons
        self.enable_btn = tk.Button(self, text='Enable', bg='green', activebackground='green')
        self.enable_btn['command'] = self.enable_btn_clicked
        self.enable_btn.grid(row=0, column=2, sticky=tk.NSEW)

        self.kill_btn = tk.Button(self, text='Kill', bg='red', activebackground='red')
        self.kill_btn['command'] = self.kill_btn_clicked
        self.kill_btn.grid(row=1, column=2, sticky=tk.NSEW)

        # Create experiment buttons
        self.experiment_button = []
        for i in range(4):
            label = f'Exp {i}'
            if i == 0:
                label = 'Idle'
            self.experiment_button = tk.Button(self, text=label)
            self.experiment_button['command'] = lambda: self.experiment_btn_clicked(i)
            self.experiment_button.grid(row=i // 2, column=3 + (i % 2), sticky=tk.NSEW)

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

    def experiment_btn_clicked(self, index):
        self.zmq.set_parameter('vh.sw_experiment', index)


if __name__ == '__main__':
    app = Client()
    app.mainloop()
