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

        # configure grid
        rows = 2
        num_experiments = 16
        experiment_cols = int(num_experiments / 2 + 0.999)
        columns = 3 + experiment_cols
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
        for i in range(num_experiments):
            label = f'Exp {i}'
            if i == 0:
                label = 'Idle'
            elif i == 1:
                label = 'Fake\nhome'
            elif i == 2:
                label = 'SS\nsingle'
            elif i == 3:
                label = 'Odo'
            elif i == 4:
                label = 'Both\nseq'
            elif i == 5:
                label = 'RMSE\nexp'
            elif i == 6:
                label = 'INS\nHDG'
            elif i == 7:
                label = 'Corr\nboth'
            elif i == 8:
                label = 'Corr\nodo'
            elif i == 9:
                label = 'U\nboth'
            elif i == 10:
                label = 'U\nodo'
            elif i == 11:
                label = "Corr\nss's"
            elif i == 12:
                label = 'S\nodo'
            elif i == 13:
                label = 'S\nboth'
            elif i == 14:
                label = "S\nss's"
            self.experiment_button = tk.Button(self, text=label)
            self.experiment_button['command'] = lambda x=i: self.experiment_btn_clicked(x)
            self.experiment_button.grid(row=i // experiment_cols, column=3 + (i % experiment_cols), sticky=tk.NSEW)

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
