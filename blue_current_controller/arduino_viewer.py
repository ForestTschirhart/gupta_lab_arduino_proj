#!/usr/bin/env python3
import time
import sys
import serial
import numpy as np
import matplotlib.pyplot as plt
import threading
from queue import Queue, Empty
import re
import os

PORT = '/dev/ttyACM1'
BAUD = 250000
N = 500                # must match Arduino N
FRAME_INTERVAL = 1.0   # seconds between traces (changed to 1 second)

def read_exact(ser, n, timeout=1.0):
    buf = bytearray()
    start = time.time()
    while len(buf) < n and (time.time() - start) < timeout:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf)

def consume_ascii_ack(ser, marker="R_RECEIVED", timeout=0.5):
    start = time.time()
    while (time.time() - start) < timeout:
        line = ser.readline()
        if not line:
            break
        try:
            s = line.decode('ascii', errors='ignore').strip()
        except Exception:
            s = ""
        if s:
            print("ARDUINO:", s)
        if s == marker:
            return True
    return False

class ArduinoScope:
    def __init__(self):
        self.ser = None
        self.running = False
        self.command_queue = Queue()
        self.plot_queue = Queue()
        self.ser_lock = threading.Lock()
        self.fig = None
        self.ax = None
        self.line = None
        self.peak_scatter = None
        self.last_peaks = []         # list of (peak_index, val, pos)
        # stats lines
        self.mean_line = None
        self.plus_line = None
        self.minus_line = None
    
        # persistent initialize stats (set by "initialize" command)
        self.init_mean_line = None
        self.init_plus_line = None
        self.init_minus_line = None
        self.init_stats = (None, None)

        
    def connect(self):
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.5)
        except Exception as e:
            print("Failed to open serial port:", e)
            sys.exit(1)
        
        print(f"Connected to {self.ser.port}")
        time.sleep(1.0)
        self.ser.reset_input_buffer()
        
    def setup_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 6))
        x = np.arange(N)
        self.line, = self.ax.plot(x, np.zeros(N), '-b', linewidth=0.8)
        self.ax.set_xlim(0, N)
        self.ax.set_ylim(0, 4096)
        self.ax.set_xlabel('Sample')
        self.ax.set_ylabel('ADC Value (12-bit)')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('Arduino Scope - Live (Type commands in terminal)')
        # scatter for peaks (initially empty)
        self.peak_scatter = self.ax.scatter([], [], c='r', marker='x', s=60, label='Peaks')
        # horizontal stat lines
        self.mean_line = self.ax.axhline(0, color='magenta', lw=1.2, label='Mean')
        self.plus_line = self.ax.axhline(0, color='magenta', lw=0.8, ls='--', label='+Std')
        self.minus_line = self.ax.axhline(0, color='magenta', lw=0.8, ls='--', label='-Std')
        # persistent init-stat lines (hidden until set)
        self.init_mean_line = self.ax.axhline(0, color='green', lw=1.2, label='Init Mean', visible=False)
        self.init_plus_line = self.ax.axhline(0, color='green', lw=0.8, ls='--', label='Init +Std', visible=False)
        self.init_minus_line = self.ax.axhline(0, color='green', lw=0.8, ls='--', label='Init -Std', visible=False)
        self.ax.legend(loc='upper right')
        
    def send_command(self, command, timeout=0.5):
        """Send a command to Arduino with thread safety.

        timeout: how long (seconds) to wait for ASCII response lines from the device.
        """
        with self.ser_lock:
            try:
                self.ser.write(command.encode() + b'\n')
                self.ser.flush()
                # Read any ASCII response (short window)
                response = []
                start = time.time()
                while (time.time() - start) < float(timeout):
                        line = self.ser.readline()
                        if not line:
                            # timed out for this iteration; continue until overall timeout elapses
                            continue
                        try:
                            s = line.decode('ascii', errors='ignore').strip()
                            if s:
                                response.append(s)
                        except:
                            # ignore decode errors and continue waiting
                            continue
                return response
            except Exception as e:
                print(f"Error sending command: {e}")
                return []
    
    def get_trace(self, silent=False):
        """Get a trace from Arduino - returns numpy array or None"""
        with self.ser_lock:
            try:
                self.ser.write(b'R\n')
                self.ser.flush()
                
                # Consume ASCII ack silently for continuous updates
                start = time.time()
                while (time.time() - start) < 0.25:
                    line = self.ser.readline()
                    if not line:
                        break
                    try:
                        s = line.decode('ascii', errors='ignore').strip()
                        if not silent and s:
                            print(f"Arduino: {s}")
                        if s == "R_RECEIVED":
                            break
                    except:
                        break
                
                # Read binary data
                raw = read_exact(self.ser, N * 2, timeout=1.0)
                if len(raw) == N * 2:
                    y = np.frombuffer(raw, dtype='<u2').astype(np.int32)
                    if y.max() == 0 or y.max() > 65500:
                        y = y.byteswap()

                    # --- Read any trailing ASCII lines (Peaks info) ---
                    start_tail = time.time()
                    tail_lines = []
                    old_timeout = self.ser.timeout
                    try:
                        self.ser.timeout = 0.05
                        while (time.time() - start_tail) < 0.25:
                            line = self.ser.readline()
                            if not line:
                                break
                            try:
                                s = line.decode('ascii', errors='ignore').strip()
                            except:
                                s = ""
                            if s:
                                tail_lines.append(s)
                    finally:
                        self.ser.timeout = old_timeout

                    # Parse peaks if present.
                    peaks = []
                    if tail_lines:
                        if not silent:
                            print("Arduino (tail):")
                            for tl in tail_lines:
                                print("  ", tl)
                        m = re.search(r'Peaks found:\s*(\d+)', "\n".join(tail_lines))
                        if m:
                            for l in tail_lines:
                                pm = re.match(r'Peak\s+(\d+):\s*val=(\d+)\s+pos=(\d+)', l)
                                if pm:
                                    peaks.append((int(pm.group(1)), int(pm.group(2)), int(pm.group(3))))
                            if peaks and not silent:
                                print(f"Parsed {len(peaks)} peaks:")
                                for p in peaks:
                                    print(f"  Peak {p[0]} val={p[1]} pos={p[2]}")
                    self.last_peaks = peaks
                    return y
                else:
                    if not silent:
                        print(f"Incomplete trace: {len(raw)} bytes")
                    return None
            except Exception as e:
                if not silent:
                    print(f"Error getting trace: {e}")
                return None

    def get_stats(self):
        """Request stats from Arduino ('S' command). Returns (mean, std) or (None,None)"""
        resp = self.send_command('S')
        if not resp:
            return None, None
        joined = "\n".join(resp)
        m = re.search(r'currentMean=\s*([+-]?[0-9]*\.?[0-9]+)\s*currentStd=\s*([+-]?[0-9]*\.?[0-9]+)', joined)
        if m:
            try:
                mean = float(m.group(1))
                std = float(m.group(2))
                return mean, std
            except:
                return None, None
        # fallback: try individual lines
        mean = None; std = None
        for line in resp:
            mm = re.search(r'currentMean=\s*([+-]?[0-9]*\.?[0-9]+)', line)
            ss = re.search(r'currentStd=\s*([+-]?[0-9]*\.?[0-9]+)', line)
            if mm: mean = float(mm.group(1))
            if ss: std = float(ss.group(1))
        return mean, std

# ...existing code...
    def set_init_stats(self, mean, std):
        """Called in main thread to set persistent initialize mean/std lines"""
        self.init_stats = (mean, std)
        if mean is None:
            if self.init_mean_line:
                self.init_mean_line.set_visible(False)
                self.init_plus_line.set_visible(False)
                self.init_minus_line.set_visible(False)
        else:
            if self.init_mean_line:
                self.init_mean_line.set_visible(True)
                self.init_mean_line.set_ydata([mean, mean])
                self.init_plus_line.set_visible(True)
                self.init_plus_line.set_ydata([mean + (std or 0), mean + (std or 0)])
                self.init_minus_line.set_visible(True)
                self.init_minus_line.set_ydata([mean - (std or 0), mean - (std or 0)])
# ...existing code...

    def scope_thread(self):
        """Continuous scope data acquisition thread"""
        while self.running:
            t0 = time.time()
            
            # Get trace silently for continuous updates
            y = self.get_trace(silent=True)
            mean = None; std = None
            if y is not None:
                # request current stats (best-effort, non-blocking short window)
                try:
                    mean, std = self.get_stats()
                except Exception:
                    mean, std = None, None

                # Send data to main thread for plotting (thread-safe)
                try:
                    self.plot_queue.put(('update_plot', (y, mean, std)), timeout=0.1)
                except:
                    pass  # Queue full, skip this frame
            
            # Maintain frame rate
            dt = time.time() - t0
            if dt < FRAME_INTERVAL:
                time.sleep(FRAME_INTERVAL - dt)
    
    def update_plot(self, data):
        """Update plot - must be called from main thread. data = (y, mean, std)"""
        try:
            y, mean, std = data
            self.line.set_ydata(y)
            self.ax.set_ylim(0, max(4096, int(y.max() * 1.1)))
            self.ax.set_title(f'Arduino Scope - Peak {int(y.max())} (Live)')
            # update peak markers from last_peaks
            if self.peak_scatter is not None and self.last_peaks:
                xs = [p[2] for p in self.last_peaks]   # pos
                ys = [p[1] for p in self.last_peaks]   # val
                offsets = np.column_stack((xs, ys))
                self.peak_scatter.set_offsets(offsets)
            elif self.peak_scatter is not None:
                self.peak_scatter.set_offsets(np.empty((0, 2)))

            # update stat lines if available
            if mean is not None:
                self.mean_line.set_ydata([mean, mean])
                if std is None:
                    std = 0.0
                self.plus_line.set_ydata([mean + std, mean + std])
                self.minus_line.set_ydata([mean - std, mean - std])
            else:
                # move lines out of view if no stats
                self.mean_line.set_ydata([0, 0])
                self.plus_line.set_ydata([0, 0])
                self.minus_line.set_ydata([0, 0])
            # also reflect any persistent init_stats (if set) so they remain visible
            if self.init_stats[0] is not None:
                m_init, s_init = self.init_stats
                self.init_mean_line.set_visible(True)
                self.init_mean_line.set_ydata([m_init, m_init])
                self.init_plus_line.set_visible(True)
                self.init_plus_line.set_ydata([m_init + (s_init or 0), m_init + (s_init or 0)])
                self.init_minus_line.set_visible(True)
                self.init_minus_line.set_ydata([m_init - (s_init or 0), m_init - (s_init or 0)])
            else:
                if self.init_mean_line:
                    self.init_mean_line.set_visible(False)
                    self.init_plus_line.set_visible(False)
                    self.init_minus_line.set_visible(False)


            self.fig.canvas.draw_idle()
        except Exception as e:
            print(f"Plot update error: {e}")



    def process_plot_queue(self):
        """Process plot updates from queue - called from main thread"""
        try:
            while True:
                command, data = self.plot_queue.get_nowait()
                if command == 'update_plot':
                    self.update_plot(data)
                elif command == 'set_init_stats':
                    # data is (mean, std) or (None, None)
                    mean, std = data
                    self.set_init_stats(mean, std)

        except Empty:
            pass  # Queue empty, nothing to process
    
    def command_thread(self):
        """Handle user commands"""
        print("\n=== Arduino Scope Controller ===")
        print("Commands:")
        print("  R         - Get single trace (with debug)")
        print("  S         - Request current stats (mean/std)")
        print("  D<num>    - Set delay (e.g., D200)")
        print("  help      - Show this help")
        print("  quit/exit - Exit program")
        print("  Any other command will be sent to Arduino")
        print("=====================================")
        print("Scope updating every 1 second (silent mode)")
        print("Type commands below:\n")
        
        while self.running:
            try:
                cmd = input("Arduino> ").strip()
                
                if cmd.lower() in ['quit', 'exit', 'q']:
                    print("Exiting...")
                    self.running = False
                    break
                    
                elif cmd.lower() == 'help':
                    print("Commands: R (trace), S (stats), D<num> (delay), quit/exit")
                    continue
                    
                elif cmd == '':
                    continue
                    
                elif cmd.upper() == 'R':
                    print("Getting single trace...")
                    y = self.get_trace(silent=False)  # Show debug for manual traces
                    if y is not None:
                        print(f"Trace acquired - Peak: {y.max()}, Mean: {y.mean():.1f}")
                        if getattr(self, 'last_peaks', []):
                            print("Peaks:")
                            for p in self.last_peaks:
                                print(f"  Peak {p[0]} val={p[1]} pos={p[2]}")
                    continue
                elif cmd.upper() == 'S':
                    print("Requesting current stats...")
                    mean, std = self.get_stats()
                    if mean is not None:
                        print(f"Current Mean: {mean:.3f}, Current Std: {std:.3f}")
                    else:
                        print("No stats response or parse failed")
                    continue

                elif cmd.upper() == 'I' or cmd.lower() == 'initialize':
                    # send init command and capture returned stats (robust parsing)
                    send_cmd = 'I' if cmd.upper() == 'I' else 'initialize'
                    print(f"Sending initialize ({send_cmd})...")
                    # initialize can take a while on the device (up to ~10s). Request a longer timeout here.
                    resp = self.send_command(send_cmd, timeout=10.0)

                    # Save raw response to disk for debugging so you can inspect what the
                    # board actually printed during the long initialize operation.
                    try:
                        if resp:
                            fname = os.path.join(os.getcwd(), 'last_init_response.txt')
                            with open(fname, 'a') as f:
                                f.write(f"--- Initialize at {time.strftime('%Y-%m-%d %H:%M:%S')} ---\n")
                                for line in resp:
                                    f.write(line + '\n')
                                f.write('\n')
                        else:
                            # record empty response event
                            fname = os.path.join(os.getcwd(), 'last_init_response.txt')
                            with open(fname, 'a') as f:
                                f.write(f"--- Initialize at {time.strftime('%Y-%m-%d %H:%M:%S')} (no lines) ---\n\n")
                    except Exception as e:
                        print(f"Could not write init response file: {e}")

                    # resp is list of ascii lines returned in short window by send_command
                    mean = None
                    std = None

                    # Try to find mean/std pairs in any of the returned lines.
                    # Accept multiple possible field names used by sketches: currentMean/currentStd,
                    # meanHeight/stdHeight, mean/std, initialPeakStats, etc.
                    joined = "\n".join(resp)

                    # First try common paired patterns
                    paired_patterns = [
                        r'currentMean\s*=\s*([+-]?[0-9]*\.?[0-9]+)\s*[,; ]+\s*currentStd\s*=\s*([+-]?[0-9]*\.?[0-9]+)',
                        r'meanHeight\s*=\s*([+-]?[0-9]*\.?[0-9]+)\s*[,; ]+\s*stdHeight\s*=\s*([+-]?[0-9]*\.?[0-9]+)',
                        r'mean\s*[:=]\s*([+-]?[0-9]*\.?[0-9]+)\s*[ ,;]+\s*std\s*[:=]\s*([+-]?[0-9]*\.?[0-9]+)'
                    ]
                    for pat in paired_patterns:
                        m = re.search(pat, joined, re.IGNORECASE)
                        if m:
                            try:
                                mean = float(m.group(1)); std = float(m.group(2))
                                break
                            except:
                                mean = None; std = None

                    # If no paired match, try to find individual fields anywhere
                    if mean is None:
                        for line in resp:
                            mm = re.search(r'(?:currentMean|meanHeight|mean|initialMean)\s*[=:]?\s*([+-]?[0-9]*\.?[0-9]+)', line, re.IGNORECASE)
                            ss = re.search(r'(?:currentStd|stdHeight|std)\s*[=:]?\s*([+-]?[0-9]*\.?[0-9]+)', line, re.IGNORECASE)
                            if mm and mean is None:
                                try: mean = float(mm.group(1))
                                except: mean = None
                            if ss and std is None:
                                try: std = float(ss.group(1))
                                except: std = None

                    # As a last resort, extract two numbers from the final lines (e.g., "... 123.4 5.6")
                    if mean is None and resp:
                        # look at last 3 response lines for two floats
                        tail = '\n'.join(resp[-3:])
                        nums = re.findall(r'([+-]?[0-9]*\.?[0-9]+)', tail)
                        if len(nums) >= 2:
                            try:
                                mean = float(nums[-2]); std = float(nums[-1])
                            except:
                                mean = None; std = None

                    if mean is not None:
                        print(f"Initialize stats parsed: mean={mean:.3f}, std={std if std is not None else 0.0:.3f}")
                        try:
                            self.plot_queue.put(('set_init_stats', (mean, std)), timeout=0.1)
                        except:
                            pass
                    else:
                        print("Initialize did not return parsable stats; raw response:")
                        for l in resp:
                            print("  ", l)
                    continue

                # Send command to Arduino
                response = self.send_command(cmd)
                if response:
                    for line in response:
                        print(f"Arduino: {line}")
                else:
                    print("No response from Arduino")
                    
            except EOFError:
                print("\nExiting...")
                self.running = False
                break
            except KeyboardInterrupt:
                print("\nExiting...")
                self.running = False
                break
    
    def run(self):
        """Main run function"""
        self.connect()
        self.setup_plot()
        self.running = True
        
        # Start threads
        scope_t = threading.Thread(target=self.scope_thread, daemon=True)
        command_t = threading.Thread(target=self.command_thread, daemon=True)
        
        scope_t.start()
        command_t.start()
        
        try:
            # Keep main thread alive and handle plot events
            while self.running:
                # Process any pending plot updates (thread-safe)
                self.process_plot_queue()
                
                # Handle matplotlib events
                plt.pause(0.05)  # Faster updates
                
                # Check if plot window was closed
                if not plt.fignum_exists(self.fig.number):
                    self.running = False
                    break
        except KeyboardInterrupt:
            print("\nShutting down...")
            self.running = False
        
        # Clean up
        scope_t.join(timeout=1)
        command_t.join(timeout=1)
        
        try:
            self.ser.close()
        except:
            pass
        plt.close('all')

def main():
    scope = ArduinoScope()
    scope.run()

if __name__ == '__main__':
    main()