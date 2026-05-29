import socket
import json
import time
import re
import threading
import os
from datetime import datetime
from pathlib import Path
import pandas as pd


'''
*************************** WIFI_Pipelined_Receiver_w_Error_Detection.py ***************************

Filename:       WIFI_Pipelined_Receiver_w_Error_Detection.py
Author:         Jestin

Description:    This file implements the PC-side server for the bimanual sensorised glove
                system. It opens two TCP servers (one per glove), pipelines REQUEST_DATA
                commands to both gloves in parallel, flattens the returned nested JSON
                packets into a tabular row format, performs a dead-channel sanity check
                early in the run to abort on hardware failure, and finally writes the
                paired left/right glove samples to a timestamped CSV for downstream
                machine learning use.

Dependencies:   socket  json    time    re      threading   os      datetime
                pathlib pandas

*****************************************************************************************************
'''


# Network and timing config
HOST = "0.0.0.0"
LEFT_PORT = 5000
RIGHT_PORT = 5001

RUN_SECONDS = 3
REQUEST_INTERVAL = 0.005   # 5 ms between requests, reduce if the gloves can keep up
START_DELAY = 2.0          # short pause after Enter to allow time to get into position


# Output directory for the CSV, change this for each gesture or subject
OUTPUT_DIR = r"/home/jestin/ThesisRepo/ML/NewTestData/8_Jestin/NewNewDynamic/Double_Snap"

# Build a file prefix from the last two folder names so the filename
# identifies the subject and gesture without needing to open the file
parts = OUTPUT_DIR.split("/")
FILE_PREFIX = f"glove_data_L_{parts[-2]}_{parts[-1]}_{RUN_SECONDS}s"


# Dead-channel detection settings
# After ZERO_CHECK_AFTER_N requests the script inspects collected data and
# checks if any signal column has been zero the whole time. A dead column
# usually means a sensor is not responding (loose I2C line or dead IMU),
# in which case the script aborts rather than continue recording garbage.
ZERO_CHECK_AFTER_N = 20

# Columns to skip when checking for dead channels. These are either metadata
# (timestamps, ids) or sensors that are known to be absent on the current
# glove (palm_prox has no IMU populated).
EXCLUDE_PREFIXES = (
    "run_index", "request_id", "request_ts",
    "left_recv_time_ms", "left_glove_time_ms", "left_time",
    "right_recv_time_ms", "right_glove_time_ms", "right_time",
    "left_hand", "right_hand",
    "left_palm_prox",
    "right_palm_prox",
)


def get_next_run_index():
    '''
    Scans the output directory for existing CSVs with the current file prefix
    and returns the next available run index so that repeated runs do not
    overwrite previous data.

    Input:
        No direct inputs, reads OUTPUT_DIR and FILE_PREFIX from the module scope

    Output:
        next_index (int):   The next run index to use, equal to one greater than
                            the highest index found in existing CSV filenames, or
                            1 if no matching files are present
    '''
    pattern = re.compile(
        rf"^{FILE_PREFIX}_(\d+)_\d{{4}}-\d{{2}}-\d{{2}}_\d{{2}}-\d{{2}}-\d{{2}}\.csv$"
    )
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    max_index = 0
    for path in Path(OUTPUT_DIR).glob(f"{FILE_PREFIX}_*.csv"):
        m = pattern.match(path.name)
        if m:
            idx = int(m.group(1))
            if idx > max_index:
                max_index = idx
    return max_index + 1


RUN_INDEX = get_next_run_index()
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
OUTPUT_CSV = os.path.join(OUTPUT_DIR, f"{FILE_PREFIX}_{RUN_INDEX}_{timestamp}.csv")


def flatten_glove_json(msg, base_time_ms, hand_label):
    '''
    Converts a nested glove JSON packet into a single flat row of column/value
    pairs suitable for being collected into a pandas DataFrame.

    Each glove transmits a packet of the form:
        {"Time": ..., "Hand": "L", "Data": {"thumb": {"flex_mcp": ..., "quat_w_prox": ...}, ...}}
    which is flattened to column names of the form:
        left_thumb_mcp_flex, left_thumb_prox_quat_w, left_wrist_quat_w, etc.

    Input:
        msg (dict):                 The decoded JSON packet from the glove, expected
                                    to contain at minimum "Time", "Hand", and "Data" keys

        base_time_ms (int or None): The Time value of the first packet received from
                                    this glove, used as t = 0 for relative timestamps

        hand_label (string):        Either "left" or "right", prepended to every column
                                    name to distinguish data from the two gloves

    Output:
        row (dict): A single flat dictionary mapping column name to value, containing
                    request metadata, timing fields, and all sensor readings from msg
    '''
    current_time_ms = msg.get("Time")

    row = {
        "run_index": RUN_INDEX,
        "request_id": msg.get("request_id"),
        "request_ts": msg.get("request_ts"),
        f"{hand_label}_hand": msg.get("Hand"),
        f"{hand_label}_recv_time_ms": msg.get("_recv_time_ms"),
        f"{hand_label}_glove_time_ms": msg.get("glove_time_ms"),
    }

    # Relative time since the first packet from this glove, in milliseconds
    if current_time_ms is not None and base_time_ms is not None:
        row[f"{hand_label}_time"] = current_time_ms - base_time_ms
    else:
        row[f"{hand_label}_time"] = None

    data = msg.get("Data", {})
    for sensor_name, sensor_values in data.items():
        sensor = sensor_name.lower()

        for key, value in sensor_values.items():
            # Flex sensor reading, e.g. "flex_mcp" becomes left_thumb_mcp_flex
            if key.startswith("flex_"):
                joint = key[len("flex_"):]
                col = f"{hand_label}_{sensor}_{joint}_flex"

            # Quaternion component, e.g. "quat_w_prox" becomes left_thumb_prox_quat_w
            # Wrist quaternions only have two parts (no segment) and are handled separately
            elif key.startswith("quat_"):
                bits = key.split("_")
                if len(bits) == 3:
                    _, component, segment = bits
                    col = f"{hand_label}_{sensor}_{segment}_quat_{component}"
                elif sensor == "wrist" and len(bits) == 2:
                    _, component = bits
                    col = f"{hand_label}_{sensor}_quat_{component}"
                else:
                    col = f"{hand_label}_{sensor}_{key}"

            # Everything else, e.g. ax_prox, yaw_mid
            else:
                bits = key.split("_")
                if len(bits) == 2:
                    metric, segment = bits
                    col = f"{hand_label}_{sensor}_{segment}_{metric}"
                else:
                    col = f"{hand_label}_{sensor}_{key}"

            row[col] = value

    return row


def check_zero_channels(combined_rows):
    '''
    Inspects all rows collected so far and aborts the program if any individual
    signal column has been zero across every row. A dead column almost always
    indicates a sensor that is not responding (loose wire, dead IMU, or
    multiplexer issue), so aborting here avoids recording several seconds of
    unusable data.

    Input:
        combined_rows (dict):   The full request_id to row dictionary collected
                                from both gloves up to this point in the run

    Output:
        No return value. The function prints a list of dead columns and raises
        SystemExit(1) if any are found, otherwise it returns silently.
    '''
    if not combined_rows:
        return

    rows = list(combined_rows.values())

    # Collect every column name seen so far across all rows
    all_cols = set()
    for row in rows:
        all_cols.update(row.keys())

    # Drop metadata, time columns, and known-empty sensors
    signal_cols = []
    for c in all_cols:
        if not any(c.startswith(p) for p in EXCLUDE_PREFIXES):
            signal_cols.append(c)

    dead_cols = []
    for col in sorted(signal_cols):
        all_zero = True
        for row in rows:
            val = row.get(col)
            try:
                if float(val) != 0.0:
                    all_zero = False
                    break
            except (TypeError, ValueError):
                # None or non-numeric value, ignore for this row
                pass
        if all_zero:
            dead_cols.append(col)

    if dead_cols:
        print("\n[ERROR] Dead (all-zero) columns detected. Aborting:")
        for col in dead_cols:
            print(f"  x  {col}  all zeros across {len(rows)} sampled rows")
        print(
            "\nLikely cause: IMU or sensor not responding, or I2C failure on that finger."
            "\nFix the hardware and re-run the script.\n"
        )
        raise SystemExit(1)


def reorder_columns(df):
    '''
    Reorders the columns of the combined DataFrame into a consistent, analysis
    friendly layout. The final order is:
        [run_index, request_id] -> time columns -> hand label columns -> signal columns
    where signal columns are themselves sorted by hand, then sensor, then segment,
    then metric.

    Input:
        df (pandas.DataFrame):  The DataFrame produced from the collected paired
                                rows, with columns in arbitrary insertion order

    Output:
        df (pandas.DataFrame):  The same DataFrame with its columns rearranged
                                into the canonical order described above
    '''
    base_cols = ["run_index", "request_id"]

    time_keywords = ("request_ts", "recv_time", "glove_time", "_time")
    time_cols = [
        c for c in df.columns
        if c not in base_cols and any(k in c for k in time_keywords)
    ]

    hand_cols = [c for c in ("left_hand", "right_hand") if c in df.columns]

    # Sort priority dictionaries, anything not listed is pushed to the end (99)
    hand_order = {"left": 0, "right": 1}
    sensor_order = {
        "palm": 0, "thumb": 1, "index": 2, "middle": 3,
        "ring": 4, "pinky": 5, "wrist": 6,
    }
    segment_order = {"mid": 0, "prox": 1, "mcp": 2, "pip": 3}
    metric_order = {
        "yaw": 0, "pitch": 1, "roll": 2,
        "quat_w": 3, "quat_x": 4, "quat_y": 5, "quat_z": 6,
        "ax": 7, "ay": 8, "az": 9,
        "flex": 10,
    }
    wrist_metric_order = {
        "heading": 0, "pitch": 1, "roll": 2,
        "quat_w": 3, "quat_x": 4, "quat_y": 5, "quat_z": 6,
        "ax": 7, "ay": 8, "az": 9,
    }

    def sort_key(col):
        bits = col.split("_")
        hand = bits[0] if len(bits) > 0 else ""
        sensor = bits[1] if len(bits) > 1 else ""

        # Wrist columns have no segment, format is hand_wrist_metric
        if sensor == "wrist":
            metric = "_".join(bits[2:]) if len(bits) > 2 else ""
            return (
                hand_order.get(hand, 99),
                sensor_order.get(sensor, 99),
                99,
                wrist_metric_order.get(metric, 99),
                col,
            )

        # Flex columns: hand_sensor_segment_flex
        if len(bits) == 4 and bits[3] == "flex":
            segment = bits[2]
            return (
                hand_order.get(hand, 99),
                sensor_order.get(sensor, 99),
                segment_order.get(segment, 99),
                metric_order["flex"],
                col,
            )

        # Quaternion columns: hand_sensor_segment_quat_component
        if len(bits) == 5 and bits[3] == "quat":
            segment = bits[2]
            metric = f"quat_{bits[4]}"
            return (
                hand_order.get(hand, 99),
                sensor_order.get(sensor, 99),
                segment_order.get(segment, 99),
                metric_order.get(metric, 99),
                col,
            )

        # Standard columns: hand_sensor_segment_metric, e.g. left_thumb_prox_ax
        if len(bits) == 4:
            segment = bits[2]
            metric = bits[3]
            return (
                hand_order.get(hand, 99),
                sensor_order.get(sensor, 99),
                segment_order.get(segment, 99),
                metric_order.get(metric, 99),
                col,
            )

        # Anything unexpected is placed at the end
        return (99, 99, 99, 99, col)

    signal_cols = [
        c for c in df.columns
        if c not in base_cols and c not in time_cols and c not in hand_cols
    ]
    signal_cols.sort(key=sort_key)

    return df[base_cols + time_cols + hand_cols + signal_cols]


class GloveConnection:
    '''
    Class that encapsulates one TCP connection to a single glove (left or right).
    The class is responsible for opening a listening socket, accepting the glove
    connection, sending REQUEST_DATA commands, draining incoming bytes from the
    socket into complete JSON packets, and merging each packet into the shared
    combined_rows dictionary keyed by request_id.

    Each connection maintains its own receive buffer because TCP does not preserve
    message boundaries, so incoming data has to be split on the newline delimiter
    manually.

    Class Variables:
        label
        port
        server
        conn
        addr
        buffer
        base_time_ms
        connected

    Methods:
        setup_server(self)
        accept(self)
        send_json(self, obj)
        drain(self, combined_rows)
        close(self)
    '''

    def __init__(self, label, port):
        self.label = label
        self.port = port
        self.server = None
        self.conn = None
        self.addr = None
        self.buffer = ""
        self.base_time_ms = None
        self.connected = False

    def setup_server(self):
        '''
        Creates the listening TCP socket on (HOST, self.port) with SO_REUSEADDR
        enabled so the port can be re-bound quickly between runs without
        getting stuck in TIME_WAIT.

        Input:
            No direct inputs, uses HOST from module scope and self.port

        Output:
            No return value, assigns the bound socket to self.server
        '''
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((HOST, self.port))
        self.server.listen(1)
        self.server.settimeout(0.5)

    def accept(self):
        '''
        Blocks until the glove connects to the listening socket. The accept call
        uses a short timeout so the loop can be cleanly interrupted with Ctrl+C
        if a glove fails to connect.

        Input:
            No direct inputs, operates on self.server

        Output:
            No return value, assigns the accepted client socket to self.conn,
            its address to self.addr, and sets self.connected to True
        '''
        while not self.connected:
            try:
                conn, addr = self.server.accept()
                conn.settimeout(0.1)
                self.conn = conn
                self.addr = addr
                self.connected = True
                print(f"[{self.label}] Connected by {addr}")
            except socket.timeout:
                continue

    def send_json(self, obj):
        '''
        Serialises a Python dictionary as a single newline-terminated JSON line
        and sends it to the glove over the open TCP connection.

        Input:
            obj (dict): The command to send to the glove, typically a REQUEST_DATA
                        message containing type, request_id, and request_ts

        Output:
            No return value. If the send fails because the connection has been
            broken, self.connected is set to False and a message is printed.
        '''
        if not self.conn:
            return
        payload = json.dumps(obj) + "\n"
        try:
            self.conn.sendall(payload.encode("utf-8"))
        except (BrokenPipeError, ConnectionResetError):
            print(f"[{self.label}] Send failed, connection lost")
            self.connected = False

    def drain(self, combined_rows):
        '''
        Reads any bytes currently waiting on the socket, splits them into
        complete JSON packets on the newline delimiter, and merges each packet
        into combined_rows keyed by request_id. Partial lines are retained in
        self.buffer for the next call. Control messages from the glove (any
        packet carrying a "type" field) are skipped, only data packets are
        recorded.

        Input:
            combined_rows (dict):   The shared request_id to row dictionary into
                                    which flattened packets from this glove are
                                    merged

        Output:
            No return value, combined_rows is updated in place
        '''
        if not self.conn:
            return

        try:
            data = self.conn.recv(4096)
            if not data:
                print(f"[{self.label}] Disconnected during drain")
                self.connected = False
                return
            self.buffer += data.decode("utf-8")
        except socket.timeout:
            pass
        except ConnectionResetError:
            self.connected = False
            return

        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue

            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                print(f"[{self.label}] Invalid JSON skipped")
                continue

            # Skip control or acknowledgement messages, only data packets have no "type"
            if obj.get("type") is not None:
                continue

            # Tag with PC-side receive time for later latency analysis
            obj["_recv_time_ms"] = time.time() * 1000

            # First data packet defines t = 0 for this glove's relative timestamps
            if self.base_time_ms is None:
                self.base_time_ms = obj.get("Time")

            req_id = obj.get("request_id")
            if req_id not in combined_rows:
                combined_rows[req_id] = {}

            flat = flatten_glove_json(obj, self.base_time_ms, self.label)
            combined_rows[req_id].update(flat)
            print(f"[{self.label}] reply for request_id={req_id}")

    def close(self):
        '''
        Closes both the client and listening sockets, ignoring any errors that
        occur during shutdown.

        Input:
            No direct inputs

        Output:
            No return value
        '''
        for s in (self.conn, self.server):
            if s is not None:
                try:
                    s.close()
                except Exception:
                    pass


def accept_worker(glove):
    '''
    Threaded helper that sets up the listening socket for a single glove and
    blocks until that glove connects. Running one of these per glove in
    parallel allows the left and right gloves to connect in any order.

    Input:
        glove (GloveConnection):    The glove connection object to bring online

    Output:
        No return value, the glove is left in a connected state on success
    '''
    glove.setup_server()
    print(f"[{glove.label}] Listening on {HOST}:{glove.port}")
    glove.accept()


def main():
    '''
    Top level routine that brings up both glove connections, runs the pipelined
    request loop for RUN_SECONDS, performs the dead-channel sanity check, drains
    any trailing in-flight replies, filters down to fully paired rows, reorders
    the columns, and writes the resulting DataFrame to OUTPUT_CSV.

    Input:
        No direct inputs, all configuration is read from module level constants

    Output:
        No return value, the side effect is a CSV file written to OUTPUT_CSV
        and progress messages printed to the terminal
    '''
    left = GloveConnection("left", LEFT_PORT)
    right = GloveConnection("right", RIGHT_PORT)
    combined_rows = {}

    # Start both servers and wait for both gloves to connect in parallel
    t_left = threading.Thread(target=accept_worker, args=(left,), daemon=True)
    t_right = threading.Thread(target=accept_worker, args=(right,), daemon=True)
    t_left.start()
    t_right.start()
    t_left.join()
    t_right.join()

    print("Both gloves connected.")
    input("Press Enter to begin requesting data...")

    print(f"Waiting {START_DELAY} second(s) before starting requests...")
    time.sleep(START_DELAY)

    start = time.time()
    request_id = 0

    # Main request/response loop. The pipelined approach sends a REQUEST_DATA to
    # both gloves and then immediately drains whatever has already arrived from
    # earlier requests, without blocking on the new reply. This keeps both
    # gloves busy in parallel and maximises sample rate.
    while time.time() - start < RUN_SECONDS:
        request_id += 1
        request_ts = datetime.now().isoformat(timespec="milliseconds")

        cmd = {
            "type": "REQUEST_DATA",
            "request_id": request_id,
            "request_ts": request_ts,
        }

        left.send_json(cmd)
        right.send_json(cmd)
        print(f"[SERVER] Sent REQUEST_DATA {request_id}")

        left.drain(combined_rows)
        right.drain(combined_rows)

        # One-shot dead-channel check early in the run
        if request_id == ZERO_CHECK_AFTER_N:
            check_zero_channels(combined_rows)

        time.sleep(REQUEST_INTERVAL)

    # After the main loop ends, allow another 2 seconds for the gloves to flush
    # any replies still in flight. Without this trailing drain the last few
    # packets are typically lost.
    drain_deadline = time.time() + 2.0
    while time.time() < drain_deadline:
        left.drain(combined_rows)
        right.drain(combined_rows)
        time.sleep(0.005)

    # Keep only paired rows where both gloves replied, incomplete rows
    # (one hand missing) are not useful for bimanual gesture analysis
    complete = {}
    for rid, row in combined_rows.items():
        if "left_hand" in row and "right_hand" in row:
            complete[rid] = row

    incomplete = len(combined_rows) - len(complete)
    print(f"Total requests: {request_id} | Complete pairs: {len(complete)} | Incomplete: {incomplete}")

    if complete:
        ordered = [complete[k] for k in sorted(complete.keys())]
        df = pd.DataFrame(ordered)
        df = reorder_columns(df)
        df.to_csv(OUTPUT_CSV, index=False)
        print(f"Saved {len(df)} paired row(s) to {OUTPUT_CSV}")
    else:
        print("No complete paired rows. No CSV created.")

    left.close()
    right.close()


if __name__ == "__main__":
    main()
