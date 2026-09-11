import queue
import threading
import time

from pyvesc.protocol.packet.codec import frame, unframe

try:
    import serial
except ImportError:
    serial = None

_COMM_FORWARD_CAN = 34  # matches VESC firmware's datatypes.h COMM_PACKET_ID enum


class SerialDispatcher:
    """Single owner of a serial connection's reads, demultiplexing replies by comm-id.

    Mirrors the architectural approach VESC Tool uses: one reader continuously
    decodes complete packets and routes each to whichever consumer registered
    for that specific comm-id, rather than assuming "next complete packet in
    the buffer" answers whatever request happens to be pending. That's what
    lets an unsolicited broadcast (e.g. COMM_ROTOR_POSITION during a motor
    spin) and an unrelated request/response exchange (e.g. COMM_GET_VALUES
    telemetry polling) safely interleave on the same wire without either one
    ever consuming bytes meant for the other.
    """

    def __init__(self, serial_port):
        self._serial = serial_port
        self._write_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._buf = bytearray()
        self._waiters = {}       # comm_id -> list[queue.Queue], one per outstanding request() call
        self._subscribers = {}   # comm_id -> list[callable]
        self._stop = threading.Event()
        self._can_forward_target = None  # CAN id, or None to send straight to whatever's on the wire
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def set_can_forward_target(self, can_id) -> None:
        """Transparently wrap every future send()/request() in COMM_FORWARD_CAN,
        for talking to a VESC reachable only over CAN through a bridge (e.g. a
        VESC Express gateway) instead of directly over this serial connection.

        Pass None to disable wrapping and go back to sending directly — the
        default, and the only behavior when talking to a VESC connected
        straight to this port; existing direct-USB callers are completely
        unaffected since they never touch this setting.

        The reply side needs no matching unwrap: the bridge relays the
        target's raw, already-unwrapped reply back over this same
        connection, so it decodes exactly like a direct reply would.
        """
        self._can_forward_target = can_id

    def _reader_loop(self):
        while not self._stop.is_set():
            try:
                waiting = self._serial.in_waiting
                chunk = self._serial.read(waiting) if waiting else b''
            except (OSError, ValueError, TypeError):
                # Device gone (unplugged) or port closed under us — pyserial
                # surfaces this differently depending on timing: OSError for
                # a live disconnect, ValueError once the port object knows
                # it's closed, or TypeError if we race a close() and in_waiting
                # reads a file descriptor that just became None mid-call.
                # Either way, nothing to recover here; owning app's own
                # polling detects the loss.
                break

            if chunk:
                self._buf.extend(chunk)
            else:
                time.sleep(0.005)
                continue

            while True:
                payload, consumed = unframe(bytes(self._buf))
                if consumed == 0:
                    break
                del self._buf[:consumed]
                if payload:
                    self._dispatch(payload)

    def _dispatch(self, payload):
        comm_id = payload[0]
        with self._state_lock:
            waiters = list(self._waiters.get(comm_id, ()))
            subs = list(self._subscribers.get(comm_id, ()))
        for q in waiters:
            try:
                q.put_nowait(payload)
            except queue.Full:
                pass
        for callback in subs:
            callback(payload)

    def send(self, data: bytes) -> None:
        """Write a fire-and-forget packet (no reply expected)."""
        if self._can_forward_target is not None:
            data = self._wrap_for_can_forward(data)
        with self._write_lock:
            self._serial.write(data)

    def _wrap_for_can_forward(self, data: bytes) -> bytes:
        payload, consumed = unframe(data)
        if not payload or consumed != len(data):
            # Not a single, complete, well-formed frame -- send as-is rather
            # than risk mangling something this wasn't meant to handle; every
            # real caller here always passes exactly one frame() result.
            return data
        return frame(bytes([_COMM_FORWARD_CAN, self._can_forward_target]) + payload)

    def request(self, data: bytes, comm_id: int, timeout: float) -> bytes:
        """Send a packet and block for the next payload matching comm_id.

        Raises TimeoutError if no matching payload arrives within timeout.
        Multiple request() calls for the same comm_id may be outstanding at
        once (e.g. telemetry polling GET_VALUES while a test also polls it
        directly) — each registers its own queue, and every incoming payload
        for that comm_id is broadcast to all of them. That's safe because
        VESC replies aren't correlated to a specific request via any
        sequence ID: a GET_VALUES reply is just "current sensor state",
        interchangeable regardless of which concurrent request triggered it.
        """
        q = queue.Queue(maxsize=1)
        with self._state_lock:
            self._waiters.setdefault(comm_id, []).append(q)
        try:
            self.send(data)
            try:
                return q.get(timeout=timeout)
            except queue.Empty:
                raise TimeoutError(f"No response for comm_id {comm_id} after {timeout:.1f}s")
        finally:
            with self._state_lock:
                waiters = self._waiters.get(comm_id)
                if waiters is not None and q in waiters:
                    waiters.remove(q)
                    if not waiters:
                        del self._waiters[comm_id]

    def subscribe(self, comm_id: int, callback):
        """Register callback(payload) for every future payload matching comm_id.

        Returns an unsubscribe() function.
        """
        with self._state_lock:
            self._subscribers.setdefault(comm_id, []).append(callback)

        def unsubscribe():
            with self._state_lock:
                subs = self._subscribers.get(comm_id)
                if subs and callback in subs:
                    subs.remove(callback)

        return unsubscribe

    def stop(self) -> None:
        self._stop.set()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
