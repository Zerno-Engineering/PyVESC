import queue
import threading
import time

from pyvesc.protocol.packet.codec import unframe

try:
    import serial
except ImportError:
    serial = None


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
        self._waiters = {}       # comm_id -> queue.Queue (at most one outstanding request per comm_id)
        self._subscribers = {}   # comm_id -> list[callable]
        self._stop = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

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
            waiter = self._waiters.get(comm_id)
            subs = list(self._subscribers.get(comm_id, ()))
        if waiter is not None:
            try:
                waiter.put_nowait(payload)
            except queue.Full:
                pass
        for callback in subs:
            callback(payload)

    def send(self, data: bytes) -> None:
        """Write a fire-and-forget packet (no reply expected)."""
        with self._write_lock:
            self._serial.write(data)

    def request(self, data: bytes, comm_id: int, timeout: float) -> bytes:
        """Send a packet and block for the next payload matching comm_id.

        Raises TimeoutError if no matching payload arrives within timeout.
        Only one request per comm_id may be outstanding at a time — this
        holds for every caller in this app (telemetry is the sole periodic
        COMM_GET_VALUES requester; tests run sequentially).
        """
        q = queue.Queue(maxsize=1)
        with self._state_lock:
            if comm_id in self._waiters:
                raise RuntimeError(
                    f"A request for comm_id {comm_id} is already outstanding"
                )
            self._waiters[comm_id] = q
        try:
            self.send(data)
            try:
                return q.get(timeout=timeout)
            except queue.Empty:
                raise TimeoutError(f"No response for comm_id {comm_id} after {timeout:.1f}s")
        finally:
            with self._state_lock:
                if self._waiters.get(comm_id) is q:
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
