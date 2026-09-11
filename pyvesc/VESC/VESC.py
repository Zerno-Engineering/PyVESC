from pyvesc.protocol.base import VESCMessage
from pyvesc.protocol.interface import encode_request, encode
from pyvesc.protocol.packet.codec import frame
from pyvesc.VESC.messages import *
from pyvesc.VESC.serial_dispatcher import SerialDispatcher
import math
import queue
import struct
import threading
import time

# because people may want to use this library for their own messaging, do not make this a required package
try:
    import serial
except ImportError:
    serial = None


def _decode_float32_auto(raw_u32):
    """Port of buffer_get_float32_auto() from the VESC firmware (util/buffer.c).

    Not standard IEEE-754 — a custom sign/exponent/23-bit-mantissa packing
    used by confgenerator.c for most mc_configuration float fields.
    """
    e = (raw_u32 >> 23) & 0xFF
    sig_i = raw_u32 & 0x7FFFFF
    neg = bool(raw_u32 & (1 << 31))
    sig = 0.0
    if e != 0 or sig_i != 0:
        sig = sig_i / (8388608.0 * 2.0) + 0.5
        e -= 126
    if neg:
        sig = -sig
    return math.ldexp(sig, e)


class VESC(object):
    def __init__(self, serial_port, has_sensor=False, start_heartbeat=True, baudrate=115200, timeout=0.05):
        """
        :param serial_port: Serial device to use for communication (i.e. "COM3" or "/dev/tty.usbmodem0")
        :param has_sensor: Whether or not the bldc motor is using a hall effect sensor
        :param start_heartbeat: Whether or not to automatically start the heartbeat thread that will keep commands
                                alive.
        :param baudrate: baudrate for the serial communication. Shouldn't need to change this.
        :param timeout: timeout for the serial communication
        """

        if serial is None:
            raise ImportError("Need to install pyserial in order to use the VESCMotor class.")

        self.serial_port = serial.Serial(port=serial_port, baudrate=baudrate, timeout=timeout)

        # Single owner of all reads, demultiplexing replies by comm-id so
        # unrelated traffic (heartbeat, telemetry polling, test commands,
        # unsolicited rotor-position broadcasts) can safely interleave on the
        # same connection — see serial_dispatcher.py for why this replaces
        # each method doing its own blocking read.
        self._dispatcher = SerialDispatcher(self.serial_port)

        if has_sensor:
            self._dispatcher.send(encode(SetRotorPositionMode(SetRotorPositionMode.DISP_POS_OFF)))

        self.alive_msg = [encode(Alive())]

        self.heart_beat_thread = threading.Thread(target=self._heartbeat_cmd_func)
        self._stop_heartbeat = threading.Event()

        if start_heartbeat:
            self.start_heartbeat()

        # check firmware version and set GetValue fields to old values if pre version 3.xx
        version = self.get_firmware_version()
        if int(version.split('.')[0]) < 3:
            GetValues.fields = pre_v3_33_fields

        # store message info for getting values so it doesn't need to calculate it every time
        self._get_values_msg = encode_request(GetValues())

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        """Stop the heartbeat and the dispatcher's reader thread, then close
        the serial port — in that order, so the reader thread is never still
        touching the port when it closes (pyserial raises different, racy
        exceptions from in_waiting()/read() depending on exactly when the
        underlying file descriptor disappears mid-call otherwise).

        flush()/close() are each best-effort: if the device physically
        disappeared before we get here (e.g. the USB cable was pulled while
        connected, rather than a clean jump-to-bootloader reboot we
        triggered ourselves), they can raise platform-specific errors that
        aren't even OSError subclasses — flush()'s termios.tcdrain() raises
        plain termios.error on POSIX. None of that is actionable since this
        connection is being discarded either way.
        """
        self.stop_heartbeat()
        self._dispatcher.stop()
        if self.serial_port.is_open:
            try:
                self.serial_port.flush()
            except Exception:
                pass
            try:
                self.serial_port.close()
            except Exception:
                pass

    def _heartbeat_cmd_func(self):
        """
        Continuous function calling that keeps the motor alive
        """
        while not self._stop_heartbeat.is_set():
            time.sleep(0.1)
            try:
                for i in self.alive_msg:
                    self._dispatcher.send(i)
            except (OSError, serial.SerialException):
                # Device disappeared (e.g. USB unplugged). Nothing to retry —
                # a fresh VESC instance gets its own heartbeat on reconnect,
                # and the owning app's own polling detects the loss.
                break

    def start_heartbeat(self, can_id=None):
        """
        Starts a repetitive calling of the last set cmd to keep the motor alive.

        Args:
            can_id: Optional, used to specify the CAN ID to add to the existing heartbeat messaged
        """
        if can_id is not None:
            self.alive_msg.append(encode(Alive(can_id=can_id)))
        else:
            self.heart_beat_thread.start()

    def stop_heartbeat(self):
        """
        Stops the heartbeat thread and resets the last cmd function. THIS MUST BE CALLED BEFORE THE OBJECT GOES OUT OF
        SCOPE UNLESS WRAPPING IN A WITH STATEMENT (Assuming the heartbeat was started).
        """
        self._stop_heartbeat.set()
        if self.heart_beat_thread.is_alive():
            self.heart_beat_thread.join()

    def set_rpm(self, new_rpm, **kwargs):
        """
        Set the electronic RPM value (a.k.a. the RPM value of the stator)
        :param new_rpm: new rpm value
        """
        self._dispatcher.send(encode(SetRPM(new_rpm, **kwargs)))

    def set_current(self, new_current, **kwargs):
        """
        :param new_current: new current in milli-amps for the motor
        """
        self._dispatcher.send(encode(SetCurrent(new_current, **kwargs)))

    def set_duty_cycle(self, new_duty_cycle, **kwargs):
        """
        :param new_duty_cycle: Value of duty cycle to be set (range [-1e5, 1e5]).
        """
        self._dispatcher.send(encode(SetDutyCycle(new_duty_cycle, **kwargs)))

    def set_servo(self, new_servo_pos, **kwargs):
        """
        :param new_servo_pos: New servo position. valid range [0, 1]
        """
        self._dispatcher.send(encode(SetServoPosition(new_servo_pos, **kwargs)))

    def get_measurements(self, timeout=1.0):
        """
        :return: A msg object with attributes containing the measurement values
        """
        payload = self._dispatcher.request(self._get_values_msg, GetValues.id, timeout=timeout)
        return VESCMessage.unpack(payload)

    def get_firmware_version(self, timeout=2.0):
        payload = self._dispatcher.request(encode_request(GetVersion()), GetVersion.id, timeout=timeout)
        return str(VESCMessage.unpack(payload))

    def send_terminal_cmd(self, cmd, timeout=1.0):
        """Send a terminal command string and return all COMM_PRINT responses concatenated.

        :param cmd: Command string (e.g. "faults")
        :param timeout: Maximum seconds to wait for the full response.
        :return: Response text string, or None if no response received.
        """
        messages = []
        state_lock = threading.Lock()
        last_received_at = [time.monotonic()]

        def on_print(payload):
            try:
                msg = VESCMessage.unpack(payload)
            except Exception:
                return
            with state_lock:
                last_received_at[0] = time.monotonic()
                if hasattr(msg, 'message'):
                    messages.append(msg.message)

        unsubscribe = self._dispatcher.subscribe(Print.id, on_print)
        try:
            self._dispatcher.send(encode(TerminalCmd(cmd)))
            deadline = time.monotonic() + timeout
            # Accumulate until ~100ms of silence follows at least one message,
            # since terminal output may arrive as several COMM_PRINT packets.
            while time.monotonic() < deadline:
                time.sleep(0.02)
                with state_lock:
                    have_messages = bool(messages)
                    quiet_for = time.monotonic() - last_received_at[0]
                if have_messages and quiet_for >= 0.1:
                    break
        finally:
            unsubscribe()

        with state_lock:
            return ''.join(messages) if messages else None

    def get_fw_info(self, timeout=2.0):
        """Request COMM_FW_INFO and return (fw_major, fw_minor, fw_test, git_hash, user_git_hash).

        Parses the response manually because it contains two null-terminated
        strings which VESCMessage fields do not support simultaneously.
        Returns None if the response cannot be parsed or times out.
        """
        try:
            payload = self._dispatcher.request(encode_request(GetFwInfo), GetFwInfo.id, timeout=timeout)
        except TimeoutError:
            return None
        if len(payload) < 5:
            return None

        # payload[0] = COMM_FW_INFO (cmd id)
        # payload[1..3] = fw_major, fw_minor, fw_test_version
        # payload[4..] = GIT_COMMIT_HASH\0 USER_GIT_COMMIT_HASH\0
        fw_major = payload[1]
        fw_minor  = payload[2]
        fw_test   = payload[3]
        rest = bytes(payload[4:])

        null1 = rest.find(b'\x00')
        git_hash = rest[:null1].decode('ascii', errors='replace') if null1 >= 0 else rest.decode('ascii', errors='replace')

        rest2 = rest[null1 + 1:] if null1 >= 0 else b''
        null2 = rest2.find(b'\x00')
        user_git_hash = rest2[:null2].decode('ascii', errors='replace') if null2 >= 0 else rest2.decode('ascii', errors='replace')

        return fw_major, fw_minor, fw_test, git_hash, user_git_hash

    def get_mcu_uuid(self, timeout=2.0):
        """Extract the 12-byte STM32 UUID from a COMM_FW_VERSION response.

        COMM_FW_VERSION payload layout:
            [0]       cmd_id (COMM_FW_VERSION)
            [1]       fw_major
            [2]       fw_minor
            [3..N]    hw_name (null-terminated)
            [N+1..N+12] STM32 UUID (12 bytes)

        Returns a 24-character uppercase hex string, or None if the response
        cannot be parsed, times out, or UUID bytes are missing/all-zero.
        """
        try:
            payload = self._dispatcher.request(encode_request(GetVersion()), GetVersion.id, timeout=timeout)
        except TimeoutError:
            return None
        if len(payload) < 4:
            return None

        hw_name_start = 3
        rest = bytes(payload[hw_name_start:])
        null_pos = rest.find(b'\x00')
        if null_pos < 0:
            return None

        uuid_start = hw_name_start + null_pos + 1
        if len(payload) < uuid_start + 12:
            return None

        uuid_bytes = bytes(payload[uuid_start:uuid_start + 12])
        if uuid_bytes == b'\x00' * 12:
            return None

        return uuid_bytes.hex().upper()

    def ping_can(self, timeout=5.0):
        """Send COMM_PING_CAN and return the list of CAN ids that responded.

        Used to discover a VESC reachable only over CAN through a bridge
        (e.g. a VESC Express gateway) rather than directly on this
        connection — the bridge itself sweeps all 255 possible ids on its
        CAN bus and reports back which ones are actually present. That
        sweep consistently takes ~2.5s on real hardware regardless of this
        connection's own read timeout, so the default here is independent
        of it and generous enough to leave real margin.
        """
        _CMD = 62  # COMM_PING_CAN
        payload = self._dispatcher.request(frame(bytes([_CMD])), _CMD, timeout=timeout)
        return list(payload[1:])

    def detect_motor_rl(self, timeout=30.0):
        """Send COMM_DETECT_MOTOR_R_L and return (r_ohm, l_henry, ld_lq_diff_henry).

        Blocks in the VESC firmware for several seconds while it applies current
        to measure resistance and inductance. Returns (0.0, 0.0, 0.0) when the
        VESC reports a fault (e.g. no motor connected).
        """
        _CMD = 25  # COMM_DETECT_MOTOR_R_L
        payload = self._dispatcher.request(frame(bytes([_CMD])), _CMD, timeout=timeout)
        if len(payload) < 13:
            raise TimeoutError(f"Malformed response to COMM_DETECT_MOTOR_R_L")
        r, l, ld_lq_diff = struct.unpack_from('!iii', payload, 1)
        return r / 1e6, l / 1e3, ld_lq_diff / 1e3

    def detect_motor_flux_linkage_openloop(self, current, erpm_per_sec, duty,
                                           resistance, inductance, timeout=60.0):
        """Send COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP and return
        (linkage_wb, enc_offset, enc_ratio, enc_inverted).

        Spins the motor open-loop to measure flux linkage. Returns (0.0, ...)
        when the VESC reports a fault.

        Request field order (from commands.c):
            current [A]       × 1e3
            erpm_per_sec      × 1e3
            duty [0.0–1.0]    × 1e3
            resistance [Ω]    × 1e6
            inductance [H]    × 1e8  (optional field — always sent)
        """
        _CMD = 57  # COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP
        params = struct.pack('!iiiii',
            int(current      * 1e3),
            int(erpm_per_sec * 1e3),
            int(duty         * 1e3),
            int(resistance   * 1e6),
            int(inductance   * 1e8),
        )
        payload = self._dispatcher.request(frame(bytes([_CMD]) + params), _CMD, timeout=timeout)
        if len(payload) < 14:
            raise TimeoutError(f"Malformed response to COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP")
        linkage, enc_offset, enc_ratio = struct.unpack_from('!iii', payload, 1)
        enc_inverted = bool(payload[13])
        return linkage / 1e7, enc_offset / 1e6, enc_ratio / 1e6, enc_inverted

    def detect_encoder(self, current, timeout=30.0):
        """Send COMM_DETECT_ENCODER and return (offset_deg, ratio, inverted).

        Spins the motor briefly to find the angular offset between electrical
        zero and encoder zero. Returns offset=1001.0 when the encoder is not
        configured in the VESC firmware.

        Request: current [A] × 1e3
        Response: offset [deg] × 1e6, ratio × 1e6, inverted (u8)
        """
        _CMD = 27  # COMM_DETECT_ENCODER
        params = struct.pack('!i', int(current * 1e3))
        payload = self._dispatcher.request(frame(bytes([_CMD]) + params), _CMD, timeout=timeout)
        if len(payload) < 10:
            raise TimeoutError(f"Malformed response to COMM_DETECT_ENCODER")
        offset, ratio = struct.unpack_from('!ii', payload, 1)
        inverted = bool(payload[9])
        return offset / 1e6, ratio / 1e6, inverted

    def get_mcconf_default_motor_params(self, timeout=5.0):
        """Send COMM_GET_MCCONF_DEFAULT and return (l_h, ld_lq_diff_h, r_ohm,
        flux_linkage_wb) from the firmware's compiled-in lab-reference motor
        configuration (mcconf_zerno_drive.h MCCONF_FOC_MOTOR_* constants).

        Only the four foc_motor_* fields are decoded; the rest of the
        ~150-field mc_configuration blob is skipped by byte width. Offsets
        are fixed by the field order in confgenerator.c and don't depend on
        field values, since every field type has a constant width.
        """
        _CMD = 15  # COMM_GET_MCCONF_DEFAULT
        payload = self._dispatcher.request(frame(bytes([_CMD])), _CMD, timeout=timeout)
        if len(payload) < 174:
            raise TimeoutError(f"Malformed response to COMM_GET_MCCONF_DEFAULT")
        l_raw, ld_lq_raw, r_raw, flux_raw = struct.unpack_from('!IIII', payload, 158)
        return (
            _decode_float32_auto(l_raw),
            _decode_float32_auto(ld_lq_raw),
            _decode_float32_auto(r_raw),
            _decode_float32_auto(flux_raw),
        )

    def erase_new_app(self, size, timeout=20.0):
        """Send COMM_ERASE_NEW_APP and wait for the erase-complete response.

        Erases the "new app" staging region (separate from the currently
        running application) ahead of a USB firmware update. Erasing whole
        flash sectors is slow — VESC Tool allows up to 20s for this.

        :param size: number of bytes to erase in the staging region.
        :return: True if the firmware reported success.
        """
        _CMD = 2  # COMM_ERASE_NEW_APP
        payload = self._dispatcher.request(
            frame(bytes([_CMD]) + struct.pack('!I', size)), _CMD, timeout=timeout
        )
        if len(payload) < 2:
            raise TimeoutError(f"Malformed response to COMM_ERASE_NEW_APP")
        return bool(payload[1])

    def write_new_app_data(self, offset, data, timeout=3.0):
        """Send one COMM_WRITE_NEW_APP_DATA chunk and wait for its ack.

        Writes into the "new app" staging region only — never touches the
        currently running application, so this is safe to call even if a
        later step (jump_to_bootloader) never happens.

        :param offset: byte offset within the staging region.
        :param data: raw chunk bytes. Keep chunks well under
            PACKET_MAX_PL_LEN (512) minus 5 bytes of header; VESC Tool uses
            384-byte chunks.
        :return: True if the firmware reported success for this chunk.
        """
        _CMD = 3  # COMM_WRITE_NEW_APP_DATA
        payload = self._dispatcher.request(
            frame(bytes([_CMD]) + struct.pack('!I', offset) + bytes(data)), _CMD, timeout=timeout
        )
        if len(payload) < 2:
            raise TimeoutError(f"Malformed response to COMM_WRITE_NEW_APP_DATA")
        return bool(payload[1])

    def jump_to_bootloader(self):
        """Send COMM_JUMP_TO_BOOTLOADER.

        No response is expected: the device tears down its USB connection
        and reboots into the bootloader immediately, which validates and
        applies whatever was staged via erase_new_app()/write_new_app_data()
        before booting the new application. This is the point of no return
        in a firmware update — call only once every chunk has been
        acknowledged successfully.
        """
        _CMD = 1  # COMM_JUMP_TO_BOOTLOADER
        self._dispatcher.send(frame(bytes([_CMD])))

    def set_rotor_position_mode(self, mode):
        """Set the firmware's periodic-thread position-report mode (COMM_SET_DETECT).

        Use SetRotorPositionMode.DISP_POS_MODE_ENCODER to make the firmware
        broadcast raw encoder angle via unsolicited COMM_ROTOR_POSITION packets
        every 10ms. Use SetRotorPositionMode.DISP_POS_OFF to stop the broadcast.
        """
        self._dispatcher.send(encode(SetRotorPositionMode(mode)))

    def stream_rotor_positions(self, max_duration_s, poll_interval_s=0.05):
        """Yield (timestamp, angle_deg) for each COMM_ROTOR_POSITION packet
        received, for up to max_duration_s seconds.

        Requires set_rotor_position_mode(DISP_POS_MODE_ENCODER) to have been
        called first; the caller is responsible for turning it back off
        afterward. Samples are delivered as they arrive (via the dispatcher's
        subscribe mechanism) rather than batched on a drain interval, so
        nothing is skipped even under concurrent telemetry/other traffic.
        """
        _CMD = 22  # COMM_ROTOR_POSITION
        q = queue.Queue()

        def on_rotor_position(payload):
            if len(payload) >= 5:
                angle = struct.unpack_from('!i', payload, 1)[0] / 100000.0
                q.put((time.monotonic(), angle))

        unsubscribe = self._dispatcher.subscribe(_CMD, on_rotor_position)
        try:
            deadline = time.monotonic() + max_duration_s
            wait_slice = poll_interval_s if poll_interval_s > 0 else 0.05
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return
                try:
                    yield q.get(timeout=min(remaining, wait_slice))
                except queue.Empty:
                    continue
        finally:
            unsubscribe()

    def get_rpm(self):
        """
        :return: Current motor rpm
        """
        return self.get_measurements().rpm

    def get_duty_cycle(self):
        """
        :return: Current applied duty-cycle
        """
        return self.get_measurements().duty_now

    def get_v_in(self):
        """
        :return: Current input voltage
        """
        return self.get_measurements().v_in

    def get_motor_current(self):
        """
        :return: Current motor current
        """
        return self.get_measurements().current_motor

    def get_incoming_current(self):
        """
        :return: Current incoming current
        """
        return self.get_measurements().current_in
