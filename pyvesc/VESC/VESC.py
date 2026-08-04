from pyvesc.protocol.interface import encode_request, encode, decode
from pyvesc.protocol.packet.codec import unframe
from pyvesc.VESC.messages import *
import math
import time
import threading

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
        if has_sensor:
            self.serial_port.write(encode(SetRotorPositionMode(SetRotorPositionMode.DISP_POS_OFF)))

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
        msg = GetValues()
        self._get_values_msg = encode_request(msg)
        self._get_values_msg_expected_length = msg._full_msg_size

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_heartbeat()
        if self.serial_port.is_open:
            self.serial_port.flush()
            self.serial_port.close()

    def _heartbeat_cmd_func(self):
        """
        Continuous function calling that keeps the motor alive
        """
        while not self._stop_heartbeat.is_set():
            time.sleep(0.1)
            try:
                for i in self.alive_msg:
                    self.write(i)
            except serial.SerialException:
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

    def write(self, data, num_read_bytes=None):
        """
        A write wrapper function implemented like this to try and make it easier to incorporate other communication
        methods than UART in the future.
        :param data: the byte string to be sent
        :param num_read_bytes: number of bytes to read for decoding response
        :return: decoded response from buffer
        """
        self.serial_port.write(data)
        if num_read_bytes is not None:
            while self.serial_port.in_waiting <= num_read_bytes:
                time.sleep(0.000001)  # add some delay just to help the CPU
            time.sleep(0.01)  # let the rest of the packet arrive before reading
            response, consumed = decode(self.serial_port.read(self.serial_port.in_waiting))
            self.serial_port.reset_input_buffer()  # flush any unprocessed leftover bytes
            return response

    def set_rpm(self, new_rpm, **kwargs):
        """
        Set the electronic RPM value (a.k.a. the RPM value of the stator)
        :param new_rpm: new rpm value
        """
        self.write(encode(SetRPM(new_rpm, **kwargs)))

    def set_current(self, new_current, **kwargs):
        """
        :param new_current: new current in milli-amps for the motor
        """
        self.write(encode(SetCurrent(new_current, **kwargs)))

    def set_duty_cycle(self, new_duty_cycle, **kwargs):
        """
        :param new_duty_cycle: Value of duty cycle to be set (range [-1e5, 1e5]).
        """
        self.write(encode(SetDutyCycle(new_duty_cycle, **kwargs)))

    def set_servo(self, new_servo_pos, **kwargs):
        """
        :param new_servo_pos: New servo position. valid range [0, 1]
        """
        self.write(encode(SetServoPosition(new_servo_pos, **kwargs)))

    def get_measurements(self):
        """
        :return: A msg object with attributes containing the measurement values
        """
        return self.write(self._get_values_msg, num_read_bytes=self._get_values_msg_expected_length)

    def get_firmware_version(self):
        msg = GetVersion()
        return str(self.write(encode_request(msg), num_read_bytes=msg._full_msg_size))

    def send_terminal_cmd(self, cmd, timeout=1.0):
        """Send a terminal command string and return all COMM_PRINT responses concatenated.

        :param cmd: Command string (e.g. "faults")
        :param timeout: Maximum seconds to wait for the full response.
        :return: Response text string, or None if no response received.
        """
        self.serial_port.reset_input_buffer()
        self.serial_port.write(encode(TerminalCmd(cmd)))

        # Accumulate bytes until no new data arrives for two serial-timeout periods.
        # Serial timeout is 0.05 s, so two consecutive empty reads = ~100 ms of silence.
        accumulated = b''
        deadline = time.time() + timeout
        consecutive_empty = 0
        while time.time() < deadline:
            waiting = self.serial_port.in_waiting
            if waiting > 0:
                accumulated += self.serial_port.read(waiting)
                consecutive_empty = 0
            else:
                chunk = self.serial_port.read(1)
                if chunk:
                    accumulated += chunk
                    consecutive_empty = 0
                else:
                    consecutive_empty += 1
                    if consecutive_empty >= 2 and accumulated:
                        break

        if not accumulated:
            return None

        messages = []
        buf = accumulated
        while buf:
            response, consumed = decode(buf)
            if consumed == 0:
                break
            buf = buf[consumed:]
            if response is not None and hasattr(response, 'message'):
                messages.append(response.message)

        return ''.join(messages) if messages else None

    def get_fw_info(self):
        """Request COMM_FW_INFO and return (fw_major, fw_minor, fw_test, git_hash, user_git_hash).

        Parses the response manually because it contains two null-terminated
        strings which VESCMessage fields do not support simultaneously.
        Returns None if the response cannot be parsed.
        """
        self.serial_port.reset_input_buffer()
        self.serial_port.write(encode_request(GetFwInfo))
        time.sleep(0.1)
        raw = self.serial_port.read(self.serial_port.in_waiting)
        self.serial_port.reset_input_buffer()

        payload, _ = unframe(raw)
        if payload is None or len(payload) < 5:
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

    def get_mcu_uuid(self):
        """Extract the 12-byte STM32 UUID from a COMM_FW_VERSION response.

        COMM_FW_VERSION payload layout:
            [0]       cmd_id (COMM_FW_VERSION)
            [1]       fw_major
            [2]       fw_minor
            [3..N]    hw_name (null-terminated)
            [N+1..N+12] STM32 UUID (12 bytes)

        Returns a 24-character uppercase hex string, or None if the response
        cannot be parsed or UUID bytes are missing/all-zero.
        """
        msg = GetVersion()
        self.serial_port.reset_input_buffer()
        self.serial_port.write(encode_request(msg))
        time.sleep(0.1)
        raw = self.serial_port.read(self.serial_port.in_waiting)
        self.serial_port.reset_input_buffer()

        payload, _ = unframe(raw)
        if payload is None or len(payload) < 4:
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

    def detect_motor_rl(self, timeout=30.0):
        """Send COMM_DETECT_MOTOR_R_L and return (r_ohm, l_henry, ld_lq_diff_henry).

        Blocks in the VESC firmware for several seconds while it applies current
        to measure resistance and inductance. Returns (0.0, 0.0, 0.0) when the
        VESC reports a fault (e.g. no motor connected).
        """
        from pyvesc.protocol.packet.codec import frame, unframe
        import struct
        _CMD = 25  # COMM_DETECT_MOTOR_R_L
        self.serial_port.reset_input_buffer()
        self.serial_port.write(frame(bytes([_CMD])))
        buf = b''
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            time.sleep(0.1)
            if self.serial_port.in_waiting:
                buf += self.serial_port.read(self.serial_port.in_waiting)
            payload, consumed = unframe(buf)
            buf = buf[consumed:]
            if payload and len(payload) >= 13 and payload[0] == _CMD:
                r, l, ld_lq_diff = struct.unpack_from('!iii', payload, 1)
                return r / 1e6, l / 1e3, ld_lq_diff / 1e3
        raise TimeoutError(f"No response to COMM_DETECT_MOTOR_R_L after {timeout:.0f}s")

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
        from pyvesc.protocol.packet.codec import frame, unframe
        import struct
        _CMD = 57  # COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP
        params = struct.pack('!iiiii',
            int(current      * 1e3),
            int(erpm_per_sec * 1e3),
            int(duty         * 1e3),
            int(resistance   * 1e6),
            int(inductance   * 1e8),
        )
        self.serial_port.reset_input_buffer()
        self.serial_port.write(frame(bytes([_CMD]) + params))
        buf = b''
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            time.sleep(0.1)
            if self.serial_port.in_waiting:
                buf += self.serial_port.read(self.serial_port.in_waiting)
            payload, consumed = unframe(buf)
            buf = buf[consumed:]
            if payload and len(payload) >= 14 and payload[0] == _CMD:
                linkage, enc_offset, enc_ratio = struct.unpack_from('!iii', payload, 1)
                enc_inverted = bool(payload[13])
                return linkage / 1e7, enc_offset / 1e6, enc_ratio / 1e6, enc_inverted
        raise TimeoutError(
            f"No response to COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP after {timeout:.0f}s"
        )

    def detect_encoder(self, current, timeout=30.0):
        """Send COMM_DETECT_ENCODER and return (offset_deg, ratio, inverted).

        Spins the motor briefly to find the angular offset between electrical
        zero and encoder zero. Returns offset=1001.0 when the encoder is not
        configured in the VESC firmware.

        Request: current [A] × 1e3
        Response: offset [deg] × 1e6, ratio × 1e6, inverted (u8)
        """
        from pyvesc.protocol.packet.codec import frame, unframe
        import struct
        _CMD = 27  # COMM_DETECT_ENCODER
        params = struct.pack('!i', int(current * 1e3))
        self.serial_port.reset_input_buffer()
        self.serial_port.write(frame(bytes([_CMD]) + params))
        buf = b''
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            time.sleep(0.1)
            if self.serial_port.in_waiting:
                buf += self.serial_port.read(self.serial_port.in_waiting)
            payload, consumed = unframe(buf)
            buf = buf[consumed:]
            if payload and len(payload) >= 10 and payload[0] == _CMD:
                offset, ratio = struct.unpack_from('!ii', payload, 1)
                inverted = bool(payload[9])
                return offset / 1e6, ratio / 1e6, inverted
        raise TimeoutError(f"No response to COMM_DETECT_ENCODER after {timeout:.0f}s")

    def get_mcconf_default_motor_params(self, timeout=5.0):
        """Send COMM_GET_MCCONF_DEFAULT and return (l_h, ld_lq_diff_h, r_ohm,
        flux_linkage_wb) from the firmware's compiled-in lab-reference motor
        configuration (mcconf_zerno_drive.h MCCONF_FOC_MOTOR_* constants).

        Only the four foc_motor_* fields are decoded; the rest of the
        ~150-field mc_configuration blob is skipped by byte width. Offsets
        are fixed by the field order in confgenerator.c and don't depend on
        field values, since every field type has a constant width.
        """
        from pyvesc.protocol.packet.codec import frame, unframe
        import struct
        _CMD = 15  # COMM_GET_MCCONF_DEFAULT
        self.serial_port.reset_input_buffer()
        self.serial_port.write(frame(bytes([_CMD])))
        buf = b''
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            time.sleep(0.1)
            if self.serial_port.in_waiting:
                buf += self.serial_port.read(self.serial_port.in_waiting)
            payload, consumed = unframe(buf)
            buf = buf[consumed:]
            if payload and len(payload) >= 174 and payload[0] == _CMD:
                l_raw, ld_lq_raw, r_raw, flux_raw = struct.unpack_from('!IIII', payload, 158)
                return (
                    _decode_float32_auto(l_raw),
                    _decode_float32_auto(ld_lq_raw),
                    _decode_float32_auto(r_raw),
                    _decode_float32_auto(flux_raw),
                )
        raise TimeoutError(f"No response to COMM_GET_MCCONF_DEFAULT after {timeout:.0f}s")

    def erase_new_app(self, size, timeout=20.0):
        """Send COMM_ERASE_NEW_APP and wait for the erase-complete response.

        Erases the "new app" staging region (separate from the currently
        running application) ahead of a USB firmware update. Erasing whole
        flash sectors is slow — VESC Tool allows up to 20s for this.

        :param size: number of bytes to erase in the staging region.
        :return: True if the firmware reported success.
        """
        from pyvesc.protocol.packet.codec import frame, unframe
        import struct
        _CMD = 2  # COMM_ERASE_NEW_APP
        self.serial_port.reset_input_buffer()
        self.serial_port.write(frame(bytes([_CMD]) + struct.pack('!I', size)))
        buf = b''
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            time.sleep(0.1)
            if self.serial_port.in_waiting:
                buf += self.serial_port.read(self.serial_port.in_waiting)
            payload, consumed = unframe(buf)
            buf = buf[consumed:]
            if payload and len(payload) >= 2 and payload[0] == _CMD:
                return bool(payload[1])
        raise TimeoutError(f"No response to COMM_ERASE_NEW_APP after {timeout:.0f}s")

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
        from pyvesc.protocol.packet.codec import frame, unframe
        import struct
        _CMD = 3  # COMM_WRITE_NEW_APP_DATA
        self.serial_port.reset_input_buffer()
        self.serial_port.write(frame(bytes([_CMD]) + struct.pack('!I', offset) + bytes(data)))
        buf = b''
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            time.sleep(0.01)
            if self.serial_port.in_waiting:
                buf += self.serial_port.read(self.serial_port.in_waiting)
            payload, consumed = unframe(buf)
            buf = buf[consumed:]
            if payload and len(payload) >= 2 and payload[0] == _CMD:
                return bool(payload[1])
        raise TimeoutError(f"No response to COMM_WRITE_NEW_APP_DATA after {timeout:.0f}s")

    def jump_to_bootloader(self):
        """Send COMM_JUMP_TO_BOOTLOADER.

        No response is expected: the device tears down its USB connection
        and reboots into the bootloader immediately, which validates and
        applies whatever was staged via erase_new_app()/write_new_app_data()
        before booting the new application. This is the point of no return
        in a firmware update — call only once every chunk has been
        acknowledged successfully.
        """
        from pyvesc.protocol.packet.codec import frame
        _CMD = 1  # COMM_JUMP_TO_BOOTLOADER
        self.serial_port.write(frame(bytes([_CMD])))

    def set_rotor_position_mode(self, mode):
        """Set the firmware's periodic-thread position-report mode (COMM_SET_DETECT).

        Use SetRotorPositionMode.DISP_POS_MODE_ENCODER to make the firmware
        broadcast raw encoder angle via unsolicited COMM_ROTOR_POSITION packets
        every 10ms. Use SetRotorPositionMode.DISP_POS_OFF to stop the broadcast.
        """
        self.write(encode(SetRotorPositionMode(mode)))

    def stream_rotor_positions(self, max_duration_s, poll_interval_s=0.05):
        """Yield (timestamp, angle_deg) for each COMM_ROTOR_POSITION packet
        received, for up to max_duration_s seconds.

        Requires set_rotor_position_mode(DISP_POS_MODE_ENCODER) to have been
        called first; the caller is responsible for turning it back off
        afterward. Draining happens every poll_interval_s, but since the
        firmware broadcasts every 10ms regardless, multiple buffered packets
        may be decoded per drain — no samples are skipped.
        """
        import struct
        _CMD = 22  # COMM_ROTOR_POSITION
        buf = b''
        t0 = time.monotonic()
        while time.monotonic() - t0 < max_duration_s:
            time.sleep(poll_interval_s)
            if self.serial_port.in_waiting:
                buf += self.serial_port.read(self.serial_port.in_waiting)
            while True:
                payload, consumed = unframe(buf)
                if consumed == 0:
                    break
                buf = buf[consumed:]
                if payload and len(payload) >= 5 and payload[0] == _CMD:
                    angle = struct.unpack_from('!i', payload, 1)[0] / 100000.0
                    yield time.monotonic(), angle

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




