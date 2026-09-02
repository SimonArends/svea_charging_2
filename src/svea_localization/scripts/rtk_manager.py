#!/usr/bin/env python3

"""
Author: Mustafa Al-Janabi
"""
from serial import Serial, SerialException, EIGHTBITS, PARITY_NONE, STOPBITS_ONE
from pyubx2 import (
    UBXReader,
    SET,
    UBXMessage,
    protocol,
    NMEA_PROTOCOL,  # 1
    UBX_PROTOCOL,  # 2
    RTCM3_PROTOCOL,  # 4
)

from time import monotonic, sleep
import rclpy
import rclpy.clock
from rclpy.node import Node
from threading import Thread
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nmea_msgs.msg import Sentence
from std_msgs.msg import Bool, Float64, UInt8, UInt16
from rtcm_msgs.msg import Message


# MAP dynamic model string to corresponding number
# source https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/CfgNAV5.msg
DYN_MODEL_MAP = {
    "portable": 0,
    "stationary": 1,    
    "pedestrian": 3,
    "automotive": 4,
    "sea": 5,
    "airborne_1g": 6,  # Airborne with <1g Acceleration
    "airborne_2g": 7,  # Airborne with <2g Acceleration
    "airborne_4g": 8,  # Airborne with <4g Acceleration
    "wrist_watch": 9,
    "bike": 10,
}
# UBX NAV-PVT fixType is a navigation-fix type, not an RTK quality enum.
# RTK float/fixed is carried separately in NAV-PVT carrSoln.
GPS_QUALITIES = {
    0: NavSatStatus.STATUS_NO_FIX,
    1: NavSatStatus.STATUS_NO_FIX,  # dead reckoning only
    2: NavSatStatus.STATUS_FIX,     # 2D GNSS fix
    3: NavSatStatus.STATUS_FIX,     # 3D GNSS fix
    4: NavSatStatus.STATUS_FIX,     # GNSS + dead reckoning
    5: NavSatStatus.STATUS_NO_FIX,  # time-only fix
}


class RTKManager(Node):
    def __init__(self):
        super().__init__("rtk_manager")
        # Read parameters
        self.declare_parameter('device', '/dev/ttyACM0')
        self.device = self.get_parameter('device').value #UART: /dev/ttyS0
        self.declare_parameter('receiver_interface', 'usb')
        self.receiver_interface = self.get_parameter('receiver_interface').value.lower()
        if self.receiver_interface == 'uart1':
            self.rateUART1 = 1
            self.rateUSB = 0
        elif self.receiver_interface == 'usb':
            self.rateUART1 = 0
            self.rateUSB = 1
        else:
            raise ValueError(
                "Invalid receiver_interface. Expected 'uart1' or 'usb', "
                f"got '{self.receiver_interface}'."
            )
        self.declare_parameter('baud', 250000)
        self.baud = self.get_parameter('baud').value #UART: 38400
        self.declare_parameter('gps_frame', 'gps')
        self.frame_id = self.get_parameter('gps_frame').value
        self.declare_parameter('dynamic_model', 'portable')
        self.dynamic_model = self.get_parameter('dynamic_model').value
        #  Open serial port
        try:
            self.serial = Serial(self.device, self.baud, bytesize=EIGHTBITS, parity=PARITY_NONE,stopbits=STOPBITS_ONE,timeout=1, exclusive=True)
        except SerialException as ex:
            message = "Could not open serial port: I/O error({0}): {1}".format(
                ex.errno, ex.strerror
            )
            self.get_logger().fatal(message)
            raise RuntimeError(message) from ex
        # Initialise the ubx reader
        self.ubx_reader = UBXReader(
            self.serial,
            protfilter=UBX_PROTOCOL + NMEA_PROTOCOL + RTCM3_PROTOCOL,
        )  # Parse every protocol that can be present on the bidirectional bridge.
        # Create publishers
        self._init_pub()
        # Create subscriber
        self._init_sub()
        # Configure the receiver
        self.setup_receiver()
        # Start reading from serial port and parse messages using ubx_reader
        self.start_serial_read()

    def _init_pub(self):
        """Initializes publishers for necessary and sufficient topics"""
        # Nmea message which get sent to virtual NTRIP servers which give correction message from closes base station based on own location
        self.nmea_pub = self.create_publisher(Sentence, "nmea", 10)
        # Publish the satellite fix
        self.fix_pub = self.create_publisher(NavSatFix, "fix", 10)
        # Heading of 2-D motion in [deg]
        self.heading_motion_pub = self.create_publisher(Float64, "heading_motion", 10)
        # Heading of vehicle in 2-D in [deg]
        self.heading_vehicle_pub = self.create_publisher(Float64, "heading_vehicle", 10)
        # Combined heading accuracy of vehicle and motion headings in [deg]
        self.headingAcc_pub = self.create_publisher(Float64, "heading_accuracy", 10)
        # Ground speed (2-D) in [m/s]
        self.speed_pub = self.create_publisher(Float64, "speed", 10)
        # Estimate of ground speed accuracy in [m/s]
        self.speedAcc_pub = self.create_publisher(Float64, "speed_accuracy", 10)
        # Magnetic declination in [deg]
        self.magDec_pub = self.create_publisher(Float64, "magnetic_declination", 10)
        # Accuracy of magnetic declination in [deg]
        self.magDecAcc_pub = self.create_publisher(Float64, "magnetic_declination_accuracy", 10)
        # Explicit receiver-quality topics. NavSatStatus cannot represent
        # u-blox RTK float/fixed without losing information.
        self.fix_type_pub = self.create_publisher(UInt8, "fix_type", 10)
        self.diff_solution_pub = self.create_publisher(Bool, "differential_solution", 10)
        self.carrier_solution_pub = self.create_publisher(UInt8, "carrier_solution", 10)
        self.satellites_pub = self.create_publisher(UInt8, "satellites_visible", 10)
        self.horizontal_accuracy_pub = self.create_publisher(Float64, "horizontal_accuracy", 10)
        self.vertical_accuracy_pub = self.create_publisher(Float64, "vertical_accuracy", 10)
        self.position_covariance_valid_pub = self.create_publisher(
            Bool, "position_covariance_valid", 10
        )
        self.rtcm_crc_failed_pub = self.create_publisher(Bool, "rtcm_crc_failed", 10)
        self.rtcm_message_used_pub = self.create_publisher(UInt8, "rtcm_message_used", 10)
        self.rtcm_message_type_pub = self.create_publisher(UInt16, "rtcm_message_type", 10)
        self.rtcm_reference_station_pub = self.create_publisher(UInt16, "rtcm_reference_station", 10)


    def _init_sub(self):
        """Initialize subscribers"""
        # Subscribe to RTCM correction messages from NTRIP Client
        self.create_subscription(Message, "rtcm", self._handle_rtcm_cb, 10)

    def set_config(self, msgClass, msgID, **kwargs):
        """Utility function which write a configuration message to receiver and awaits an acknowledgement."""
        cfg = UBXMessage(
            "CFG", "CFG-MSG", SET, msgClass=msgClass, msgID=msgID, **kwargs
        )
        self._send_and_wait_for_ack(
            cfg,
            f"CFG-MSG for msgClass:{msgClass} msgID:{msgID}",
        )

    def _send_and_wait_for_ack(self, cfg, description, timeout=5.0):
        """Send a UBX configuration and wait for its ACK response."""
        self.serial.write(cfg.serialize())
        deadline = monotonic() + timeout

        while rclpy.ok() and monotonic() < deadline:
            _, parsed_msg = self.ubx_reader.read()
            if parsed_msg is None:
                continue
            if parsed_msg.identity == "ACK-ACK":
                return
            if parsed_msg.identity == "ACK-NAK":
                raise RuntimeError(f"Receiver rejected {description}")

        if not rclpy.ok():
            raise RuntimeError(f"Interrupted while waiting for ACK for {description}")
        raise TimeoutError(f"Timed out waiting for ACK for {description}")

    def set_dynamic_model(self, model):
        # CFG-MSG-NAV5 set dynModel (dynamic Model) to model
        # To understand the CFG-NAV5 msg https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/CfgNAV5.msg
        # and https://github.com/semuconsulting/pyubx2/blob/935f678a78a1038860d07aa64e600505bdc7ac00/src/pyubx2/ubxtypes_get.py#L566C6-L600
        if DYN_MODEL_MAP.get(model, None) is None:
            self.get_logger().warn(
                f'Invalid Dynamic Model Provided: {model}. Supported models are {", ".join(DYN_MODEL_MAP.keys())}.'
            )
        else:
            cfg = UBXMessage(
                "CFG",
                "CFG-NAV5",
                SET,
                msgClass=0x06,
                msgID=0x24,
                dynModel=DYN_MODEL_MAP.get(model, 0),
                dyn=1,  # MASK to update only dynamic model
            )
            self._send_and_wait_for_ack(
                cfg,
                f"CFG-NAV5 for dynamic model:{model}",
            )

    def setup_receiver(self):
        
        # CFG-MSG-NAV-STATUS set rateUSB to 0
        while rclpy.ok():
            try:
                self.set_config(0x01, 0x03, rateUART1=self.rateUART1, rateUSB=self.rateUSB)
            except Exception as e:
                self.get_logger().error(f"Error occurred while setting up receiver: {e}")
                sleep(2)
            else:
                break

        # Disable high-volume raw measurement messages that are not used by
        # this node. Keeping them enabled can overflow a UART bridge.
        self.set_config(0x02, 0x15, rateUART1=0, rateUSB=0)  # RXM-RAWX
        self.set_config(0x02, 0x13, rateUART1=0, rateUSB=0)  # RXM-SFRBX

        # NTRIP only needs GGA. Position, velocity, and covariance are taken
        # from UBX NAV messages, so disable the other periodic NMEA messages.
        self.set_config(
            0xF0, 0x00, rateUART1=self.rateUART1, rateUSB=self.rateUSB
        )  # NMEA-GGA
        for nmea_msg_id in (0x01, 0x02, 0x03, 0x04, 0x05):
            self.set_config(
                0xF0, nmea_msg_id, rateUART1=0, rateUSB=0
            )

        # CFG-MSG-NAV-PVT set rateUSB to 1
        self.set_config(0x01, 0x07, rateUART1=self.rateUART1, rateUSB=self.rateUSB)
        
        # CFG-MSG-NAV-COV set rateUSB to 1
        self.set_config(0x01, 0x36, rateUART1=self.rateUART1, rateUSB=self.rateUSB)
        
        # CFG-MSG-RXM-RTCM set rateUSB to 1
        self.set_config(0x02, 0x32, rateUART1=self.rateUART1, rateUSB=self.rateUSB)
        
        # CFG-NAV5 set dynModel to self.dynamic_model
        self.set_dynamic_model(self.dynamic_model)

    def start_serial_read(self):
        """Start new thread which reads from the serial port and handles the incoming messages from the receiver."""
        self.nav_sat_fix_msg = NavSatFix()
        Thread(target=self._read_serial_handler, args=(), daemon=True).start()


    def _handle_rtcm_cb(self, msg):
        """Callback which listens to RTCM messages from NTRIP clients and writes them to the receiver."""
        raw_rtcm = bytes(msg.message)
        try:
            written = self.serial.write(raw_rtcm)
        except SerialException as ex:
            self.get_logger().error(f"Failed to write RTCM to receiver: {ex}")
            return
        if written != len(raw_rtcm):
            self.get_logger().error(
                f"Incomplete RTCM serial write: {written}/{len(raw_rtcm)} bytes"
            )

    def _read_serial_handler(self):
        """Continuously read and handle incoming serial messages."""
        while rclpy.ok():
            raw_msg, parsed_msg = self.ubx_reader.read()
            if not raw_msg or parsed_msg is None:
                continue
            msg_protocol = protocol(raw_msg)
            if msg_protocol == UBX_PROTOCOL:
                self.nav_sat_fix_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
                self.nav_sat_fix_msg.header.frame_id = self.frame_id
                if parsed_msg.identity == "NAV-PVT":
                    # Understanding the PVT message
                    # https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavPVT.msg
                    # and
                    # https://github.com/semuconsulting/pyubx2/blob/master/src/pyubx2/ubxtypes_get.py

                    fix_type = int(parsed_msg.fixType)
                    carrier_solution = int(getattr(parsed_msg, "carrSoln", 0))
                    differential_solution = bool(getattr(parsed_msg, "diffSoln", 0))
                    self.fix_type_pub.publish(UInt8(data=fix_type))
                    self.diff_solution_pub.publish(Bool(data=differential_solution))
                    self.carrier_solution_pub.publish(UInt8(data=carrier_solution))
                    self.satellites_pub.publish(UInt8(data=int(parsed_msg.numSV)))
                    self.horizontal_accuracy_pub.publish(
                        Float64(data=float(parsed_msg.hAcc) * 1e-3)
                    )
                    self.vertical_accuracy_pub.publish(
                        Float64(data=float(parsed_msg.vAcc) * 1e-3)
                    )
                    self.nav_sat_fix_msg.status.status = (
                        GPS_QUALITIES.get(fix_type, NavSatStatus.STATUS_NO_FIX)
                        if parsed_msg.gnssFixOk
                        else NavSatStatus.STATUS_NO_FIX
                    )
                    self.nav_sat_fix_msg.status.service = NavSatStatus.SERVICE_GPS

                    # Only use position and motion fields from valid fixes.
                    if parsed_msg.gnssFixOk:
                        # Comment out the following to debug with a print
                        # lon = parsed_msg.lon
                        # lat = parsed_msg.lat
                        # height = parsed_msg.height / 1e3  # [m]
                        # horz_acc = parsed_msg.hAcc / 1e3  # [m]
                        # vert_acc = parsed_msg.vAcc / 1e3  # [m]
                        # fix_type = parsed_msg.fixType
                        # print(
                        #     parsed_msg.iTOW,
                        #     lat,
                        #     lon,
                        #     height,
                        #     "m",
                        #     parsed_msg.hAcc / 10,
                        #     "cm",
                        #     parsed_msg.vAcc / 10,
                        #     "cm",
                        #     "fix",
                        #     fix_type,
                        # )

                        self.nav_sat_fix_msg.latitude = parsed_msg.lat  # [deg]
                        self.nav_sat_fix_msg.longitude = parsed_msg.lon  # [deg]
                        self.nav_sat_fix_msg.altitude = parsed_msg.height / 1e3  # [m]
                        # Publish Speed
                        self.speed_pub.publish(
                            Float64(data=parsed_msg.gSpeed * 1e-3)
                        )  # [m/s] convert from mm/s to m/s
                        self.speedAcc_pub.publish(
                            Float64(data=parsed_msg.sAcc * 1e-3)
                        )  # [m/s] convert from mm/s to m/s
                        # Publish Heading
                        self.heading_motion_pub.publish(
                            # Float64(data=parsed_msg.headMot * 1e-5)
                            Float64(data=parsed_msg.headMot)
                        )  # [deg]
                        self.heading_vehicle_pub.publish(
                            # Float64(data=parsed_msg.headVeh * 1e-5)
                            Float64(data=parsed_msg.headVeh)
                        )  # [deg]
                        self.headingAcc_pub.publish(
                            # Float64(data=parsed_msg.headAcc * 1e-5)
                            Float64(data=parsed_msg.headAcc)
                        )  # [deg]
                        # Publish magDec
                        self.magDec_pub.publish(
                            # Float64(data=parsed_msg.magDec * 1e-2)
                            Float64(data=parsed_msg.magDec)
                        )  # [deg]
                        self.magDecAcc_pub.publish(
                            # Float64(data=parsed_msg.magAcc * 1e-2)
                            Float64(data=parsed_msg.magAcc)
                        )  # [deg]

                elif parsed_msg.identity == "RXM-RTCM":
                    # Receiver-side confirmation that RTCM crossed the serial
                    # bridge. pyubx2 exposes the bit fields on supported UBX
                    # protocol versions; fall back to decoding flags.
                    flags = int(getattr(parsed_msg, "flags", 0))
                    crc_failed = bool(getattr(parsed_msg, "crcFailed", flags & 0x01))
                    message_used = int(getattr(parsed_msg, "msgUsed", (flags >> 1) & 0x03))
                    message_type = int(getattr(parsed_msg, "msgType", 0))
                    reference_station = int(getattr(parsed_msg, "refStation", 0))
                    self.rtcm_crc_failed_pub.publish(Bool(data=crc_failed))
                    self.rtcm_message_used_pub.publish(UInt8(data=message_used))
                    self.rtcm_message_type_pub.publish(UInt16(data=message_type))
                    self.rtcm_reference_station_pub.publish(UInt16(data=reference_station))

                # https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/CfgNAV5.msg
                elif parsed_msg.identity == "NAV-COV":
                    # NAV-COV is always sent directly after NAV-PVT, publish upon NAV-COV arrival
                    # Building a covariance matrix in the ENU frame
                    # [EE EN EU
                    #  NE NN NU
                    #  UE UN UU]
                    # However ZED-F9P gives us NED so we use the following equivalent form
                    # [ EE  EN -ED
                    #   NE  NN -ND
                    #  -DE -DN  DD]

                    # In some other Ublox GPS library the covariance is estimated by
                    # computing the diagonal elements using the hAcc, and vAcc values in the
                    # PVT message. In such case we convert the values to meters and raise by 2 to g
                    # an approximation of the
                    # positive_covariance[0] = (pvt_msg.hAcc / 1e3) ** 2
                    # positive_covariance[4] = (pvt_msg.hAcc / 1e3) ** 2
                    # positive_covariance[4] = (pvt_msg.vAcc / 1e3) ** 2
                    # see https://github.com/KumarRobotics/ublox/blob/4f107f3b82135160a1aca3ef0689fd119199bbef/ublox_gps/src/node.cpp#LL779C1-L787C62
                    # However, using the NAV-COV message gives more accurate results
                    # and experimentation shows that the diagonal approximation methods gives
                    # (EE + NN) is approx. (pvt_msg.hAcc / 1e3) ** 2
                    # Ie. the east-east and north-north component add up to the approximated
                    # diagonal values horizontally.
                    # DD is almost the same as (pvt_msg.vAcc / 1e3) ** 2 which
                    # means that both methods which approximate the vertical covariance are in agreements.
                    # Of course, the additional benefit of using the NAV-COV message is that we also get
                    # the covariance values for the cross-terms.

                    covariance_valid = bool(parsed_msg.posCovValid)
                    self.position_covariance_valid_pub.publish(
                        Bool(data=covariance_valid)
                    )
                    if covariance_valid:
                        ee = parsed_msg.posCovEE
                        ne = parsed_msg.posCovNE
                        ed = parsed_msg.posCovED
                        nn = parsed_msg.posCovNN
                        nd = parsed_msg.posCovND
                        dd = parsed_msg.posCovDD
                        self.nav_sat_fix_msg.position_covariance = [
                            ee, ne, -ed,
                            ne, nn, -nd,
                            -ed, -nd, dd,
                        ]
                        self.nav_sat_fix_msg.position_covariance_type = (
                            self.nav_sat_fix_msg.COVARIANCE_TYPE_KNOWN
                        )
                    else:
                        self.nav_sat_fix_msg.position_covariance = [0.0] * 9
                        self.nav_sat_fix_msg.position_covariance_type = (
                            self.nav_sat_fix_msg.COVARIANCE_TYPE_UNKNOWN
                        )
                    # Publish fix data
                    self.fix_pub.publish(self.nav_sat_fix_msg)
                    # Reset fix message after publishing
                    self.nav_sat_fix_msg = NavSatFix()
            if msg_protocol == NMEA_PROTOCOL:
                try:
                    nmea_str = raw_msg.decode("ascii")
                    nmea_sentence_msg = Sentence()
                    nmea_sentence_msg.sentence = nmea_str
                    nmea_sentence_msg.header.frame_id = self.frame_id
                    nmea_sentence_msg.header.stamp = self.get_clock().now().to_msg()
                    self.nmea_pub.publish(nmea_sentence_msg)

                except UnicodeError as e:
                    self.get_logger().warn(
                        "Skipped adding a NMEA sentence from serial device becuase it could not be decoded as an ASCII string. The bytes were {0}".format(
                            raw_msg
                        )
                    )

    def destroy_node(self):
        if hasattr(self, "serial") and self.serial.is_open:
            self.serial.close()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    rtk_manager = None
    try:
        rtk_manager = RTKManager()
        rclpy.spin(rtk_manager)
    except KeyboardInterrupt:
        pass
    finally:
        if rtk_manager is not None:
            rtk_manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
