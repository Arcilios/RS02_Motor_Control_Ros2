# robstride_motor_ros/can_node.py

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

# IMPORTANT: make sure this matches your package name
from motor_control_interfaces.srv import RobStrideJointControl
from .robstride_motor_linux import RobStrideMotorLinux


class RobStrideCanNode(Node):
    """
    Low-level CAN node:
    - Manage multiple RobStrideMotorLinux instances indexed by joint name
    - Provide /robstride_joint_control service
    - Periodically publish /joint_states
    """

    def __init__(self):
        super().__init__('motor_control_node')

        # Declare and read default parameters
        self.declare_parameter('default_can_interface', 'can0')
        self.declare_parameter('default_master_id', 255)

        self.default_iface = self.get_parameter(
            'default_can_interface').get_parameter_value().string_value
        self.default_master = self.get_parameter(
            'default_master_id').get_parameter_value().integer_value

        # Read the "motors" sub-namespace
        # raw_motors is like: {'motors.shoulder_pitch.can_interface': ..., ...}
        raw_motors = self.get_parameters_by_prefix('motors')
        self.motors_cfg = self._parse_motors_params(raw_motors)

        # Mapping: joint_name -> RobStrideMotorLinux instance
        self.drivers = {}
        self._create_drivers_from_cfg()

        # Create the joint control service
        self.srv = self.create_service(
            RobStrideJointControl,
            'robstride_joint_control',
            self.handle_joint_control
        )

        # joint_states publisher
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        # 50 Hz timer to publish joint states
        self.timer = self.create_timer(0.02, self.publish_joint_states)

        self.get_logger().info(
            f'RobStride CAN node started with joints: {list(self.drivers.keys())}'
        )

    # -------- Parse config: build motor info dictionary from parameters --------

    def _parse_motors_params(self, raw):
        """
        Convert get_parameters_by_prefix('motors') result into:

        {
          'shoulder_pitch': {
              'can_interface': 'can0',
              'master_id': 255,
              'motor_id': 1,
              'actuator_type': 0
          },
          ...
        }
        """
        motors = {}
        for full_name, param in raw.items():  # full_name like 'motors.shoulder_pitch.can_interface'
            parts = full_name.split('.')
            if len(parts) != 3:
                continue
            _, joint_name, field = parts
            if joint_name not in motors:
                motors[joint_name] = {}
            # Keep both string and numeric parameters
            motors[joint_name][field] = param.value
        return motors

    def _create_drivers_from_cfg(self):
        """Instantiate RobStrideMotorLinux for each joint from the parsed config."""
        for joint_name, cfg in self.motors_cfg.items():
            iface = cfg.get('can_interface', self.default_iface)
            master_id = cfg.get('master_id', self.default_master)
            motor_id = cfg['motor_id']
            actuator_type = cfg.get('actuator_type', 0)

            self.get_logger().info(
                f'Init motor: {joint_name} on {iface}, master={master_id}, '
                f'motor_id={motor_id}, actuator_type={actuator_type}'
            )
            self.drivers[joint_name] = RobStrideMotorLinux(
                iface=iface,
                master_id=master_id,
                motor_id=motor_id,
                actuator_type=actuator_type,
            )

    # -------- Service callback: find driver by joint_name and execute command --------

    def handle_joint_control(self, request, response):
        joint = request.joint_name
        if joint not in self.drivers:
            response.success = False
            response.message = f'Unknown joint_name: {joint}'
            return response

        motor = self.drivers[joint]
        cmd = request.command_type

        pos = vel = tq = temp = 0.0
        try:
            if cmd == RobStrideJointControl.Request.CMD_ENABLE:
                pos, vel, tq, temp = motor.enable_motor()
                msg = f'{joint}: enabled'

            elif cmd == RobStrideJointControl.Request.CMD_DISABLE:
                pos, vel, tq, temp = motor.disable_motor()
                msg = f'{joint}: disabled'

            elif cmd == RobStrideJointControl.Request.CMD_VELOCITY:
                pos, vel, tq, temp = motor.send_velocity_mode_command(
                    velocity_rad_s=request.velocity,
                    acceleration_rad_s2=request.acceleration if request.acceleration != 0.0 else None,
                )
                msg = f'{joint}: velocity={request.velocity} rad/s'

            elif cmd == RobStrideJointControl.Request.CMD_POSITION_PP:
                pos, vel, tq, temp = motor.pos_pp_control(
                    speed_rad_s=request.velocity,
                    acceleration_rad_s2=request.acceleration,
                    angle_rad=request.position,
                )
                msg = f'{joint}: pos_pp pos={request.position} rad'

            elif cmd == RobStrideJointControl.Request.CMD_CURRENT:
                pos, vel, tq, temp = motor.current_control(
                    iq_command=request.iq,
                    id_command=request.id,
                )
                msg = f'{joint}: iq={request.iq} A, id={request.id} A'

            elif cmd == RobStrideJointControl.Request.CMD_MOTION:
                pos, vel, tq, temp = motor.send_motion_command(
                    torque=request.torque,
                    position_rad=request.position,
                    velocity_rad_s=request.velocity,
                    kp=request.kp if request.kp != 0.0 else 0.5, #Set default PID
                    kd=request.kd if request.kd != 0.0 else 0.1,
                )
                msg = f'{joint}: motion command'

            elif cmd == RobStrideJointControl.Request.CMD_READ_STATUS:
                motor.receive_status_frame(timeout=0.01)
                pos = motor.position
                vel = motor.velocity
                tq = motor.torque
                temp = motor.temperature
                msg = f'{joint}: status read'

            else:
                raise ValueError(f'Unknown command_type: {cmd}')

            response.success = True
            response.message = msg
            response.position = float(pos)
            response.velocity = float(vel)
            response.torque = float(tq)
            response.temperature = float(temp)
            return response

        except Exception as e:
            self.get_logger().error(f'Error on joint {joint}: {e}')
            response.success = False
            response.message = f'Error: {e}'
            response.position = 0.0
            response.velocity = 0.0
            response.torque = 0.0
            response.temperature = 0.0
            return response

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for joint, motor in self.drivers.items():
            # Lightweight approach: try to update status every loop
            # (you can change this to a slower polling rate if needed)
            try:
                motor.receive_status_frame(timeout=0.0)
            except Exception:
                pass

            msg.name.append(joint)
            msg.position.append(float(getattr(motor, 'position', 0.0)))
            msg.velocity.append(float(getattr(motor, 'velocity', 0.0)))
            msg.effort.append(float(getattr(motor, 'torque', 0.0)))

        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobStrideCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
